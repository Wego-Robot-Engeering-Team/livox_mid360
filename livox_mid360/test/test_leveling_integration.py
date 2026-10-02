#!/usr/bin/env python3
"""Exercise deskew and legacy leveling with synthetic PointCloud2 and IMU."""

import math
import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField


@pytest.mark.parametrize("deskew", [True, False], ids=["deskew", "legacy"])
def test_leveling_integration(deskew):
    rclpy.init()
    node = rclpy.create_node("mid360_smoke_test")
    imu_pub = node.create_publisher(Imu, "/test/imu", qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, "/test/points", qos_profile_sensor_data)
    received = []
    orientations = []
    subscription = node.create_subscription(
        PointCloud2, "/test/leveled", received.append, qos_profile_sensor_data)
    imu_subscription = node.create_subscription(
        Imu, "/test/orientation", orientations.append, qos_profile_sensor_data)
    process = subprocess.Popen([
        os.environ["LIVOX_LEVELING_EXECUTABLE"], "--ros-args",
        "-r", "points:=/test/points", "-r", "imu:=/test/imu",
        "-r", "points_leveled:=/test/leveled", "-r", "imu_orientation:=/test/orientation",
        "-p", "initialization_samples:=5",
        "-p", f"deskew:={str(deskew).lower()}",
    ], start_new_session=True)
    try:
        deadline = time.monotonic() + 10
        while (imu_pub.get_subscription_count() == 0 or cloud_pub.get_subscription_count() == 0
               or subscription.get_publisher_count() == 0):
            if process.poll() is not None:
                raise RuntimeError("Leveling process failed to start")
            if time.monotonic() > deadline:
                raise RuntimeError("ROS discovery timed out")
            rclpy.spin_once(node, timeout_sec=0.05)
        angle = math.radians(30)

        def set_stamp(stamp, nanoseconds):
            stamp.sec, stamp.nanosec = divmod(nanoseconds, 1_000_000_000)

        def publish_imu(nanoseconds):
            msg = Imu()
            msg.header.frame_id = "livox_frame"
            set_stamp(msg.header.stamp, nanoseconds)
            msg.linear_acceleration.x = -math.sin(angle)
            msg.linear_acceleration.z = math.cos(angle)
            imu_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.015)

        def make_cloud(point_stamps, include_timestamp=deskew):
            cloud = PointCloud2()
            cloud.header.frame_id = "livox_frame"
            set_stamp(cloud.header.stamp, point_stamps[0])
            cloud.height, cloud.width = 1, 3
            cloud.point_step = 24 if include_timestamp else 16
            cloud.row_step = cloud.width * cloud.point_step
            cloud.fields = [
                PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                for name, offset in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))
            ]
            if include_timestamp:
                cloud.fields.append(PointField(
                    name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1))
            data = bytearray()
            for (x, y), stamp in zip(((1.0, 0.0), (2.0, 1.0), (-1.0, 2.0)), point_stamps):
                # A horizontal plane at z=-1 seen by a sensor pitched by +30 deg.
                raw_x = math.cos(angle) * x + math.sin(angle)
                raw_z = math.sin(angle) * x - math.cos(angle)
                data.extend(struct.pack("<ffff", raw_x, y, raw_z, 17.0))
                if include_timestamp:
                    data.extend(struct.pack("<d", float(stamp)))
            cloud.data = bytes(data)
            return cloud

        def spin_for(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)

        def wait_for_cloud(previous_count):
            deadline = time.monotonic() + 5
            while len(received) == previous_count and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert len(received) > previous_count, "No leveled cloud received"
            return received[-1]

        for i in range(20):
            publish_imu(100_000_000_000 + i * 5_000_000)
        point_stamps = [100_085_000_000, 100_090_000_000, 100_092_500_000]
        cloud = make_cloud(point_stamps)
        cloud_pub.publish(cloud)
        output = wait_for_cloud(0)
        assert output.header.frame_id == "livox_level"
        expected_stamp = max(point_stamps) if deskew else point_stamps[0]
        assert output.header.stamp.sec * 1_000_000_000 + output.header.stamp.nanosec == expected_stamp
        assert output.fields == cloud.fields
        assert output.point_step == cloud.point_step
        for offset in range(0, len(output.data), cloud.point_step):
            x, y, z, intensity = struct.unpack_from("<ffff", bytes(output.data), offset)
            assert abs(z + 1.0) < 1e-5, (x, y, z)
            assert intensity == 17.0
            assert (bytes(output.data[offset + 12:offset + cloud.point_step]) ==
                    bytes(cloud.data[offset + 12:offset + cloud.point_step]))
        assert orientations and abs(
            orientations[-1].linear_acceleration.z - math.cos(angle) * 9.80665) < 1e-6

        if deskew:
            # Timestamp-free clouds must fail instead of silently leveling.
            count = len(received)
            cloud_pub.publish(make_cloud(point_stamps, include_timestamp=False))
            spin_for(0.3)
            assert len(received) == count, "Timestamp-free cloud was silently accepted for deskew"

            # The first points are covered; the last point needs a future IMU bracket.
            future_stamps = [100_092_500_000, 100_100_000_000, 100_102_500_000]
            cloud_pub.publish(make_cloud(future_stamps))
            spin_for(0.05)
            assert len(received) == count, "Deskew extrapolated beyond the latest IMU"
            publish_imu(100_100_000_000)
            publish_imu(100_105_000_000)
            future_output = wait_for_cloud(count)
            assert future_output.header.stamp.sec == 100
            assert future_output.header.stamp.nanosec == 102_500_000

        # Unmatched future data must expire rather than use a stale orientation.
        count = len(received)
        cloud_pub.publish(make_cloud([101_085_000_000, 101_090_000_000, 101_092_500_000]))
        spin_for(0.5)
        assert len(received) == count, "Stale IMU was used for an unmatched cloud"

        # Time reset (bag loop/sensor reconnect) must initialize cleanly again.
        for i in range(20):
            publish_imu(10_000_000_000 + i * 5_000_000)
        cloud_pub.publish(make_cloud([10_085_000_000, 10_090_000_000, 10_092_500_000]))
        reset_output = wait_for_cloud(count)
        assert reset_output.header.stamp.sec == 10, "No recovery after IMU time reset"
        print(f"PASS: deskew={deskew}, tilted plane, SI conversion, timestamp coverage and time-reset recovery")
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        node.destroy_subscription(subscription)
        node.destroy_subscription(imu_subscription)
        node.destroy_node()
        rclpy.shutdown()
