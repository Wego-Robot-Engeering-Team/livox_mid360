#!/usr/bin/env python3
"""Exercise the real leveling process with tilted synthetic PointCloud2 and IMU."""

import math
import os
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField


def main():
    rclpy.init()
    node = rclpy.create_node("mid360_smoke_test")
    imu_pub = node.create_publisher(Imu, "/test/imu", qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, "/test/points", qos_profile_sensor_data)
    received = []
    orientations = []
    subscription = node.create_subscription(PointCloud2, "/test/leveled", received.append, qos_profile_sensor_data)
    imu_subscription = node.create_subscription(Imu, "/test/orientation", orientations.append, qos_profile_sensor_data)
    process = subprocess.Popen([
        "ros2", "run", "livox_mid360", "cloud_leveling_node", "--ros-args",
        "-r", "points:=/test/points", "-r", "imu:=/test/imu",
        "-r", "points_leveled:=/test/leveled", "-r", "imu_orientation:=/test/orientation",
        "-p", "initialization_samples:=5",
    ], start_new_session=True)
    try:
        deadline = time.monotonic() + 10
        while (imu_pub.get_subscription_count() == 0 or cloud_pub.get_subscription_count() == 0):
            if process.poll() is not None:
                raise RuntimeError("Leveling process failed to start")
            if time.monotonic() > deadline:
                raise RuntimeError("ROS discovery timed out")
            rclpy.spin_once(node, timeout_sec=0.05)
        angle = math.radians(30)

        def publish_imu(seconds):
            msg = Imu()
            msg.header.frame_id = "livox_frame"
            msg.header.stamp.sec = int(seconds)
            msg.header.stamp.nanosec = round((seconds - int(seconds)) * 1e9)
            msg.linear_acceleration.x = -math.sin(angle)
            msg.linear_acceleration.z = math.cos(angle)
            imu_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.015)

        for i in range(20):
            publish_imu(100.0 + i * 0.005)
        cloud = PointCloud2()
        cloud.header.frame_id = "livox_frame"
        cloud.header.stamp.sec = 100
        cloud.header.stamp.nanosec = 92500000  # Between the two last IMU samples.
        cloud.height, cloud.width = 1, 3
        cloud.point_step, cloud.row_step = 16, 48
        cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                        for name, offset in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))]
        data = bytearray()
        for x, y in ((1.0, 0.0), (2.0, 1.0), (-1.0, 2.0)):
            # Express a horizontal plane at z=-1 in a sensor pitched by +30 deg.
            raw_x = math.cos(angle) * x + math.sin(angle)
            raw_z = math.sin(angle) * x - math.cos(angle)
            data.extend(struct.pack("<ffff", raw_x, y, raw_z, 17.0))
        cloud.data = bytes(data)
        cloud_pub.publish(cloud)
        deadline = time.monotonic() + 5
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert received, "No leveled cloud received"
        output = received[-1]
        assert output.header.frame_id == "livox_level"
        assert output.header.stamp == cloud.header.stamp
        for offset in range(0, len(output.data), 16):
            x, y, z, intensity = struct.unpack_from("<ffff", bytes(output.data), offset)
            assert abs(z + 1.0) < 1e-5, (x, y, z)
            assert intensity == 17.0
        assert orientations and abs(orientations[-1].linear_acceleration.z - math.cos(angle) * 9.80665) < 1e-6

        # Unmatched future data must expire rather than use a stale orientation.
        count = len(received)
        cloud.header.stamp.sec = 101
        cloud_pub.publish(cloud)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert len(received) == count, "Stale IMU was used for an unmatched cloud"

        # Time reset (bag loop/sensor reconnect) must initialize cleanly again.
        for i in range(20):
            publish_imu(10.0 + i * 0.005)
        cloud.header.stamp.sec = 10
        cloud_pub.publish(cloud)
        deadline = time.monotonic() + 5
        while len(received) == count and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(received) > count, "No recovery after IMU time reset"
        print("PASS: tilted ground leveling, timestamp interpolation, SI conversion, stale rejection and time-reset recovery")
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


if __name__ == "__main__":
    main()
