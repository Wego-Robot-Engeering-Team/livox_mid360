"""MID-360 acquisition, optional gravity leveling and RViz."""

import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ("true", "false", "1", "0"):
        raise ValueError(f"{name} must be true or false")
    return value in ("true", "1")


def _setup(context):
    share = Path(get_package_share_directory("livox_mid360"))
    value = lambda name: LaunchConfiguration(name).perform(context)
    leveling = _boolean(context, "leveling")
    actions = []
    if _boolean(context, "driver"):
        config_path = str(Path(value("config_file")).expanduser().resolve(strict=True))
        if leveling:
            config = json.loads(Path(config_path).read_text())
            # The IMU remains in livox_frame; rotating points inside the driver
            # would break the shared coordinate convention used by our filter.
            for lidar in config["lidar_configs"]:
                extrinsic = lidar.get("extrinsic_parameter", {})
                if any(float(extrinsic.get(axis, 0)) != 0 for axis in ("roll", "pitch", "yaw", "x", "y", "z")):
                    raise ValueError("Leveling requires zero driver extrinsic_parameter values")
        frequency = float(value("publish_freq"))
        if not math.isfinite(frequency) or not 0 < frequency <= 100:
            raise ValueError("publish_freq must be in (0, 100]")
        actions.append(Node(
            package="livox_ros_driver2", executable="livox_ros_driver2_node",
            name="livox_lidar_publisher", output="screen",
            parameters=[{
                "xfer_format": 0, "multi_topic": 0, "data_src": 0,
                "publish_freq": frequency, "output_data_type": 0,
                "frame_id": "livox_frame", "user_config_path": config_path,
                "lvx_file_path": "", "cmdline_input_bd_code": "livox0000000001",
                "use_sim_time": _boolean(context, "use_sim_time"),
            }],
            remappings=[("livox/lidar", value("points_topic")), ("livox/imu", value("imu_topic"))],
        ))
    acceleration_scale = float(value("acceleration_scale"))
    if not math.isfinite(acceleration_scale) or acceleration_scale <= 0:
        raise ValueError("acceleration_scale must be positive and finite")
    common = {"use_sim_time": _boolean(context, "use_sim_time")}
    if leveling:
        actions.append(Node(
            package="livox_mid360", executable="cloud_leveling_node",
            name="cloud_leveling_node", output="screen",
            parameters=[value("leveling_config"), common,
                        {"acceleration_scale": acceleration_scale, "deskew": _boolean(context, "deskew")}],
            remappings=[("points", value("points_topic")), ("imu", value("imu_topic")),
                        ("points_leveled", value("leveled_topic")),
                        ("imu_orientation", "/livox/imu_orientation")],
        ))
    if _boolean(context, "imu_visualization"):
        actions.append(Node(
            package="livox_mid360", executable="imu_visualizer_node",
            name="imu_visualizer_node", output="screen",
            parameters=[common, {"acceleration_scale": acceleration_scale}],
            remappings=[("imu", value("imu_topic")), ("imu_markers", "/livox/imu_markers")],
        ))
    if _boolean(context, "rviz"):
        rviz_file = value("rviz_config") or str(
            share / "rviz" / ("leveled.rviz" if leveling else "raw.rviz"))
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2",
                            arguments=["-d", rviz_file], parameters=[common], output="screen"))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("livox_mid360"))
    defaults = [
        ("leveling", "false", "Run the separate cloud leveling process"),
        ("deskew", "true", "Correct rotation during each scan using per-point absolute timestamps"),
        ("driver", "true", "Start hardware driver; false for rosbag replay"),
        ("rviz", "true", "Start RViz"),
        ("imu_visualization", LaunchConfiguration("rviz"), "Publish IMU arrows and numeric values"),
        ("config_file", str(share / "config" / "MID360_config.json"), "Livox SDK configuration file"),
        ("publish_freq", "10.0", "Point cloud publication frequency in Hz"),
        ("points_topic", "/livox/lidar", "Raw PointCloud2 topic"),
        ("imu_topic", "/livox/imu", "Raw IMU topic"),
        ("leveled_topic", "/livox/points_leveled", "Leveled PointCloud2 topic"),
        ("acceleration_scale", "9.80665", "Raw acceleration to m/s^2 multiplier"),
        ("leveling_config", str(share / "config" / "leveling.yaml"), "Leveling ROS parameter YAML"),
        ("rviz_config", "", "Optional custom RViz config for remapped topics/frames"),
        ("use_sim_time", "false", "Use clock from bag replay"),
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in defaults],
        OpaqueFunction(function=_setup),
    ])
