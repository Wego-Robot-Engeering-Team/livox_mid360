from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file = Path(get_package_share_directory("livox_mid360")) / "launch" / "mid360.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument("leveling", default_value="true"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)),
                                 launch_arguments={"leveling": LaunchConfiguration("leveling")}.items()),
    ])
