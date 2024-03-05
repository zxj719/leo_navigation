from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import xacro

def generate_launch_description():
    ld = LaunchDescription()

    # Include SLAM Toolbox standard launch file
    launch_slamtoolbox = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
    launch_arguments={}.items(),
    )

    launch_map_saver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('nav2_map_server'), '/launch', '/map_saver_server.launch.py']),
    launch_arguments={}.items(),
    )

    ld.add_action(launch_slamtoolbox)
    ld.add_action(launch_map_saver)

    return ld