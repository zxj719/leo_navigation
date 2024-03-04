import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import xacro

def generate_launch_description():
    ld = LaunchDescription()
    pkg_name = 'teleop_leo'

    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav = PathJoinSubstitution([get_package_share_directory(pkg_name), 'config', 'bt_nav.yaml'])

    launch_node_joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {"dev": "/dev/input/js0"},
            {"coalesce_interval": "0.02"},
            {"autorepeat_rate": "30.0"},
        ],
    )

    launch_node_joy_teleop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="joy_teleop_node",
        remappings=[("/cmd_vel", "cmd_vel")],
        parameters=[config_bt_nav],
    )

    ld.add_action(launch_node_joy)
    ld.add_action(launch_node_joy_teleop)


    return ld