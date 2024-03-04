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


    launch_key_teleop = Node(
            name="key_teleop_node",
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            output='screen',
            prefix=["xterm -e"],
            parameters=[
                {'speed': '0.4'},
                {'turn': '1.0'},
                {'repeat_rate': '10.0'},
                {'key_timeout': '0.3'},],
            remappings=[("/cmd_vel", "cmd_vel")],
        )
    
    ld.add_action(launch_key_teleop)

    return ld