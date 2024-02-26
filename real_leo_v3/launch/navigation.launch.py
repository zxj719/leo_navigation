from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Include SLAM Toolbox standard launch file
    launch_twist_mux = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('real_leo'), '/launch', '/twist_mux.launch.py']),
    launch_arguments={}.items(),
    )
    
    launch_move_base = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('real_leo'), '/launch', '/move_base.launch.py']),
    launch_arguments={}.items(),
    )

    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_twist_mux)
    ld.add_action(launch_move_base)

    return ld
