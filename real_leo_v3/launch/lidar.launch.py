from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()
    
    launch_rplidar = Node(
      name='rplidar_node',
      package='rplidar_ros',
      executable='rplidarNode',
      output = 'screen',
      parameters=[{'serial_port':'/dev/lidar', 'frame_id':'laser_frame'}]
    )
    
    
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_rplidar)
    return ld
