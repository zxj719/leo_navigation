from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()
    
    pkg = get_package_share_directory('real_leo')
    params = PathJoinSubstitution([pkg, 'config', 'slam_gmapping.yaml'])
    
    launch_gmapping = Node(
      name='gmapping',
      package='gmapping',
      executable='slam_gmapping',
      parameters=[params]
    )
    
    
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_gmapping)
    return ld


# <launch>
#   <node name="gmapping"
#     pkg="gmapping"
#     exec="slam_gmapping">
#     <rosparam value="$(command 'load')"
#       file="$(find-pkg-share real_leo)/config/slam_gmapping.yaml" />
#   </node>
# </launch>