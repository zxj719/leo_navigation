from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()
    
    pkg = get_package_share_directory('real_leo')
    
    config_topics = PathJoinSubstitution([pkg, 'config', 'twist_mux', 'topics.yaml'])
    config_locks = PathJoinSubstitution([pkg, 'config', 'twist_mux', 'locks.yaml'])
    
    node_twist_mux = Node(
      package='twist_mux',
      executable='twist_mux',
      parameters=[config_topics, config_locks],
      remappings=[('cmd_vel_out', 'cmd_vel')],
    )
    
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(node_twist_mux)
    return ld
# <launch>
#   <arg name="cmd_vel_out" default="cmd_vel" />

#   <node name="twist_mux"
#     pkg="twist_mux"
#     exec="twist_mux">
#     <remap from="cmd_vel_out" to="$(var cmd_vel_out)" />
#     <rosparam value="$(command 'load')"
#       file="$(find-pkg-share real_leo)/config/twist_mux/topics.yaml" />
#     <rosparam value="$(command 'load')"
#       file="$(find-pkg-share real_leo)/config/twist_mux/locks.yaml" />
#   </node>

# </launch>