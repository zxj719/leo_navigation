from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()
    
    pkg = get_package_share_directory('real_leo')
    # Remappings
    remappings = [
        ('/odom', '/odometry/filtered'),
        ('/cmd_vel', '/nav_vel')
    ]
    
    # Load Parameters from YAML Files
    move_base_params = PathJoinSubstitution([pkg, 'config', 'move_base', 'move_base.yaml'])
    costmap_common_params = PathJoinSubstitution([pkg, 'config', 'move_base', 'costmaps', 'costmap_common.yaml'])
    global_costmap_params = PathJoinSubstitution([pkg, 'config', 'move_base', 'costmaps', 'global_costmap.yaml'])
    local_costmap_params = PathJoinSubstitution([pkg, 'config', 'move_base', 'costmaps', 'local_costmap.yaml'])
    local_planner_params = PathJoinSubstitution([pkg, 'config', 'move_base', 'planners', 'local_planner.yaml'])
    global_planner_params = PathJoinSubstitution([pkg, 'config', 'move_base', 'planners', 'global_planner.yaml'])

    # Include move_base node
    node_move_base = Node(
        package='move_base',
        executable='move_base',
        name='move_base',
        parameters=[move_base_params],
        remappings=remappings
    )
    
    node_global_costmap = Node(
        package='move_base',
        executable='move_base',
        namespace="global_costmap",
        parameters=[costmap_common_params],
        remappings=remappings
    )
    
    node_local_costmap = Node(
        package='move_base',
        executable='move_base',
        namespace="local_costmap",
        parameters=[costmap_common_params],
        remappings=remappings
    )
    
    node_costmap = Node(
        package='move_base',
        executable='move_base',
        parameters=[global_costmap_params, local_costmap_params, local_planner_params, global_planner_params],
        remappings=remappings
    )

    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(node_move_base)
    ld.add_action(node_global_costmap)
    ld.add_action(node_local_costmap)
    ld.add_action(node_costmap)

    return ld



# <launch>
#   <remap from="odom" to="odometry/filtered" />
#   <remap from="cmd_vel" to="nav_vel" />

#   <node name="move_base"
#     pkg="move_base"
#     exec="move_base"
#     clear_params="true">
#     <rosparam value="$(command 'load')"
#       file="$(find-pkg-share real_leo)/config/move_base/move_base.yaml" />

#     <!-- costmaps -->
#     <rosparam value="$(command 'load')"
#       ns="global_costmap"
#       file="$(find-pkg-share real_leo)/config/move_base/costmaps/costmap_common.yaml" />
#     <rosparam value="$(command 'load')"
#       ns="local_costmap"
#       file="$(find-pkg-share real_leo)/config/move_base/costmaps/costmap_common.yaml" />
#     <rosparam value="$(command 'load')"
#       file="$(find-pkg-share real_leo)/config/move_base/costmaps/local_costmap.yaml" />
#     <rosparam value="$(command 'load')"
#       file="$(find-pkg-share real_leo)/config/move_base/costmaps/global_costmap.yaml" />

#     <!-- planners -->
#     <rosparam value="$(command 'load')"
#       file="$(find-pkg-share real_leo)/config/move_base/planners/local_planner.yaml" />
#     <rosparam value="$(command 'load')"
#       file="$(find-pkg-share real_leo)/config/move_base/planners/global_planner.yaml" />
#   </node>
# </launch>