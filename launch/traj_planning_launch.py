from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='traj_planning',
            executable='rrt_planner_node',
            name='rrt_planner_node',
            output='screen',
        ),
        Node(
            package='traj_planning',
            executable='visualizer_node',
            name='visualizer_node',
            output='screen',
        ),
        Node(
            package='traj_planning',
            executable='HO_reference_finder',
            name='HO_reference_finder',
            output='screen',
        ),
        Node(
            package='traj_planning',
            executable='ACS_reference_finder',
            name='ACS_reference_finder',
            output='screen',
        ),
        Node(
            package='common_config',
            executable='parameter_publisher_node',
            name='parameter_publisher_node',
            output='screen',
        ),
    ])
