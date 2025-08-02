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
    ])
