from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare the "use_wii_controller" parameter
        DeclareLaunchArgument(
            'use_wii_controller',
            default_value='false',
            description='Set to true to use the Wii controller'
        ),

        Node(
            package='traj_planning',
            executable='rrt_planner_node',
            name='rrt_planner_node',
            output='screen',
            parameters=[{'use_wii_controller': LaunchConfiguration('use_wii_controller')}]
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
