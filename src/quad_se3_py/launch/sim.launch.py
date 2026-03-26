from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quad_se3_py',
            executable='trajectory_node',
            output='screen',
        ),
        Node(
            package='quad_se3_py',
            executable='controller_node',
            output='screen',
        ),
        Node(
            package='quad_se3_py',
            executable='dynamics_node',
            output='screen',
        ),
    ])
