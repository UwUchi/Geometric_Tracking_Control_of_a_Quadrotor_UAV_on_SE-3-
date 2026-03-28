from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = f"{get_package_share_directory('quad_se3_py')}/rviz/quad_se3.rviz"

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('trajectory_mode', default_value='hover'),
        DeclareLaunchArgument('path_max_points', default_value='2000'),
        DeclareLaunchArgument('show_error_markers', default_value='true'),
        DeclareLaunchArgument('initial_roll_deg', default_value='0.0'),
        DeclareLaunchArgument('initial_pitch_deg', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw_deg', default_value='0.0'),
        Node(
            package='quad_se3_py',
            executable='trajectory_node',
            output='screen',
            parameters=[{
                'trajectory_mode': LaunchConfiguration('trajectory_mode'),
            }],
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
            parameters=[{
                'initial_roll_deg': LaunchConfiguration('initial_roll_deg'),
                'initial_pitch_deg': LaunchConfiguration('initial_pitch_deg'),
                'initial_yaw_deg': LaunchConfiguration('initial_yaw_deg'),
            }],
        ),
        Node(
            package='quad_se3_py',
            executable='visualization_node',
            output='screen',
            parameters=[{
                'path_max_points': LaunchConfiguration('path_max_points'),
                'show_error_markers': LaunchConfiguration('show_error_markers'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
