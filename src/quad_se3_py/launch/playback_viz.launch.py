from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_rviz_config = (
        f"{get_package_share_directory('quad_se3_py')}/rviz/quad_recording.rviz"
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('trajectory_mode', default_value='hover'),
        DeclareLaunchArgument('trajectory_start_time_sec', default_value='0.0'),
        DeclareLaunchArgument('reference_time_offset_sec', default_value='0.0'),
        DeclareLaunchArgument('path_max_points', default_value='2000'),
        DeclareLaunchArgument('show_error_markers', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        Node(
            package='quad_se3_py',
            executable='visualization_node',
            output='screen',
            parameters=[{
                'path_max_points': LaunchConfiguration('path_max_points'),
                'show_error_markers': LaunchConfiguration('show_error_markers'),
                'trajectory_mode': LaunchConfiguration('trajectory_mode'),
                'trajectory_start_time_sec': LaunchConfiguration(
                    'trajectory_start_time_sec'
                ),
                'reference_time_offset_sec': LaunchConfiguration(
                    'reference_time_offset_sec'
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
