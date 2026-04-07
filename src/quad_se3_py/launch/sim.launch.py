from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('trajectory_mode', default_value='hover'),
        DeclareLaunchArgument(
            'trajectory_start_time_sec',
            default_value='0.0',
        ),
        DeclareLaunchArgument('reference_time_offset_sec', default_value='0.0'),
        DeclareLaunchArgument('initial_roll_deg', default_value='0.0'),
        DeclareLaunchArgument('initial_pitch_deg', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw_deg', default_value='0.0'),
        Node(
            package='quad_se3_py',
            executable='sim_node',
            output='screen',
            parameters=[{
                'trajectory_mode': LaunchConfiguration('trajectory_mode'),
                'trajectory_start_time_sec': LaunchConfiguration(
                    'trajectory_start_time_sec'
                ),
                'reference_time_offset_sec': LaunchConfiguration(
                    'reference_time_offset_sec'
                ),
                'initial_roll_deg': LaunchConfiguration('initial_roll_deg'),
                'initial_pitch_deg': LaunchConfiguration('initial_pitch_deg'),
                'initial_yaw_deg': LaunchConfiguration('initial_yaw_deg'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
