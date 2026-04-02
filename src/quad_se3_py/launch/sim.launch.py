from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('trajectory_mode', default_value='hover'),
        DeclareLaunchArgument('initial_roll_deg', default_value='0.0'),
        DeclareLaunchArgument('initial_pitch_deg', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw_deg', default_value='0.0'),
        Node(
            package='quad_se3_py',
            executable='trajectory_node',
            output='screen',
            parameters=[{
                'trajectory_mode': LaunchConfiguration('trajectory_mode'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='quad_se3_py',
            executable='controller_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='quad_se3_py',
            executable='dynamics_node',
            output='screen',
            parameters=[{
                'initial_roll_deg': LaunchConfiguration('initial_roll_deg'),
                'initial_pitch_deg': LaunchConfiguration('initial_pitch_deg'),
                'initial_yaw_deg': LaunchConfiguration('initial_yaw_deg'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
