from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_launch = PathJoinSubstitution([
        FindPackageShare('race_bringup'),
        'launch',
        'sim_ros2_control.launch.py',
    ])
    slam_launch = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'slam_launch.py'])
    nav_launch = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
    slam_params = PathJoinSubstitution([FindPackageShare('race_navigation'), 'config', 'slam_toolbox.yaml'])
    nav_params = PathJoinSubstitution([FindPackageShare('race_navigation'), 'config', 'nav2_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'paused': 'false',
                'rviz': LaunchConfiguration('rviz'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        TimerAction(
            period=12.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': slam_params,
                }.items(),
            )],
        ),
        TimerAction(
            period=16.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': nav_params,
                    'autostart': 'true',
                }.items(),
            )],
        ),
    ])
