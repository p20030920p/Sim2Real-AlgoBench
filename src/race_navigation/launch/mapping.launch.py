from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_launch = PathJoinSubstitution([
        FindPackageShare('race_bringup'),
        'launch',
        'sim_ros2_control.launch.py',
    ])
    slam_launch = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'slam_launch.py'])
    slam_params = PathJoinSubstitution([FindPackageShare('race_navigation'), 'config', 'slam_toolbox.yaml'])
    rviz_config = PathJoinSubstitution([FindPackageShare('race_bringup'), 'rviz', 'race_sim.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # Keep the outer RViz choice separate from race_bringup's inner
        # argument, which is intentionally forced false below.
        DeclareLaunchArgument('mapping_rviz', default_value=LaunchConfiguration('rviz')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'paused': 'false',
                # RViz starts separately below, after the simulation TF tree and
                # SLAM lifecycle node are active. This avoids startup-only stale
                # timestamp / empty-map errors in its message filters.
                'rviz': 'false',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        TimerAction(
            # The ros2_control controller and cmd_vel adapter are started at
            # 9 s and 10 s by sim_ros2_control.launch.py. Start SLAM after
            # those components have had time to publish odom -> base_footprint.
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
            period=15.0,
            actions=[Node(
                package='race_bringup',
                executable='rviz_compat',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                condition=IfCondition(LaunchConfiguration('mapping_rviz')),
            )],
        ),
    ])
