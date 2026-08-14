"""Start the race simulation, static-map AMCL localization and Nav2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    navigation_share = FindPackageShare('race_navigation')
    bringup_share = FindPackageShare('race_bringup')
    nav2_share = FindPackageShare('nav2_bringup')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'sim_ros2_control.launch.py'])
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'rviz': 'false',
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, 'launch', 'localization_launch.py'])
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([navigation_share, 'launch', 'navigation_core.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    rviz = Node(
        package='race_bringup',
        executable='rviz_compat',
        name='rviz2',
        output='screen',
        # Do not call this argument "rviz": the included simulation launch
        # also has an rviz argument which is deliberately set to false.
        condition=IfCondition(LaunchConfiguration('nav_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('nav_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='8.0727'),
        DeclareLaunchArgument('spawn_y', default_value='7.5312'),
        DeclareLaunchArgument('spawn_z', default_value='0.25'),
        DeclareLaunchArgument('spawn_yaw', default_value='-1.5708'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([navigation_share, 'maps', 'race_map.yaml']),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([navigation_share, 'config', 'nav2_params.yaml']),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([navigation_share, 'rviz', 'nav2_default_view.rviz']),
        ),
        simulation,
        # Gazebo, bridges and ros2_control must exist before lifecycle nodes configure.
        TimerAction(period=12.0, actions=[localization, navigation]),
        TimerAction(period=18.0, actions=[rviz]),
    ])
