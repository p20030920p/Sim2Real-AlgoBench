"""Complete legal one-button race: known map, AMCL, Nav2 search and visual finish."""

import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Where the algorithm-swapped parameter file is generated. Outside the install
# tree on purpose: it is a per-run artefact, not something to package.
GENERATED_PARAMS_DIR = '/tmp/race_nav2_params'


def _params_for_algorithm(algorithm, base_params):
    """Generate a nav2_params file whose global planner is `algorithm`.

    Only the planner_server block differs from the race baseline, so two runs
    with different algorithms share the costmap, controller, AMCL settings and
    behaviour tree, and are therefore comparable. Returns the base file
    unchanged when no algorithm was requested.
    """
    if not algorithm:
        return base_params

    os.makedirs(GENERATED_PARAMS_DIR, exist_ok=True)
    out = os.path.join(GENERATED_PARAMS_DIR, f'nav2_{algorithm}.yaml')
    # Resolve through symlinks: this file is usually loaded from the install
    # tree, but the helper it needs lives in the repository.
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))
    tool = os.path.join(repo_root, 'tools', 'make_planner_params.py')
    if not os.path.exists(tool):
        raise RuntimeError(
            f'cannot switch planner: {tool} is missing '
            '(it lives in the repository, not in the install tree)')

    subprocess.run(
        ['python3', tool, '--algorithm', algorithm, '--out', out,
         '--base', base_params],
        check=True, capture_output=True, text=True)
    return out


def generate_launch_description():
    nav_share = FindPackageShare('race_navigation')
    bringup_share = FindPackageShare('race_bringup')
    nav2_share = FindPackageShare('nav2_bringup')
    control_share = FindPackageShare('race_control')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    algorithm = os.environ.get('RACE_GLOBAL_PLANNER', '')
    params = _params_for_algorithm(
        algorithm, os.path.normpath(os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            '..', 'config', 'nav2_params.yaml')))
    if algorithm:
        print(f'[competition] global planner overridden to {algorithm!r}; '
              f'params: {params}')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'sim_ros2_control.launch.py'])),
        launch_arguments={
            'headless': LaunchConfiguration('headless'), 'rviz': 'false',
            'stress': LaunchConfiguration('stress'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items())

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, 'launch', 'localization_launch.py'])),
        launch_arguments={
            'map': map_file, 'use_sim_time': use_sim_time, 'params_file': params,
            'autostart': 'true', 'use_composition': 'False', 'use_respawn': 'False',
        }.items())

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav_share, 'launch', 'navigation_core.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time, 'params_file': params, 'autostart': 'true',
        }.items())

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('race_vision'), 'launch', 'green_detection.launch.py'])))

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'nav_rviz', default_value='true',
            description='Start the Nav2 RViz window. Kept separate from the simulation rviz argument.'),
        DeclareLaunchArgument('stress', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='8.0727'),
        DeclareLaunchArgument('spawn_y', default_value='7.5312'),
        DeclareLaunchArgument('spawn_z', default_value='0.25'),
        DeclareLaunchArgument('spawn_yaw', default_value='-1.5708'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([
            nav_share, 'maps', 'race_map.yaml'])),
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([
            nav_share, 'rviz', 'nav2_default_view.rviz'])),
        simulation,
        Node(package='race_control', executable='twist_priority_mux',
             name='twist_priority_mux', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='race_control', executable='race_metrics',
             name='race_metrics', output='screen',
             parameters=[{
                 'use_sim_time': use_sim_time,
                 # Default in the node is an absolute path from the machine it
                 # was written on; point it at this repository instead so the
                 # reports land next to the code that produced them.
                 'report_dir': os.environ.get('RACE_REPORT_DIR', os.path.normpath(
                     os.path.join(
                         os.path.dirname(os.path.realpath(__file__)),
                         '..', '..', '..', '..', 'reports'))),
             }]),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='gz_moving_obstacle_bridge', output='screen',
            arguments=[
                '/dynamic_obstacle/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/dynamic_obstacle_2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            ], condition=IfCondition(LaunchConfiguration('stress'))),
        TimerAction(period=13.0, condition=IfCondition(LaunchConfiguration('stress')), actions=[
            Node(package='race_control', executable='moving_obstacle',
                 name='moving_obstacle', output='screen', parameters=[{
                     'use_sim_time': use_sim_time,
                     'command_topic': '/dynamic_obstacle/cmd_pos',
                     'amplitude': 1.6, 'period_seconds': 8.0}]),
            Node(package='race_control', executable='moving_obstacle',
                 name='moving_obstacle_upper_corridor', output='screen', parameters=[{
                     'use_sim_time': use_sim_time,
                     'command_topic': '/dynamic_obstacle_2/cmd_pos',
                     'amplitude': 1.4, 'period_seconds': 7.0}]),
        ]),
        TimerAction(period=10.0, actions=[vision]),
        TimerAction(period=12.0, actions=[localization, navigation]),
        TimerAction(period=18.0, condition=IfCondition(LaunchConfiguration('nav_rviz')), actions=[
            Node(package='race_bringup', executable='rviz_compat', name='rviz2',
                 output='screen', arguments=['-d', LaunchConfiguration('rviz_config')],
                 parameters=[{'use_sim_time': use_sim_time}])]),
        TimerAction(period=20.0, actions=[
            Node(package='race_control', executable='map_search_autonomy',
                 name='map_search_autonomy', output='screen',
                 parameters=[PathJoinSubstitution([
                     control_share, 'config', 'map_search_autonomy.yaml'])])]),
    ])
