from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_launch = PathJoinSubstitution([FindPackageShare('race_gazebo'), 'launch', 'map.launch.py'])
    urdf_path = PathJoinSubstitution([FindPackageShare('race_description'), 'urdf', 'omni_car_ros2_control.urdf'])
    rviz_config = PathJoinSubstitution([FindPackageShare('race_bringup'), 'rviz', 'race_sim.rviz'])
    bridge_config = PathJoinSubstitution([FindPackageShare('race_bringup'), 'config', 'bridge_sensors.yaml'])
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_sensor_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_omni_car',
        output='screen',
        arguments=[
            '-name', 'omni_car',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    omni_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['omni_drive_controller', '--controller-manager', '/controller_manager'],
    )

    cmd_vel_converter = Node(
        package='race_control',
        executable='twist_to_twist_stamped',
        name='twist_to_twist_stamped',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'in_topic': '/cmd_vel',
            'out_topic': '/omni_drive_controller/cmd_vel',
            'frame_id': 'base_footprint',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo server only, without GUI.'),
        DeclareLaunchArgument('paused', default_value='false', description='Start Gazebo paused.'),
        DeclareLaunchArgument('stress', default_value='false', description='Use the stress-test world.'),
        DeclareLaunchArgument(
            'render_engine',
            default_value='ogre2',
            description='Gazebo render engine passed through to race_gazebo.',
        ),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz2.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Gazebo simulation clock.'),
        # Competition start pad centre.  The car faces into the arena (-Y).
        DeclareLaunchArgument('spawn_x', default_value='8.0727', description='Robot spawn X in world/map coordinates.'),
        DeclareLaunchArgument('spawn_y', default_value='7.5312', description='Robot spawn Y in world/map coordinates.'),
        DeclareLaunchArgument('spawn_z', default_value='0.25', description='Robot spawn Z.'),
        DeclareLaunchArgument('spawn_yaw', default_value='-1.5708', description='Robot spawn yaw in radians.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_launch),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'paused': LaunchConfiguration('paused'),
                'stress': LaunchConfiguration('stress'),
                'render_engine': LaunchConfiguration('render_engine'),
            }.items(),
        ),
        robot_state_publisher,
        rviz2,
        gz_bridge,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=7.0, actions=[joint_state_spawner]),
        TimerAction(period=9.0, actions=[omni_spawner]),
        TimerAction(period=10.0, actions=[cmd_vel_converter]),
    ])
