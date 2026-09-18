"""Compatibility entry point for the rules-compliant autonomous task.

This launch no longer starts AMCL/Nav2 and never sends a fixed map goal.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    competition = PathJoinSubstitution([
        FindPackageShare('race_navigation'), 'launch', 'competition.launch.py'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('stress', default_value='false',
                              description='Load the dynamic-obstacle world instead of the nominal arena.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(competition),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'rviz': LaunchConfiguration('rviz'),
                'stress': LaunchConfiguration('stress'),
            }.items(),
        ),
    ])
