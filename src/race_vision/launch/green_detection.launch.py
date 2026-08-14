from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('race_vision'), 'config', 'green_detector.yaml'])
    return LaunchDescription([
        Node(
            package='race_vision',
            executable='green_board_detector',
            name='green_board_detector',
            output='screen',
            parameters=[params],
        )
    ])
