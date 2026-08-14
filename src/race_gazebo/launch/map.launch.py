import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_gazebo(context, *args, **kwargs):
    pkg_share = get_package_share_directory('race_gazebo')
    models_dir = os.path.join(pkg_share, 'models')
    race_description_share = get_package_share_directory('race_description')
    race_description_resource_root = os.path.dirname(race_description_share)
    stress = LaunchConfiguration('stress').perform(context).lower() in ('1', 'true', 'yes', 'on')
    world_name = 'competition_stress.world' if stress else 'competition_world.world'
    world_path = os.path.join(pkg_share, 'worlds', world_name)

    headless = LaunchConfiguration('headless').perform(context).lower() in ('1', 'true', 'yes', 'on')
    paused = LaunchConfiguration('paused').perform(context).lower() in ('1', 'true', 'yes', 'on')
    render_engine = LaunchConfiguration('render_engine').perform(context)

    gz_args = []
    if headless:
        gz_args.append('-s')
        # Server-only mode still needs an offscreen render target for camera
        # and GPU lidar sensors.
        gz_args.append('--headless-rendering')
    if not paused:
        gz_args.append('-r')
    gz_args.extend(['--render-engine', render_engine])
    gz_args.append(world_path)

    gz_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    ])

    return [
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_dir),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=race_description_resource_root),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': ' '.join(gz_args)}.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo server only, without GUI.'),
        DeclareLaunchArgument('paused', default_value='false', description='Start Gazebo paused.'),
        DeclareLaunchArgument(
            'stress',
            default_value='false',
            description='Enable uneven lighting, rough traction patches and a moving obstacle.',
        ),
        DeclareLaunchArgument(
            'render_engine',
            default_value='ogre2',
            description='Gazebo render engine.',
        ),
        # VMware SVGA exposes OpenGL to RViz but Ogre2/EGL fails on the virtual
        # DRI device. Mesa llvmpipe keeps Ogre2, gpu_lidar and the GUI working.
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
        SetEnvironmentVariable(name='MESA_GL_VERSION_OVERRIDE', value='4.5'),
        OpaqueFunction(function=_launch_gazebo),
        # The Gazebo process has inherited the settings above. Restore the
        # launch environment so RViz uses VMware's working OpenGL 4.3 path.
        UnsetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE'),
        UnsetEnvironmentVariable(name='MESA_GL_VERSION_OVERRIDE'),
    ])
