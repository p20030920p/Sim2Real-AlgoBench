# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Standalone algorithm benchmark harness.

Brings up a map server and a planner server with *every* algorithm in
algo_registry.yaml registered at once, then prints the index table. Useful on
its own for comparing algorithms on the race map without running the whole race
scenario, and it is the same parameter block that Competition.launch.py uses.

    ros2 launch algo_bringup algo_planner_bench.launch.py
    ros2 launch algo_bringup algo_planner_bench.launch.py planner_index:=4

Then, to plan with a specific algorithm, call ComputePathToPose with its
planner_id — any registered id works, not just the active one:

    ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \\
      "{goal: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0}}}}" \\
      --feedback
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

# Installed as a Python package by this package's CMakeLists, so it is already
# on PYTHONPATH once the workspace is sourced.
from algo_bringup import registry as registry_mod


def _bringup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("algo_bringup")
    race_share = get_package_share_directory("race_navigation")

    registry_path = LaunchConfiguration("registry").perform(context)
    if not registry_path:
        registry_path = os.path.join(bringup_share, "config", "algo_registry.yaml")

    planner_index = LaunchConfiguration("planner_index").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    map_yaml = LaunchConfiguration("map").perform(context)
    if not map_yaml:
        map_yaml = os.path.join(race_share, "maps", "race_map.yaml")

    registry = registry_mod.load_registry(registry_path)

    # A launch argument can override the index without editing the file.
    if planner_index != "":
        registry.setdefault("active", {})["planner"] = int(planner_index)

    print(registry_mod.describe(registry))

    bt_dir = os.path.join("/tmp", "algo_bringup_bt")
    bt_map = registry_mod.write_behavior_trees(registry, bt_dir)
    print(f"[algo_bringup] behaviour trees written to {bt_dir}")
    for index, path in sorted(bt_map.items()):
        print(f"[algo_bringup]   index {index:>2} -> {os.path.basename(path)}")
    print()

    planner_params = registry_mod.planner_server_params(registry)

    global_costmap = {
        "global_costmap": {
            "global_costmap": {
                "ros__parameters": {
                    "update_frequency": 1.0,
                    "publish_frequency": 1.0,
                    "frame_id": "map",
                    "robot_base_frame": "base_footprint",
                    "resolution": 0.05,
                    "track_unknown_space": False,
                    "use_sim_time": use_sim_time,
                    "plugins": ["static_layer", "obstacle_layer", "inflation_layer"],
                    "static_layer": {
                        "plugin": "nav2_costmap_2d::StaticLayer",
                        "map_subscribe_transient_local": True,
                    },
                    "obstacle_layer": {
                        "plugin": "nav2_costmap_2d::ObstacleLayer",
                        "enabled": True,
                        "observation_sources": "scan",
                        "scan": {
                            "topic": "/scan",
                            "max_obstacle_height": 2.0,
                            "clearing": True,
                            "marking": True,
                            "data_type": "LaserScan",
                        },
                    },
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "cost_scaling_factor": 3.0,
                        "inflation_radius": 0.35,
                    },
                }
            }
        }
    }

    planner_server = LifecycleNode(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace="",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            planner_params,
            global_costmap,
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_planner",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server", "planner_server"]},
        ],
    )

    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"yaml_filename": map_yaml},
            {"topic_name": "map"},
            {"frame_id": "map"},
        ],
    )

    # The costmap refuses to activate until map -> robot_base_frame exists. In
    # the full race stack AMCL and the odometry controller publish that chain;
    # for a standalone bench there is no robot, so publish identity transforms
    # for both candidate base frames. This is what makes the harness usable
    # without launching Gazebo.
    def _static_tf(parent, child):
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"bench_static_tf_{child}",
            output="log",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", parent, "--child-frame-id", child,
            ],
        )

    return [
        map_server,
        _static_tf("map", "base_footprint"),
        _static_tf("base_footprint", "base_link"),
        planner_server,
        lifecycle_manager,
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory("algo_bringup")

    return LaunchDescription([
        DeclareLaunchArgument(
            "registry",
            default_value=os.path.join(bringup_share, "config", "algo_registry.yaml"),
            description="Algorithm registry YAML — the algorithm index lives here.",
        ),
        DeclareLaunchArgument(
            "planner_index",
            default_value="",
            description="Override active.planner for this run only.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Map YAML; defaults to the race map from race_navigation.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running against Gazebo.",
        ),
        OpaqueFunction(function=_bringup),
    ])
