#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Ask every registered planner for a path and draw them all in RViz.

Each planner is requested through ComputePathToPose with its own planner_id, so
what RViz shows is what the plugin actually returned, not a redrawing of it.
Paths are published as a MarkerArray with one colour per algorithm, which is
what makes the comparison readable in a single view.

Usage:
    ros2 run algo_bringup ...            # or: python3 tools/publish_algo_paths.py
    python3 tools/publish_algo_paths.py --start -3.0 -5.0 --goal 10.0 7.0
"""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src",
                                "algo_bringup"))
try:
    from algo_bringup import registry as registry_mod
except Exception:                                     # allow standalone use
    registry_mod = None

try:
    from nav2_msgs.action import ComputePathToPose
except ImportError:                                   # pragma: no cover
    ComputePathToPose = None

# One colour per algorithm, ordered so neighbouring panels stay distinguishable.
COLOURS = [
    (0.90, 0.10, 0.10), (0.10, 0.45, 0.90), (0.10, 0.65, 0.25),
    (0.95, 0.55, 0.05), (0.60, 0.20, 0.75), (0.05, 0.70, 0.70),
    (0.85, 0.20, 0.55), (0.45, 0.45, 0.45),
]


class PathCollector(Node):
    def __init__(self, planner_ids, start, goal, settle):
        super().__init__("algo_path_publisher")
        self.pub = self.create_publisher(MarkerArray, "/algo_paths", 1)
        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.planner_ids = planner_ids
        self.start = start
        self.goal = goal
        self.settle = settle
        self.markers = MarkerArray()

    def make_marker(self, name, poses, index):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "algo_paths"
        marker.id = index
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.06
        colour = COLOURS[index % len(COLOURS)]
        marker.color = ColorRGBA(r=colour[0], g=colour[1], b=colour[2], a=1.0)
        marker.pose.orientation.w = 1.0
        for p in poses:
            marker.points.append(Point(x=p[0], y=p[1], z=0.05))
        return marker

    def request(self, planner_id):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("compute_path_to_pose action not available")
            return None

        goal = ComputePathToPose.Goal()
        goal.start.header.frame_id = "map"
        goal.start.pose.position.x = self.start[0]
        goal.start.pose.position.y = self.start[1]
        goal.start.pose.orientation.w = 1.0
        goal.goal.header.frame_id = "map"
        goal.goal.pose.position.x = self.goal[0]
        goal.goal.pose.position.y = self.goal[1]
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = planner_id
        goal.use_start = True

        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f"{planner_id}: goal rejected")
            return None

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=25.0)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().warn(f"{planner_id}: no result")
            return None

        path = wrapped.result.path
        if not path.poses:
            self.get_logger().warn(f"{planner_id}: empty path")
            return None
        return [(p.pose.position.x, p.pose.position.y) for p in path.poses]

    def run(self):
        # Let RViz finish loading before the first path arrives.
        time.sleep(3.0)
        drawn = 0
        for planner_id in self.planner_ids:
            poses = self.request(planner_id)
            if poses is None:
                self.get_logger().warn(f"{planner_id}: skipped")
                continue
            self.markers.markers.append(self.make_marker(planner_id, poses, drawn))
            drawn += 1
            self.get_logger().info(f"{planner_id}: {len(poses)} poses")
            # Publish repeatedly so a late-joining RViz still receives them.
            for _ in range(int(self.settle * 4)):
                self.pub.publish(self.markers)
                time.sleep(0.25)

        for _ in range(int(self.settle * 4)):
            self.pub.publish(self.markers)
            time.sleep(0.25)
        self.get_logger().info(f"published {drawn} paths on /algo_paths")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", nargs=2, type=float, default=[-3.0, -5.0])
    ap.add_argument("--goal", nargs=2, type=float, default=[10.0, 7.0])
    ap.add_argument("--settle", type=float, default=2.0,
                    help="seconds to hold each path on screen")
    ap.add_argument("--registry", default=None)
    ap.add_argument("--skip", nargs="*", default=["jps"],
                    help="planner ids or algorithm names to leave out")
    args = ap.parse_args()

    planner_ids = None
    if registry_mod is not None:
        reg = registry_mod.load_registry(args.registry or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "src", "algo_bringup",
            "config", "algo_registry.yaml"))
        planner_ids = [e["id"] for e in registry_mod.planners(reg)
                       if e.get("algorithm") not in args.skip]

    if not planner_ids:
        planner_ids = ["P1_astar"]

    rclpy.init()
    node = PathCollector(planner_ids, tuple(args.start), tuple(args.goal), args.settle)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
