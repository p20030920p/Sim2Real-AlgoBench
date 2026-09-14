#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Drive the simulated car along a path produced by algo_core.

This is a kinematics-level follower, not the competition stack: it takes the
planned path, reads the car's pose from odometry, and publishes `/cmd_vel`
towards the next waypoint. The motion is real — Gazebo physics, the
omni_drive_controller, the same `/cmd_vel` chain the race uses — but the
decision of where to go comes from here rather than from Nav2, so a recording
made with it demonstrates the planners' output, not the full autonomy stack.

Usage:
    python3 tools/drive_path.py --dump /tmp/plan.bin --algorithm astar
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def read_path(dump_path: str, algorithm: str) -> np.ndarray:
    with open(dump_path, "rb") as fh:
        blob = fh.read()
    pos = 0

    def take(fmt, size):
        nonlocal pos
        value = struct.unpack_from(fmt, blob, pos)[0]
        pos += size
        return value

    for _ in range(take("<I", 4)):
        nlen = take("<I", 4)
        name = blob[pos:pos + nlen].decode()
        pos += nlen
        take("<B", 1)
        take("<d", 8)
        take("<d", 8)
        take("<Q", 8)
        npath = take("<I", 4)
        path = np.frombuffer(blob, dtype="<f8", count=npath * 2, offset=pos).reshape(-1, 2)
        pos += npath * 16
        nexp = take("<I", 4)
        pos += nexp * 4
        if name == algorithm:
            return path
    raise SystemExit(f"{algorithm} not found in {dump_path}")


class Driver(Node):
    def __init__(self, path: np.ndarray, speed: float, tolerance: float,
                 spawn, spawn_yaw: float):
        super().__init__("path_driver")
        # Odometry starts at zero where the car spawned, so it has to be
        # rotated and shifted into the map frame the path is expressed in.
        self.spawn = spawn
        self.spawn_yaw = spawn_yaw
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(Odometry, "/omni_drive_controller/odom", self.on_odom, 10)
        self.path = path
        self.speed = speed
        self.tolerance = tolerance
        self.pose = None
        self.index = 0

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s_ = math.cos(self.spawn_yaw), math.sin(self.spawn_yaw)
        self.pose = (self.spawn[0] + c * p.x - s_ * p.y,
                     self.spawn[1] + s_ * p.x + c * p.y,
                     yaw + self.spawn_yaw)

    def spin(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self) -> None:
        self.spin(2.0)
        if self.pose is None:
            raise SystemExit("no odometry on /omni_drive_controller/odom")

        rate_hz = 20.0
        while self.index < len(self.path):
            target = self.path[self.index]
            dx = target[0] - self.pose[0]
            dy = target[1] - self.pose[1]
            dist = math.hypot(dx, dy)

            if dist < self.tolerance:
                self.index += 1
                continue

            heading = math.atan2(dy, dx)
            error = math.atan2(math.sin(heading - self.pose[2]),
                               math.cos(heading - self.pose[2]))

            cmd = Twist()
            if abs(error) > 0.5:                    # turn in place first
                cmd.angular.z = max(-1.2, min(1.2, 1.8 * error))
            else:
                cmd.linear.x = self.speed * max(0.25, math.cos(error))
                cmd.angular.z = max(-1.2, min(1.2, 1.5 * error))
            self.pub.publish(cmd)
            self.spin(1.0 / rate_hz)

        self.pub.publish(Twist())                   # stop
        self.spin(0.5)
        self.get_logger().info(
            f"reached the goal: {self.pose[0]:.2f}, {self.pose[1]:.2f}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", default="/tmp/plan.bin")
    ap.add_argument("--algorithm", default="astar")
    ap.add_argument("--speed", type=float, default=0.35)
    ap.add_argument("--tolerance", type=float, default=0.25)
    ap.add_argument("--spawn-x", type=float, default=8.0727)
    ap.add_argument("--spawn-y", type=float, default=7.5312)
    ap.add_argument("--spawn-yaw", type=float, default=-1.5708)
    args = ap.parse_args()

    path = read_path(args.dump, args.algorithm)
    print(f"{args.algorithm}: {len(path)} waypoints, "
          f"({path[0][0]:.2f},{path[0][1]:.2f}) -> ({path[-1][0]:.2f},{path[-1][1]:.2f})")

    rclpy.init()
    node = Driver(path, args.speed, args.tolerance,
                  (args.spawn_x, args.spawn_y), args.spawn_yaw)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
