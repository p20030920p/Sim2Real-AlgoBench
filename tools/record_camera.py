#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Record a ROS image topic to numbered PNGs.

Used to capture the Gazebo top-down camera while the car drives. The topic is
bridged from Gazebo with ros_gz_bridge, so this is the simulator's own render
rather than a reconstruction.

Usage:
    python3 tools/record_camera.py --topic /top_view --out-dir /tmp/frames \
        --fps 5 --seconds 60
"""

from __future__ import annotations

import argparse
import os
import time

import cv2
import numpy as np
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, topic: str, out_dir: str, fps: float, seconds: float,
                 spawn=(8.0727, 7.5312), spawn_yaw=-1.5708):
        super().__init__("camera_recorder")
        self.topic = topic
        self.out_dir = out_dir
        self.interval = 1.0 / fps
        self.seconds = seconds
        self.latest = None
        self.odom = None
        self.log = []
        self.count = 0
        self.spawn = spawn
        self.spawn_yaw = spawn_yaw
        self.create_subscription(Image, topic, self.on_image, 1)
        self.create_subscription(Odometry, "/omni_drive_controller/odom", self.on_odom, 10)

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s_ = math.cos(self.spawn_yaw), math.sin(self.spawn_yaw)
        self.odom = (self.spawn[0] + c * p.x - s_ * p.y,
                     self.spawn[1] + s_ * p.x + c * p.y,
                     yaw + self.spawn_yaw)

    def on_image(self, msg: Image) -> None:
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        self.latest = frame[:, :, :3].copy()

    def run(self) -> None:
        deadline = time.time() + self.seconds
        next_shot = time.time()
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is None or time.time() < next_shot:
                continue
            path = os.path.join(self.out_dir, f"f{self.count:05d}.png")
            cv2.imwrite(path, self.latest)
            # Record where the car actually was for this frame, so the overlay
            # can be checked against it rather than assumed correct.
            self.log.append({"frame": self.count, "odom": self.odom})
            self.count += 1
            next_shot += self.interval
        with open(os.path.join(self.out_dir, "odom.json"), "w", encoding="utf-8") as fh:
            json.dump(self.log, fh)
        self.get_logger().info(f"wrote {self.count} frames to {self.out_dir}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/top_view")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    node = Recorder(args.topic, args.out_dir, args.fps, args.seconds)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
