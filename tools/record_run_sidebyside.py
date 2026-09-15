#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Record a run as Gazebo (left) beside RViz (right), with the path drawn in both.

The left half is the simulator's own top-down camera; the right half is the RViz
window captured off the screen. Both show the same moment of the same run, so
the clip answers "what did the planner decide, and what did the robot actually
do about it" in one view.

The path is drawn into the Gazebo world as flat markers rather than overlaid on
the image. Gazebo then renders it in the same projection as the arena, so the
line is correct without calibrating a world -> pixel mapping, which every
attempt here got wrong by a metre or more. The path comes from the same /plan
topic RViz draws, so the two halves cannot disagree.

Usage:
    python3 tools/record_run_sidebyside.py --algorithm astar --out-dir /tmp/run
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import subprocess
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

_HERE = os.path.dirname(os.path.abspath(__file__))


def _load_xwd_capture():
    """Import tools/xwd_capture.py by path; tools is not a package."""
    spec = importlib.util.spec_from_file_location(
        'xwd_capture', os.path.join(_HERE, 'xwd_capture.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


XWD = _load_xwd_capture()

# Window title to capture for the right-hand half. xdotool is not installed on
# this machine, so the window is found the same way tools/xwd_capture.py does.
RVIZ_WINDOW = os.environ.get('RACE_RVIZ_WINDOW', 'RViz')


class RunRecorder(Node):
    def __init__(self, out_dir, fps, spawn, spawn_yaw):
        super().__init__('sidebyside_recorder')
        self.out_dir = out_dir
        self.fps = fps
        self.interval = 1.0 / fps
        self.spawn = spawn
        self.spawn_yaw = spawn_yaw

        self.latest_image = None
        self.pose = None
        self.state = None
        self.outcome = None
        self.plan = None
        self.plan_drawn = False
        self.rviz_id = None
        self.rviz_hits = 0
        self.rviz_misses = 0
        self.frames = []
        self.count = 0

        self.create_subscription(Image, '/top_view', self.on_image, 1)
        self.create_subscription(
            Odometry, '/omni_drive_controller/odom', self.on_odom, 20)
        self.create_subscription(Path, '/plan', self.on_plan, 2)
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(String, '/race/state', self.on_state, latched)
        self.create_subscription(Bool, '/race/complete', self.on_complete, latched)

    def on_image(self, msg):
        self.latest_image = np.frombuffer(
            msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3].copy()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(self.spawn_yaw), math.sin(self.spawn_yaw)
        self.pose = (self.spawn[0] + c * p.x - s * p.y,
                     self.spawn[1] + s * p.x + c * p.y,
                     yaw + self.spawn_yaw)

    def on_plan(self, msg):
        path = np.array([[p.pose.position.x, p.pose.position.y]
                         for p in msg.poses], dtype=float)
        if self.plan is None or len(path) > len(self.plan):
            self.plan = path

    def on_state(self, msg):
        self.state = msg.data

    def on_complete(self, msg):
        if msg.data:
            self.outcome = 'COMPLETE'

    def draw_plan_into_world(self, name='pathline'):
        """Spawn the planned path as flat markers so Gazebo draws it exactly."""
        if self.plan is None or len(self.plan) < 2 or self.plan_drawn:
            return
        path = os.path.join(self.out_dir, 'plan.npy')
        np.save(path, self.plan)
        try:
            result = subprocess.run(
                ['python3', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         'world_line.py'),
                 '--name', name, '--from-npy', path,
                 '--spacing', '0.25', '--bgr', '40,40,235', '--size', '0.12'],
                capture_output=True, text=True, timeout=300)
            self.plan_drawn = True
            print(result.stdout.strip() or result.stderr.strip()[-200:], flush=True)
        except Exception as exc:                       # noqa: BLE001
            print(f'could not draw the path: {exc}', flush=True)

    def draw_trail_into_world(self, name='trailline'):
        """Spawn the driven trail at the end, from the recorded odometry."""
        poses = [f['pose'] for f in self.frames if f['pose']]
        if len(poses) < 2:
            return
        path = os.path.join(self.out_dir, 'trail.npy')
        np.save(path, np.asarray(poses, dtype=float)[:, :2])
        try:
            subprocess.run(
                ['python3', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         'world_line.py'),
                 '--name', name, '--from-npy', path, '--stride', '3',
                 '--spacing', '0.25', '--bgr', '40,200,40', '--size', '0.10'],
                capture_output=True, text=True, timeout=300)
        except Exception:                              # noqa: BLE001
            pass

    def label(self, image):
        cv2.rectangle(image, (0, 0), (image.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(image, self.state or '', (8, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return image

    def run(self, seconds):
        deadline = time.time() + seconds
        next_shot = time.time()
        writer = None
        finished_at = None
        grace = 20.0
        xwd = os.path.join(self.out_dir, '.rviz.xwd')
        try:
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.03)

                if self.outcome == 'COMPLETE' and finished_at is None:
                    finished_at = time.time()
                if finished_at is not None and time.time() - finished_at > grace:
                    break
                if self.latest_image is None or time.time() < next_shot:
                    continue

                # The top camera's image axes are not the world axes the map,
                # the path and RViz use, so the two halves would show the arena
                # turned against each other. Measured by putting a 1.2 m marker
                # at a known world point, one at a time, and differencing the
                # frame to find it (the camera is over the arena centre, so its
                # own position is image (240, 240)):
                #
                #   world +X 3 m  ->  offset ( -0.5, -82.1)   image up
                #   world -X 3 m  ->  offset ( -9.0, +81.5)   image down
                #   world +Y 3 m  ->  offset (-82.5,  +8.0)   image left
                #
                # 82 px for 3 m is 27.4 px/m, against 27.2 predicted from the
                # camera height and field of view, so the measurement is sound.
                # A quarter turn clockwise puts +X right and +Y up, which is the
                # orientation the map and RViz are drawn in.
                left = cv2.rotate(self.latest_image.copy(), cv2.ROTATE_90_CLOCKWISE)
                right = None
                try:
                    if self.rviz_id is None:
                        self.rviz_id = XWD.find_window(RVIZ_WINDOW)
                    if self.rviz_id:
                        XWD.grab(self.rviz_id, xwd)
                        right = cv2.cvtColor(
                            np.asarray(XWD.read_xwd(xwd).convert('RGB')),
                            cv2.COLOR_RGB2BGR)
                except Exception:                      # noqa: BLE001
                    right = None
                if right is None:
                    # Counted and reported: a failed grab silently produces a
                    # half-black clip, which is worse than no clip.
                    self.rviz_misses += 1
                else:
                    self.rviz_hits += 1

                left = self.label(left)
                height = left.shape[0]
                if right is None:
                    right = np.full((height, height, 3), 32, np.uint8)
                else:
                    scale = height / right.shape[0]
                    right = cv2.resize(
                        right, (max(1, int(right.shape[1] * scale)), height),
                        interpolation=cv2.INTER_AREA)
                    right = self.label(right)
                frame = np.hstack([left, right])

                if writer is None:
                    h, w = frame.shape[:2]
                    writer = cv2.VideoWriter(
                        os.path.join(self.out_dir, 'video.mp4'),
                        cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (w, h))
                writer.write(frame)
                self.frames.append({
                    'frame': self.count,
                    'pose': list(self.pose) if self.pose else None,
                    'state': self.state,
                })
                self.count += 1
                next_shot += self.interval
        finally:
            if writer is not None:
                writer.release()
            if os.path.exists(xwd):
                os.remove(xwd)

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--algorithm', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--fps', type=float, default=4.0)
    ap.add_argument('--seconds', type=float, default=1500.0)
    ap.add_argument('--spawn-x', type=float, default=8.0727)
    ap.add_argument('--spawn-y', type=float, default=7.5312)
    ap.add_argument('--spawn-yaw', type=float, default=-1.5708)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    node = RunRecorder(args.out_dir, args.fps,
                       (args.spawn_x, args.spawn_y), args.spawn_yaw)
    try:
        node.run(args.seconds)
    finally:
        with open(os.path.join(args.out_dir, 'run.json'), 'w',
                  encoding='utf-8') as handle:
            json.dump({'algorithm': args.algorithm,
                       'plan': node.plan.tolist() if node.plan is not None else None,
                       'frames': node.frames,
                       'outcome': node.outcome}, handle)
        meta = {'algorithm': args.algorithm, 'frames': node.count,
                'outcome': node.outcome, 'last_state': node.state,
                'plan_poses': 0 if node.plan is None else len(node.plan),
                'rviz_frames_captured': node.rviz_hits,
                'rviz_frames_missing': node.rviz_misses}
        with open(os.path.join(args.out_dir, 'meta.json'), 'w',
                  encoding='utf-8') as handle:
            json.dump(meta, handle, indent=2)
        print(json.dumps(meta), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
