#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Record one complete competition run from above, with the planner's path.

Used to compare global planners on *the same scenario*: the run is the full
stack (AMCL, Nav2, vision finish, state machine), the only difference between
two recordings being which algorithm serves the global planner.

Outputs, into --out-dir:
    f%05d.png      one frame per sample, with the overlay already drawn
    run.json       per-frame state/pose plus the planner path and the outcome
    meta.json      algorithm, outcome, elapsed time, frame count

The overlay needs the world -> pixel mapping of the top camera. Rather than
assuming the camera is axis-aligned (it is not: the image axes sit about -7.4
degrees off the world axes), the mapping is passed in as a similarity solved
from markers of known world position.

Usage:
    python3 tools/record_autonomy_run.py --algorithm astar --out-dir /tmp/run_astar
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

# Solved from ground-truth markers (see tools/calibrate_camera.py). A point at
# world (wx, wy) lands at pixel (px, py) with
#     px = cx + ax*wx - ay*wy
#     py = cy + ay*wx + ax*wy
DEFAULT_SIMILARITY = {
    'cx': 278.50, 'cy': 397.88, 'ax': 37.4040, 'ay': 4.8274,
    'note': 'fitted to spawn/goal markers of known world pose',
}


class RunRecorder(Node):
    def __init__(self, image_topic, out_dir, fps, similarity, spawn, spawn_yaw,
                 out_width=520):
        super().__init__('autonomy_run_recorder')
        self.image_topic = image_topic
        self.out_dir = out_dir
        self.interval = 1.0 / fps
        self.fps = fps
        self.out_width = out_width
        self.sim = similarity
        self.spawn = spawn
        self.spawn_yaw = spawn_yaw

        self.latest_image = None
        self.pose = None
        self.state = None
        self.plan = None
        self.outcome = None
        self.frames = []
        self.count = 0
        self.canvas = None
        self.trail_pt = None

        self.create_subscription(Image, image_topic, self.on_image, 1)
        self.create_subscription(
            Odometry, '/omni_drive_controller/odom', self.on_odom, 20)
        # Nav2 exposes the global plan as /plan from planner_server; the
        # controller republishes the path it is actually following on
        # /received_global_plan. Subscribe to both and prefer whichever is
        # actually being published on this Nav2 version.
        for topic in ('/plan', '/received_global_plan', '/global_plan'):
            self.create_subscription(Path, topic, self.on_plan, 2)

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(String, '/race/state', self.on_state, latched)
        self.create_subscription(Bool, '/race/complete', self.on_complete, latched)

    # -- conversions -------------------------------------------------------
    def world_to_pixel(self, wx, wy):
        s = self.sim
        return (s['cx'] + s['ax'] * wx - s['ay'] * wy,
                s['cy'] + s['ay'] * wx + s['ax'] * wy)

    # -- callbacks ---------------------------------------------------------
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
        # Keep the longest path seen rather than the most recent one. Nav2
        # replans continuously and the last message before the run ends is the
        # short final approach onto the pad, which is not what the clip should
        # draw as "the path the planner produced".
        path = np.array([[p.pose.position.x, p.pose.position.y]
                         for p in msg.poses], dtype=float)
        if self.plan is None or len(path) > len(self.plan):
            self.plan = path

    def on_state(self, msg):
        self.state = msg.data

    def on_complete(self, msg):
        if msg.data:
            self.outcome = 'COMPLETE'

    # -- drawing -----------------------------------------------------------
    def draw(self, image):
        # Redraw only the newest segment of the driven trail. Rebuilding the
        # whole trail from the frame log on every sample is quadratic, and over
        # a race lasting thousands of samples that costs more CPU than the
        # simulator gets. The canvas keeps the trail drawn so far.
        if self.canvas is None or self.canvas.shape != image.shape:
            self.canvas = image.copy()
            self.trail_pt = None
        canvas = self.canvas
        canvas[:] = image

        if self.plan is not None and len(self.plan) > 1:
            pts = np.array([self.world_to_pixel(x, y)
                            for x, y in self.plan], np.int32)
            cv2.polylines(canvas, [pts], False, (40, 40, 235), 2, cv2.LINE_AA)
            for p in pts:
                cv2.circle(canvas, tuple(p), 3, (40, 40, 235), -1, cv2.LINE_AA)

        if self.pose:
            point = self.world_to_pixel(self.pose[0], self.pose[1])
            if self.trail_pt is not None:
                cv2.line(canvas, self.trail_pt,
                         (int(point[0]), int(point[1])), (0, 200, 0), 2, cv2.LINE_AA)
            self.trail_pt = (int(point[0]), int(point[1]))
            cv2.circle(canvas, self.trail_pt, 6, (0, 0, 255), -1, cv2.LINE_AA)

        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(canvas, self.state or '', (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
        return canvas

    def run(self, seconds):
        """Sample the camera into a video file, plus a pose/state log.

        Frames go to MP4 rather than numbered PNGs: a race lasts hundreds of
        seconds of wall clock here, and PNG encoding at that length costs more
        CPU than the simulator gets, which drags the run out further still. The
        video is the deliverable; the per-frame log is what the analysis reads.
        """
        deadline = time.time() + seconds
        next_shot = time.time()
        writer = None
        # Stop a little after the task itself finishes, rather than always
        # running out the window. An algorithm that never finds a path would
        # otherwise burn the whole budget before anyone can look at it.
        finished_at = None
        grace = 20.0
        try:
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.03)
                if self.outcome == 'COMPLETE' and finished_at is None:
                    finished_at = time.time()
                if finished_at is not None and time.time() - finished_at > grace:
                    break
                if self.latest_image is None or time.time() < next_shot:
                    continue
                sim_time = None
                if self.get_clock().now().nanoseconds:
                    sim_time = self.get_clock().now().nanoseconds / 1e9
                self.frames.append({
                    'frame': self.count,
                    't': sim_time,
                    'pose': list(self.pose) if self.pose else None,
                    'state': self.state,
                })
                image = self.draw(self.latest_image.copy())
                if self.out_width and image.shape[1] != self.out_width:
                    scale = self.out_width / float(image.shape[1])
                    image = cv2.resize(
                        image, (self.out_width, int(round(image.shape[0] * scale))),
                        interpolation=cv2.INTER_AREA)
                if writer is None:
                    height, width = image.shape[:2]
                    writer = cv2.VideoWriter(
                        os.path.join(self.out_dir, 'video.mp4'),
                        cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (width, height))
                    if not writer.isOpened():
                        raise RuntimeError('could not open the output video')
                writer.write(image)
                self.count += 1
                next_shot += self.interval
        finally:
            if writer is not None:
                writer.release()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--algorithm', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--image-topic', default='/top_view')
    ap.add_argument('--fps', type=float, default=3.0)
    ap.add_argument('--width', type=int, default=520,
                    help='output width; frames are downsampled before writing')
    ap.add_argument('--seconds', type=float, default=210.0)
    ap.add_argument('--spawn-x', type=float, default=8.0727)
    ap.add_argument('--spawn-y', type=float, default=7.5312)
    ap.add_argument('--spawn-yaw', type=float, default=-1.5708)
    ap.add_argument('--similarity', default=None,
                    help='JSON file with cx, cy, ax, ay')
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    sim = dict(DEFAULT_SIMILARITY)
    if args.similarity:
        with open(args.similarity, 'r', encoding='utf-8') as handle:
            sim.update(json.load(handle))

    rclpy.init()
    node = RunRecorder(args.image_topic, args.out_dir, args.fps, sim,
                       (args.spawn_x, args.spawn_y), args.spawn_yaw, args.width)
    try:
        node.run(args.seconds)
    finally:
        with open(os.path.join(args.out_dir, 'run.json'), 'w',
                  encoding='utf-8') as handle:
            json.dump({
                'algorithm': args.algorithm,
                'plan': node.plan.tolist() if node.plan is not None else None,
                'frames': node.frames,
                'outcome': node.outcome,
                'similarity': sim,
            }, handle)
        meta = {
            'algorithm': args.algorithm,
            'frames': node.count,
            'outcome': node.outcome,
            'last_state': node.state,
            'plan_poses': 0 if node.plan is None else len(node.plan),
            'similarity': sim,
        }
        with open(os.path.join(args.out_dir, 'meta.json'), 'w',
                  encoding='utf-8') as handle:
            json.dump(meta, handle, indent=2)
        print(json.dumps(meta))
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
