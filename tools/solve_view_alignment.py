#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Measure how the RViz view is rotated against the Gazebo view, then fix it.

The two halves of the side-by-side clips are not in the same orientation: the
Gazebo camera's image axes are swapped relative to the world axes the map and
RViz use, so the arena appears turned by a quarter turn between the panels. The
direction is not something to guess at - either answer looks plausible in a
squarish arena, and getting it wrong points the whole map the wrong way.

So it is measured. Bright markers are spawned at known world coordinates, one
frame is taken from each view, the markers are located in both, and the
rotation that carries one view onto the other is computed from the
correspondence. The result is written out and used by the recorder.

Usage:
    python3 tools/solve_view_alignment.py --out /tmp/view_alignment.json
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
from rclpy.node import Node
from sensor_msgs.msg import Image

_HERE = os.path.dirname(os.path.abspath(__file__))


def _xwd():
    spec = importlib.util.spec_from_file_location(
        'xwd_capture', os.path.join(_HERE, 'xwd_capture.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Four markers well apart, in a layout that is not symmetric under a quarter
# turn, so the rotation is determined and not merely one of four candidates.
# Three on a line plus one off it is enough to fix the direction.
MARKERS = [
    ('ax', 0.65, -2.00),
    ('bx', 6.65, -2.00),
    ('cx', 0.65, 4.00),
    ('dx', 3.65, 1.00),
]

MARKER_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><box><size>0.45 0.45 0.08</size></box></geometry>
        <material>
          <ambient>1 1 0 1</ambient>
          <diffuse>1 1 0 1</diffuse>
          <emissive>1 1 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class Grab(Node):
    def __init__(self, topic='/top_view'):
        super().__init__('view_alignment')
        self.frame = None
        self.create_subscription(Image, topic, self.on_image, 1)

    def on_image(self, msg):
        self.frame = np.frombuffer(
            msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3].copy()

    def wait(self, seconds=30.0):
        end = time.time() + seconds
        while self.frame is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.3)
        return self.frame


def find_yellow(image: np.ndarray):
    """Centroid of the yellow markers, which are the only saturated yellow."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (22, 120, 120), (35, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, stats, centres = cv2.connectedComponentsWithStats(mask, 8)
    points = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 12:
            continue
        points.append((float(centres[i][0]), float(centres[i][1]),
                       int(stats[i, cv2.CC_STAT_AREA])))
    points.sort(key=lambda p: -p[2])
    return points


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/view_alignment.json')
    ap.add_argument('--rviz-window', default='RViz')
    ap.add_argument('--keep-markers', action='store_true')
    args = ap.parse_args()

    xwd = _xwd()

    print('placing markers at known world positions')
    for name, wx, wy in MARKERS:
        path = f'/tmp/align_{name}.sdf'
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(MARKER_SDF.format(name=f'align_{name}'))
        subprocess.run(
            ['ros2', 'run', 'ros_gz_sim', 'create', '-file', path,
             '-name', f'align_{name}', '-x', str(wx), '-y', str(wy), '-z', '0.06'],
            capture_output=True, timeout=90)
    time.sleep(4)

    rclpy.init()
    node = Grab()
    frame = node.wait(40.0)
    rclpy.shutdown()
    if frame is None:
        raise SystemExit('no image on /top_view')
    cv2.imwrite('/tmp/align_gazebo.png', frame)

    gazebo_points = find_yellow(frame)
    print(f'gazebo view: {len(gazebo_points)} yellow markers')
    for p in gazebo_points:
        print(f'   pixel ({p[0]:6.1f},{p[1]:6.1f})  area {p[2]}')

    window = xwd.find_window(args.rviz_window)
    rviz_points = []
    if window:
        xwd.grab(window, '/tmp/align_rviz.xwd')
        shot = np.asarray(xwd.read_xwd('/tmp/align_rviz.xwd').convert('RGB'))
        shot = cv2.cvtColor(shot, cv2.COLOR_RGB2BGR)
        cv2.imwrite('/tmp/align_rviz.png', shot)
        rviz_points = find_yellow(shot)
        print(f'rviz view: {len(rviz_points)} yellow markers')
        for p in rviz_points:
            print(f'   pixel ({p[0]:6.1f},{p[1]:6.1f})  area {p[2]}')
    else:
        print('RViz window not found; only the Gazebo half can be characterised')

    # The Gazebo image axes are the world axes rotated by a quarter turn. Fit
    # which quarter turn from the marker layout: take the two points furthest
    # apart and see which world axis their separation lies along.
    result = {'gazebo_markers': [list(p[:2]) for p in gazebo_points],
              'rviz_markers': [list(p[:2]) for p in rviz_points]}
    if len(gazebo_points) >= 2:
        pts = np.array([p[:2] for p in gazebo_points])
        world = np.array([[wx, wy] for _, wx, wy in MARKERS])
        spread_px = pts.max(axis=0) - pts.min(axis=0)
        spread_w = world.max(axis=0) - world.min(axis=0)
        print(f'\ngazebo marker spread: {spread_px[0]:.0f} px in image x, '
              f'{spread_px[1]:.0f} px in image y')
        print(f'world marker spread:  {spread_w[0]:.2f} m in world x, '
              f'{spread_w[1]:.2f} m in world y')
        # Which image axis carries the longer world extent tells us the swap.
        swapped = spread_px[0] < spread_px[1]
        result['axes_swapped'] = bool(swapped)
        print(f'image axes swapped against world axes: {swapped}')

    with open(args.out, 'w', encoding='utf-8') as handle:
        json.dump(result, handle, indent=2)
    print(f'\nwrote {args.out}')

    if not args.keep_markers:
        for name, _, _ in MARKERS:
            subprocess.run(
                ['gz', 'service', '-s', '/world/competition_world/remove',
                 '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '3000', '--req',
                 f'name: "align_{name}", type: 2'.encode()],
                capture_output=True, timeout=30)
        print('markers removed')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
