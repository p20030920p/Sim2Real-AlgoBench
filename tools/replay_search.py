#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Replay one planner's search into RViz, cell by cell.

`algo_plan_dump` records the order in which the algorithm expanded cells. The
planner returns in a few milliseconds, so replaying that order on a human
timescale is the only way to see what the search actually did: which way it
spread, how much of the map it touched, and where it gave up.

The cells and the path are the algorithm's own output, taken from the same
algo_core the Nav2 plugin loads. Only the playback speed is invented.

Usage:
    python3 tools/replay_search.py --dump /tmp/plan.bin --algorithm dijkstra
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Matches the costmap the search ran on; used to convert a cell index to metres.
DEFAULT_ORIGIN = (-3.700, -6.342)
DEFAULT_RESOLUTION = 0.050
DEFAULT_SIZE = 294

# Early expansions pale, late expansions saturated.
EXPAND_START = (1.00, 0.89, 0.66)
EXPAND_END = (0.96, 0.46, 0.17)
PATH_COLOUR = (0.84, 0.16, 0.16)


def read_dump(path: str) -> dict:
    with open(path, "rb") as fh:
        blob = fh.read()
    pos = 0

    def take(fmt, size):
        nonlocal pos
        value = struct.unpack_from(fmt, blob, pos)[0]
        pos += size
        return value

    out = {}
    for _ in range(take("<I", 4)):
        nlen = take("<I", 4)
        name = blob[pos:pos + nlen].decode()
        pos += nlen
        success = take("<B", 1) == 1
        take("<d", 8)                      # cost
        ms = take("<d", 8)
        take("<Q", 8)                      # iterations
        npath = take("<I", 4)
        path = np.frombuffer(blob, dtype="<f8", count=npath * 2, offset=pos).reshape(-1, 2)
        pos += npath * 16
        nexp = take("<I", 4)
        expanded = np.frombuffer(blob, dtype="<i4", count=nexp, offset=pos)
        pos += nexp * 4
        out[name] = {"success": success, "ms": ms, "path": path,
                     "expanded": expanded.copy()}
    return out


class Replay(Node):
    def __init__(self, topic: str):
        super().__init__("search_replay")
        self.pub = self.create_publisher(MarkerArray, topic, 1)

    def publish(self, markers: list[Marker]) -> None:
        array = MarkerArray()
        array.markers = markers
        self.pub.publish(array)
        rclpy.spin_once(self, timeout_sec=0.0)

    @staticmethod
    def clear_marker() -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.action = Marker.DELETEALL
        return marker

    def expanded_marker(self, cells: np.ndarray, colours: np.ndarray) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "expanded"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = DEFAULT_RESOLUTION
        marker.scale.z = 0.01
        marker.pose.orientation.w = 1.0

        origin_x, origin_y = DEFAULT_ORIGIN
        res = DEFAULT_RESOLUTION
        points, colors = [], []
        for i in range(len(cells)):
            idx = int(cells[i])
            gx, gy = idx % DEFAULT_SIZE, idx // DEFAULT_SIZE
            points.append(Point(x=origin_x + (gx + 0.5) * res,
                                y=origin_y + (gy + 0.5) * res,
                                z=0.02))
            c = colours[i]
            colors.append(ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0))
        marker.points = points
        marker.colors = colors
        return marker

    def path_marker(self, path: np.ndarray) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.07
        marker.color = ColorRGBA(r=PATH_COLOUR[0], g=PATH_COLOUR[1],
                                 b=PATH_COLOUR[2], a=1.0)
        marker.pose.orientation.w = 1.0
        marker.points = [Point(x=float(x), y=float(y), z=0.06) for x, y in path]
        return marker

    def label_marker(self, text: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "label"
        marker.id = 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 1.6
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        # Above the arena, which tops out near y = 8.4, so the text never
        # covers the search it is describing.
        marker.pose.position.x = 3.65
        marker.pose.position.y = 12.5
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.text = text
        return marker


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", required=True)
    ap.add_argument("--algorithm", required=True)
    ap.add_argument("--topic", default="/algo_paths")
    ap.add_argument("--reveal", type=float, default=5.0,
                    help="seconds spent revealing the expanded cells")
    ap.add_argument("--steps", type=int, default=60, help="reveal updates")
    ap.add_argument("--hold", type=float, default=3.0,
                    help="seconds to hold the finished picture")
    args = ap.parse_args()

    dump = read_dump(args.dump)
    if args.algorithm not in dump:
        print(f"unknown algorithm {args.algorithm!r}; have {sorted(dump)}", file=sys.stderr)
        return 2
    rec = dump[args.algorithm]
    if not rec["success"]:
        print(f"{args.algorithm} did not find a path; nothing to replay", file=sys.stderr)
        return 1

    cells = rec["expanded"]
    n = len(cells)
    t = np.linspace(0.0, 1.0, n, dtype=np.float32)[:, None]
    colours = (np.array(EXPAND_START, dtype=np.float32) * (1 - t) +
               np.array(EXPAND_END, dtype=np.float32) * t)

    # Keep this short: a long line runs off both edges of the viewport and
    # clips the algorithm name, which is the one thing the label has to say.
    label = args.algorithm

    rclpy.init()
    node = Replay(args.topic)
    try:
        time.sleep(2.0)                                  # let RViz settle
        node.publish([node.clear_marker()])
        time.sleep(0.7)

        step = max(1, n // args.steps)
        interval = args.reveal / max(1, len(range(step, n + step, step)))
        shown = 0
        for end in range(step, n + step, step):
            end = min(end, n)
            node.publish([
                node.expanded_marker(cells[:end], colours[:end]),
                node.label_marker(label),
            ])
            shown = end
            time.sleep(interval)

        # The path appears once the search has finished, as it does in the planner.
        node.publish([
            node.expanded_marker(cells[:shown], colours[:shown]),
            node.path_marker(rec["path"]),
            node.label_marker(label),
        ])
        deadline = time.time() + args.hold
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        print(f"replayed {args.algorithm}: {shown} cells, {len(rec['path'])} poses")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
