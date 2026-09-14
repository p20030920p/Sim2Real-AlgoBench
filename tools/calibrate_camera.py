#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Calibrate the world -> pixel mapping of the Gazebo top-down camera.

Assuming the camera sits at the centre of the arena looking straight down is not
good enough: the mount pose, the field of view and any image-plane rotation all
shift the mapping, and an overlay that merely looks plausible over corridors is
not evidence that it lines up with the car.

So the transform is measured. Bright markers are placed at known world
coordinates, one frame is captured, their pixel centres are found, and a least
squares affine fit is solved from the world positions to the pixels. The result
is written to JSON and used by the recording tools.

Usage:
    python3 tools/calibrate_camera.py --topic /top_view --out /tmp/cam_calib.json
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Spread across the arena but clear of the walls, so all of them stay visible.
MARKERS = [
    (-3.0, -5.0), (10.0, -5.0), (10.0, 7.0), (-3.0, 7.0),
    (3.5, 1.0), (3.5, 7.5), (-1.0, 1.0), (7.5, 1.0),
]

MARKER_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><box><size>0.36 0.36 0.06</size></box></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <emissive>1 0 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def place_markers() -> None:
    for i, (x, y) in enumerate(MARKERS):
        path = f"/tmp/marker_{i}.sdf"
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(MARKER_SDF.format(name=f"calib_marker_{i}"))
        subprocess.run(
            ["ros2", "run", "ros_gz_sim", "create", "-file", path,
             "-name", f"calib_marker_{i}", "-x", str(x), "-y", str(y), "-z", "0.1"],
            capture_output=True, timeout=40)


def remove_markers() -> None:
    world = os.environ.get("CALIB_WORLD", "competition_world")
    for i in range(len(MARKERS)):
        subprocess.run(
            ["gz", "service", "-s", f"/world/{world}/remove",
             "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
             "--timeout", "2000", "--req",
             f'name: "calib_marker_{i}", type: 2'],
            capture_output=True, timeout=25)


class Grab(Node):
    def __init__(self, topic: str):
        super().__init__("calib_grab")
        self.frame = None
        self.create_subscription(Image, topic, self.on_image, 1)

    def on_image(self, msg: Image) -> None:
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        self.frame = a[:, :, :3].copy()

    def wait(self, seconds: float = 20.0):
        end = time.time() + seconds
        while self.frame is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.frame


def detect(img: np.ndarray) -> list[tuple[float, float]]:
    """Centres of the red markers, largest first."""
    bgr = img[:, :, ::-1].astype(np.int16)
    b, g, r = bgr[:, :, 0], bgr[:, :, 1], bgr[:, :, 2]
    mask = ((r > 110) & (r - g > 55) & (r - b > 55)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, stats, centres = cv2.connectedComponentsWithStats(mask, 8)
    blobs = [(centres[i][0], centres[i][1], stats[i, cv2.CC_STAT_AREA])
             for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= 12]
    blobs.sort(key=lambda t: -t[2])
    return [(x, y) for x, y, _ in blobs]


def fit(world: np.ndarray, pixels: np.ndarray) -> np.ndarray:
    """Least squares affine world -> pixel, returned as a 2x3 matrix."""
    a = np.hstack([world, np.ones((len(world), 1))])
    coeff, *_ = np.linalg.lstsq(a, pixels, rcond=None)
    return coeff.T


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/top_view")
    ap.add_argument("--out", default="/tmp/cam_calib.json")
    ap.add_argument("--frame", default="/tmp/cam_calib.png")
    ap.add_argument("--keep-markers", action="store_true")
    args = ap.parse_args()

    print(f"placing {len(MARKERS)} markers ...")
    place_markers()
    time.sleep(3)

    rclpy.init()
    node = Grab(args.topic)
    frame = node.wait(25.0)
    rclpy.shutdown()
    if frame is None:
        raise SystemExit("no image on " + args.topic)
    cv2.imwrite(args.frame, frame)

    blobs = detect(frame)
    print(f"detected {len(blobs)} red blobs, need at least 3")
    if len(blobs) < 3:
        raise SystemExit("not enough markers visible; adjust MARKERS")

    # Match each blob to the nearest predicted marker, then refit. The initial
    # prediction only needs to be roughly right to disambiguate eight points.
    world = np.array(MARKERS, dtype=float)
    guess = np.array([[400 + (x - 3.65) * 40, 400 - (y - 1.0) * 40] for x, y in MARKERS])
    pairs_w, pairs_p = [], []
    for wx, wy in world:
        pred = np.array([400 + (wx - 3.65) * 40, 400 - (wy - 1.0) * 40])
        dists = [np.hypot(px - pred[0], py - pred[1]) for px, py in blobs]
        j = int(np.argmin(dists))
        if dists[j] < 260:
            pairs_w.append((wx, wy))
            pairs_p.append(blobs[j])
    if len(pairs_w) < 3:
        raise SystemExit("could not match markers to blobs")

    # Fit, drop the worst pair, refit: one spurious blob in the scene is enough
    # to skew an eight-point fit, and the good pairs identify themselves by
    # having small residuals.
    w = np.array(pairs_w)
    p = np.array(pairs_p)
    for _ in range(max(0, len(w) - 3)):
        m = fit(w, p)
        resid = p - (np.hstack([w, np.ones((len(w), 1))]) @ m.T)
        err = np.hypot(resid[:, 0], resid[:, 1])
        if err.max() < 25.0:
            break
        w = np.delete(w, int(np.argmax(err)), axis=0)
        p = np.delete(p, int(np.argmax(err)), axis=0)

    m = fit(w, p)
    resid = p - (np.hstack([w, np.ones((len(w), 1))]) @ m.T)
    err = np.hypot(resid[:, 0], resid[:, 1])
    pairs_w = [tuple(v) for v in w]
    pairs_p = [tuple(v) for v in p]
    print(f"kept {len(w)} markers, residual max {err.max():.2f} px, mean {err.mean():.2f} px")

    data = {
        "matrix": m.tolist(),
        "markers": [[float(a), float(b)] for a, b in pairs_w],
        "pixels": [[float(a), float(b)] for a, b in pairs_p],
        "residual_px": float(err.max()),
    }
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    print(f"wrote {args.out}")

    if not args.keep_markers:
        remove_markers()
        print("markers removed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
