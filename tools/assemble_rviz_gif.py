#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Turn captured RViz frames into the README GIF.

The capture is a whole window, so it carries RViz's Displays panel and the
window title bar. Rather than hard-coding a crop that breaks whenever the window
moves, this finds the 3D viewport by its background colour and then crops to the
content inside it, so the result is the map and the paths and nothing else.

Usage:
    python3 tools/assemble_rviz_gif.py /tmp/rvframes docs/media/rviz_planners.gif
"""

from __future__ import annotations

import argparse
import glob
import os

import numpy as np
from PIL import Image

# RViz's viewport background, from Global Options in planning_demo.rviz.
BACKGROUND = np.array([32, 34, 38], dtype=np.int16)
TOLERANCE = 26


def viewport_box(frames: list[np.ndarray]) -> tuple[int, int, int, int]:
    """Bounding box of the largest region whose colour is the viewport background."""
    sample = frames[len(frames) // 2]
    is_bg = (np.abs(sample.astype(np.int16) - BACKGROUND).max(axis=2) <= TOLERANCE)

    # Rows and columns that are predominantly viewport background.
    col_frac = is_bg.mean(axis=0)
    row_frac = is_bg.mean(axis=1)
    cols = np.where(col_frac > 0.35)[0]
    rows = np.where(row_frac > 0.35)[0]
    if len(cols) == 0 or len(rows) == 0:
        h, w = sample.shape[:2]
        return 0, 0, w, h

    # The panel sits on the left, so the viewport is the right-hand run.
    right_start = cols[0]
    runs, start = [], cols[0]
    for a, b in zip(cols, cols[1:]):
        if b - a > 1:
            runs.append((start, a))
            start = b
    runs.append((start, cols[-1]))
    left, right = max(runs, key=lambda r: r[1] - r[0])
    top, bottom = rows[0], rows[-1]
    return int(left), int(top), int(right), int(bottom)


def content_box(frames: list[np.ndarray], view: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    """Tight box around everything drawn on the viewport, taken from the last frame.

    The last frame has every path on screen, so a crop derived from it is stable
    for the whole sequence and the map never shifts between frames.
    """
    left, top, right, bottom = view
    last = frames[-1][top:bottom + 1, left:right + 1]
    differs = (np.abs(last.astype(np.int16) - BACKGROUND).max(axis=2) > TOLERANCE)

    # Ignore the grid: it is only faintly different from the background.
    strength = np.abs(last.astype(np.int16) - BACKGROUND).max(axis=2)
    mask = differs & (strength > 45)

    rows = np.where(mask.any(axis=1))[0]
    cols = np.where(mask.any(axis=0))[0]
    if len(rows) == 0 or len(cols) == 0:
        return 0, 0, last.shape[1] - 1, last.shape[0] - 1

    margin = 12
    y0 = max(0, rows[0] - margin)
    y1 = min(last.shape[0] - 1, rows[-1] + margin)
    x0 = max(0, cols[0] - margin)
    x1 = min(last.shape[1] - 1, cols[-1] + margin)

    # Square it up so the GIF does not jump in aspect.
    h, w = y1 - y0, x1 - x0
    # Square up for a stable aspect, but never beyond what the viewport holds.
    side = min(max(h, w), last.shape[0], last.shape[1])
    cy, cx = (y0 + y1) // 2, (x0 + x1) // 2
    y0 = max(0, min(last.shape[0] - side, cy - side // 2))
    x0 = max(0, min(last.shape[1] - side, cx - side // 2))
    return x0, y0, x0 + side, y0 + side


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("frames", help="directory of captured PNGs")
    ap.add_argument("out", help="output GIF")
    ap.add_argument("--width", type=int, default=760)
    ap.add_argument("--fps", type=float, default=8.0)
    ap.add_argument("--colors", type=int, default=96)
    ap.add_argument("--every", type=int, default=1, help="keep every Nth frame")
    ap.add_argument("--viewport", default=None, metavar="L,T,R,B",
                    help="3D viewport box in captured pixels. Detection is only reliable "
                         "when the panel does not contain dark UI, so passing this explicitly "
                         "is the dependable path.")
    ap.add_argument("--mp4", default=None)
    args = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(args.frames, "*.png")))
    if not paths:
        raise SystemExit(f"no frames in {args.frames}")
    if args.every > 1:
        paths = paths[::args.every]
    print(f"frames : {len(paths)}")

    raw = [np.array(Image.open(p).convert("RGB")) for p in paths]
    if args.viewport:
        view = tuple(int(v) for v in args.viewport.split(","))
    else:
        view = viewport_box(raw)
    box = content_box(raw, view)
    print(f"viewport: {view}   content: {box}   size: {box[2] - box[0]}x{box[3] - box[1]}")

    vx0, vy0, _, _ = view
    cropped = []
    for arr in raw:
        sub = arr[vy0:view[3] + 1, vx0:view[2] + 1]
        sub = sub[box[1]:box[3], box[0]:box[2]]
        img = Image.fromarray(sub)
        if img.width != args.width:
            img = img.resize((args.width, int(round(img.height * args.width / img.width))),
                             Image.LANCZOS)
        cropped.append(img)

    strip = Image.new("RGB", (cropped[0].width, cropped[0].height * min(6, len(cropped))))
    step = max(1, len(cropped) // 6)
    for i, im in enumerate(cropped[::step][:6]):
        strip.paste(im, (0, i * im.height))
    palette = strip.quantize(colors=args.colors, method=Image.MEDIANCUT)
    quant = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in cropped]

    quant[0].save(args.out, save_all=True, append_images=quant[1:], optimize=True,
                  loop=0, duration=int(round(1000 / args.fps)), disposal=2)
    mb = os.path.getsize(args.out) / 1048576
    print(f"wrote  : {args.out} ({mb:.2f} MB)")

    if args.mp4:
        import cv2
        h, w = cropped[0].height, cropped[0].width
        writer = cv2.VideoWriter(args.mp4, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))
        for im in cropped:
            writer.write(cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR))
        writer.release()
        print(f"wrote  : {args.mp4}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
