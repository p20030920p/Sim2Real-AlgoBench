#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Render the README planning demo from real planner output.

Reads the dump produced by `algo_plan_dump` and draws one panel per algorithm:
the cells the search expanded, in the order it expanded them, then the path it
returned, with the robot running along it. Panels share a scale, so the
difference between a Dijkstra sweep and a D* Lite repair is visible rather than
asserted.

The dump comes from the same algo_core library the Nav2 plugin loads. Nothing
here re-implements a planner.

Usage:
    python3 tools/render_planning_demo.py dump.bin map.pgm out.gif
"""

from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# --- palette -----------------------------------------------------------------
COL_FREE = (246, 246, 244)
COL_OCCUPIED = (38, 38, 42)
COL_UNKNOWN = (176, 176, 172)
COL_PATH = (214, 40, 40)
COL_ROBOT = (255, 255, 255)
COL_ROBOT_EDGE = (24, 24, 28)
COL_TEXT = (28, 28, 32)
COL_TEXT_DIM = (112, 112, 118)
COL_EXPAND_START = (255, 226, 168)   # first cells expanded
COL_EXPAND_END = (244, 118, 44)      # last cells expanded

# Algorithms worth showing, in the order they should read.
PANEL_ORDER = ["dijkstra", "astar", "weighted_astar", "gbfs", "theta_star", "d_star_lite"]
LABELS = {
    "dijkstra": "Dijkstra",
    "astar": "A*",
    "weighted_astar": "Weighted A*",
    "gbfs": "GBFS",
    "theta_star": "Theta*",
    "d_star_lite": "D* Lite",
    "jps": "JPS",
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("dump", help="binary dump from algo_plan_dump")
    p.add_argument("map", help="the map PGM the dump was produced from")
    p.add_argument("out", help="output .gif")
    p.add_argument("--panel", type=int, default=322, help="panel edge in px")
    p.add_argument("--cols", type=int, default=3)
    p.add_argument("--search-frames", type=int, default=40)
    p.add_argument("--path-frames", type=int, default=26)
    p.add_argument("--hold-frames", type=int, default=14)
    p.add_argument("--fps", type=float, default=10.0)
    p.add_argument("--colors", type=int, default=128)
    p.add_argument("--mp4", default=None, help="also write an mp4 alongside the gif")
    return p.parse_args()


# --- inputs ------------------------------------------------------------------

def read_pgm(path: str) -> np.ndarray:
    """Binary PGM (P5) -> uint8 array, row 0 = top."""
    with open(path, "rb") as fh:
        data = fh.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            while data[pos:pos + 1] != b"\n":
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    assert tokens[0] == b"P5", "not a binary PGM"
    w, h = int(tokens[1]), int(tokens[2])
    pos += 1                                    # single whitespace after maxval
    pixels = np.frombuffer(data, dtype=np.uint8, count=w * h, offset=pos)
    return pixels.reshape(h, w)


def read_dump(path: str) -> dict:
    """Parse the algo_plan_dump binary format."""
    out = {}
    with open(path, "rb") as fh:
        blob = fh.read()
    pos = 0

    def take(fmt, size):
        nonlocal pos
        value = struct.unpack_from(fmt, blob, pos)[0]
        pos += size
        return value

    count = take("<I", 4)
    for _ in range(count):
        nlen = take("<I", 4)
        name = blob[pos:pos + nlen].decode()
        pos += nlen
        success = take("<B", 1) == 1
        cost = take("<d", 8)
        ms = take("<d", 8)
        iters = take("<Q", 8)
        npath = take("<I", 4)
        path = np.frombuffer(blob, dtype="<f8", count=npath * 2, offset=pos).reshape(-1, 2)
        pos += npath * 2 * 8
        nexp = take("<I", 4)
        expanded = np.frombuffer(blob, dtype="<i4", count=nexp, offset=pos)
        pos += nexp * 4
        out[name] = {"success": success, "cost": cost, "ms": ms, "iters": iters,
                     "path": path, "expanded": expanded.copy()}
    return out


# --- drawing -----------------------------------------------------------------

def load_fonts(panel: int):
    size_name = max(13, int(panel * 0.062))
    size_stat = max(11, int(panel * 0.046))
    size_head = max(12, int(panel * 0.050))
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    regular = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]

    def pick(paths, size):
        for p in paths:
            if os.path.exists(p):
                return ImageFont.truetype(p, size)
        return ImageFont.load_default()

    return pick(candidates, size_name), pick(regular, size_stat), pick(regular, size_head)


def build_background(pgm: np.ndarray, origin, resolution, size: int,
                     free_thresh: float, occupied_thresh: float) -> Image.Image:
    """Map -> panel-sized RGB background, using map_server's trinary rule."""
    occ = (255.0 - pgm.astype(np.float32)) / 255.0
    rgb = np.empty(pgm.shape + (3,), dtype=np.uint8)
    rgb[...] = COL_UNKNOWN
    rgb[occ <= free_thresh] = COL_FREE
    rgb[occ >= occupied_thresh] = COL_OCCUPIED

    img = Image.fromarray(rgb, "RGB")
    if img.size != (size, size):
        img = img.resize((size, size), Image.NEAREST)
    return img


def world_to_panel(x: float, y: float, origin, resolution, world_h: float, size: int):
    """World metres -> panel pixels. The grid's y axis runs upward."""
    px = (x - origin[0]) / resolution
    py = (y - origin[1]) / resolution
    py = world_h - py                      # flip: image row 0 is the top
    scale = size / (world_h)
    return px * scale, py * scale


def expansion_colours(n: int) -> np.ndarray:
    """Colour per expansion step, early cells pale and late cells saturated."""
    if n <= 0:
        return np.zeros((0, 3), dtype=np.uint8)
    t = np.linspace(0.0, 1.0, n, dtype=np.float32)[:, None]
    start = np.array(COL_EXPAND_START, dtype=np.float32)
    end = np.array(COL_EXPAND_END, dtype=np.float32)
    return (start + (end - start) * t).astype(np.uint8)


def render(dump: dict, pgm: np.ndarray, args, origin, resolution,
           free_thresh: float, occupied_thresh: float) -> list[Image.Image]:
    panel = args.panel
    names = [n for n in PANEL_ORDER if n in dump and dump[n]["success"]]
    if not names:
        raise SystemExit("no successful algorithms in the dump")

    rows = int(np.ceil(len(names) / args.cols))
    head_h = int(panel * 0.14)
    pad = 10
    width = args.cols * panel + (args.cols + 1) * pad
    height = rows * (panel + head_h) + (rows + 1) * pad
    world_h, world_w = pgm.shape
    size = panel

    fname, fstat, fhead = load_fonts(panel)

    background = build_background(pgm, origin, resolution, size,
                                  free_thresh, occupied_thresh)

    # Precompute per-panel reveal frames and colours.
    total_search = args.search_frames
    panels = []
    for name in names:
        rec = dump[name]
        expanded = rec["expanded"]
        colours = expansion_colours(len(expanded))
        # Cell index -> (col, row) in panel pixels.
        xs = (expanded % world_w).astype(np.int32)
        ys = (expanded // world_w).astype(np.int32)
        px = (xs * (size / world_w)).astype(np.int32)
        py = ((world_h - 1 - ys) * (size / world_h)).astype(np.int32)

        path_px = [world_to_panel(px_, py_, origin, resolution, world_h, size)
                   for px_, py_ in rec["path"]]
        panels.append({"name": name, "rec": rec, "px": px, "py": py,
                       "colours": colours, "path_px": path_px})

    frames: list[Image.Image] = []

    def compose(search_progress: float, path_progress: float) -> Image.Image:
        canvas = Image.new("RGB", (width, height), (255, 255, 255))
        draw = ImageDraw.Draw(canvas)
        for i, p in enumerate(panels):
            r, c = divmod(i, args.cols)
            ox = pad + c * (panel + pad)
            oy = pad + r * (panel + head_h + pad)

            rec = p["rec"]
            total = len(p["px"])
            shown = int(total * search_progress)

            tile = background.copy()
            tile_px = tile.load()
            for j in range(shown):
                col = tuple(int(v) for v in p["colours"][j])
                tile_px[int(p["px"][j]), int(p["py"][j])] = col

            tile_draw = ImageDraw.Draw(tile)
            tile_draw.rectangle([0, 0, size - 1, size - 1], outline=(206, 206, 202))

            # Path and robot.
            if path_progress > 0 and len(p["path_px"]) > 1:
                n = max(1, int(len(p["path_px"]) * path_progress))
                tile_draw.line(p["path_px"][:n], fill=COL_PATH, width=3, joint="curve")
                hx, hy = p["path_px"][n - 1]
                rad = max(3, int(size * 0.016))
                tile_draw.ellipse([hx - rad, hy - rad, hx + rad, hy + rad],
                                  fill=COL_ROBOT, outline=COL_ROBOT_EDGE, width=2)

            canvas.paste(tile, (ox, oy))

            # Header: name on the left, measurements on the right.
            label = LABELS.get(p["name"], p["name"])
            draw.text((ox + 2, oy + panel + 4), label, font=fname, fill=COL_TEXT)
            stat = f"{len(rec['expanded']):,} cells   {rec['ms']:.1f} ms"
            tw = draw.textlength(stat, font=fstat)
            draw.text((ox + panel - tw - 2, oy + panel + 6), stat,
                      font=fstat, fill=COL_TEXT_DIM)
        return canvas

    for i in range(total_search):
        frames.append(compose((i + 1) / total_search, 0.0))
    for i in range(args.path_frames):
        frames.append(compose(1.0, (i + 1) / args.path_frames))
    tail = compose(1.0, 1.0)
    for _ in range(args.hold_frames):
        frames.append(tail.copy())
    return frames


def save_gif(frames: list[Image.Image], out: str, fps: float, colors: int) -> float:
    strip = Image.new("RGB", (frames[0].width, frames[0].height * min(8, len(frames))))
    step = max(1, len(frames) // 8)
    for i, f in enumerate(frames[::step][:8]):
        strip.paste(f, (0, i * f.height))
    palette = strip.quantize(colors=colors, method=Image.MEDIANCUT)

    quant = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
    quant[0].save(out, save_all=True, append_images=quant[1:], optimize=True,
                  loop=0, duration=int(round(1000.0 / fps)), disposal=2)
    return os.path.getsize(out) / (1024 * 1024)


def save_mp4(frames: list[Image.Image], out: str, fps: float) -> None:
    import cv2
    h, w = frames[0].height, frames[0].width
    writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for f in frames:
        writer.write(cv2.cvtColor(np.array(f), cv2.COLOR_RGB2BGR))
    writer.release()


def main() -> int:
    args = parse_args()
    pgm = read_pgm(args.map)
    dump = read_dump(args.dump)

    origin = (-3.700, -6.342)
    resolution = 0.050
    frames = render(dump, pgm, args, origin, resolution, 0.196, 0.65)
    print(f"frames : {len(frames)}  {frames[0].width}x{frames[0].height}")

    mb = save_gif(frames, args.out, args.fps, args.colors)
    print(f"wrote  : {args.out}  ({mb:.2f} MB)")

    if args.mp4:
        save_mp4(frames, args.mp4, args.fps)
        print(f"wrote  : {args.mp4}  "
              f"({os.path.getsize(args.mp4) / 1048576:.2f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
