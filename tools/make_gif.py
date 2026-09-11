#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Convert a screen recording into a GitHub-safe animated GIF.

GitHub renders GIF inline in Markdown but does not play an .mp4 from a relative
repository path, so the demonstration videos are shipped as GIFs. A naive
conversion is useless — the source here is 1920x1080 at 60 fps, and a
full-rate, full-resolution GIF would run to hundreds of megabytes. Three things
keep it small:

* downscale to GitHub's actual content width, so the browser is not asked to
  shrink a huge image;
* drop the frame rate to something a screen recording does not need;
* quantise every frame against ONE shared palette, which lets the GIF encoder
  store only the rectangles that changed between frames.

Dithering is off by default: on flat UI and simulation imagery it adds moving
pixel noise that both inflates the file and looks worse.

Usage:
    python3 tools/make_gif.py docs/media/dynamic_obstacle.mp4 \\
        docs/media/dynamic_obstacle.gif --width 960 --fps 12
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np
from PIL import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", help="input video")
    parser.add_argument("output", help="output .gif")
    parser.add_argument("--width", type=int, default=960,
                        help="output width in px; GitHub renders at ~900 CSS px")
    parser.add_argument("--fps", type=float, default=12.0, help="output frame rate")
    parser.add_argument("--colors", type=int, default=192, help="palette size")
    parser.add_argument("--start", type=float, default=0.0, help="start time in seconds")
    parser.add_argument("--duration", type=float, default=None,
                        help="clip length in seconds; default is the whole video")
    parser.add_argument("--dither", action="store_true",
                        help="enable Floyd-Steinberg dithering (photographic material only)")
    parser.add_argument("--crop", default=None, metavar="L,T,R,B",
                        help="crop the source to this box in source pixels before scaling. "
                             "Removing desktop chrome and the taskbar matters more than it "
                             "looks: a strip of small text and icons is the most expensive "
                             "thing a GIF can contain.")
    parser.add_argument("--max-mb", type=float, default=4.5,
                        help="fail loudly above this size instead of shipping a heavy asset")
    return parser.parse_args()


def parse_crop(spec: str | None) -> tuple[int, int, int, int] | None:
    if not spec:
        return None
    parts = [int(v) for v in spec.split(",")]
    if len(parts) != 4:
        raise SystemExit("--crop expects L,T,R,B")
    return parts[0], parts[1], parts[2], parts[3]


def read_frames(source: str, width: int, fps: float,
                start: float, duration: float | None,
                crop: tuple[int, int, int, int] | None) -> tuple[list[Image.Image], float]:
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {source}")

    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    src_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    first = int(round(start * src_fps))
    last = src_count if duration is None else min(src_count, first + int(round(duration * src_fps)))
    step = max(1, int(round(src_fps / fps)))

    if crop:
        left, top, right, bottom = crop
        crop_w = (right - left) if right > 0 else src_w - left
        crop_h = (bottom - top) if bottom > 0 else src_h - top
    else:
        left = top = 0
        crop_w, crop_h = src_w, src_h

    out_h = int(round(crop_h * width / crop_w))
    out_h -= out_h % 2          # even height keeps the scaler and encoder happy

    frames: list[Image.Image] = []
    index = first
    cap.set(cv2.CAP_PROP_POS_FRAMES, first)
    while index < last:
        ok, frame = cap.read()
        if not ok:
            break
        if (index - first) % step == 0:
            if crop:
                frame = frame[top:top + crop_h, left:left + crop_w]
            small = cv2.resize(frame, (width, out_h), interpolation=cv2.INTER_AREA)
            frames.append(Image.fromarray(cv2.cvtColor(small, cv2.COLOR_BGR2RGB)))
        index += 1
    cap.release()

    if not frames:
        raise SystemExit("no frames decoded")

    effective_fps = src_fps / step
    print(f"source : {src_w}x{src_h} @ {src_fps:.1f} fps, {src_count} frames")
    if crop:
        print(f"crop   : {crop_w}x{crop_h} at ({left},{top})")
    print(f"output : {width}x{out_h} @ {effective_fps:.1f} fps, {len(frames)} frames")
    return frames, effective_fps


def shared_palette(frames: list[Image.Image], colors: int) -> Image.Image:
    """Build one palette from frames spread across the clip.

    A per-frame palette would give slightly better colour per frame but destroy
    inter-frame redundancy, which is where the compression actually comes from.
    """
    picks = frames[:: max(1, len(frames) // 12)][:12]
    strip = Image.new("RGB", (frames[0].width, frames[0].height * len(picks)))
    for i, frame in enumerate(picks):
        strip.paste(frame, (0, i * frame.height))
    return strip.quantize(colors=colors, method=Image.MEDIANCUT)


def main() -> int:
    args = parse_args()

    frames, fps = read_frames(
        args.source, args.width, args.fps, args.start, args.duration, parse_crop(args.crop))

    palette = shared_palette(frames, args.colors)
    dither = Image.Dither.FLOYDSTEINBERG if args.dither else Image.Dither.NONE
    quantised = [f.quantize(palette=palette, dither=dither) for f in frames]

    # Frame 0 must carry the full image; later frames are delta encoded against
    # their predecessor, which is what `optimize` plus a shared palette enables.
    quantised[0].save(
        args.output,
        save_all=True,
        append_images=quantised[1:],
        optimize=True,
        loop=0,
        duration=int(round(1000.0 / fps)),
        disposal=2,
    )

    size_mb = os.path.getsize(args.output) / (1024 * 1024)
    print(f"wrote  : {args.output}  ({size_mb:.2f} MB)")

    if size_mb > args.max_mb:
        print(
            f"WARNING: {size_mb:.2f} MB exceeds the {args.max_mb:.1f} MB budget. "
            f"Lower --width, --fps or --colors, or clip with --start/--duration.",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
