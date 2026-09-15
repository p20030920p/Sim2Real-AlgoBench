#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Screen capture for the README recordings, using only what is already here.

This machine has xwd but no ImageMagick, no ffmpeg, no scrot and no Python
screen-capture package, and pip refuses to install one (PEP 668). xwd writes a
plain XWD file, which is a 100-byte big-endian header followed by the window
name, an optional colour map and the pixels, so it is straightforward to read
without any dependency.

Usage:
    python3 tools/xwd_capture.py --list
    python3 tools/xwd_capture.py --window "RViz" --out /tmp/shot.png
    python3 tools/xwd_capture.py --record "RViz" --out-dir /tmp/frames --fps 5 --seconds 20
"""

from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
import time

import numpy as np
from PIL import Image

HEADER_FIELDS = 25


def read_xwd(path: str) -> Image.Image:
    """Parse an XWD file into a PIL image."""
    with open(path, "rb") as fh:
        blob = fh.read()

    header = struct.unpack(">25I", blob[: HEADER_FIELDS * 4])
    (header_size, _version, _format, _depth, width, height, _xoffset,
     byte_order, _bitmap_unit, _bitmap_bit_order, _bitmap_pad, bits_per_pixel,
     bytes_per_line, _visual_class, red_mask, green_mask, blue_mask, _bits_rgb,
     _cmap_entries, ncolors, *_rest) = header

    pos = header_size
    # Window name: null terminated, padded to a four byte boundary.
    end = blob.index(b"\0", pos)
    pos = end + 1
    pos += (-pos) % 4
    pos += ncolors * 12                       # colour map entries, if any

    expected = bytes_per_line * height
    # Take the pixels from the end of the file rather than from a computed
    # offset. The window-name field's padding to a four byte boundary is not
    # specified consistently between writers, but the pixel block is always
    # last, so anchoring there is immune to that.
    raw = blob[len(blob) - expected:]
    if len(raw) < expected:
        raise ValueError(f"truncated XWD: wanted {expected} bytes, got {len(raw)}")

    channels = bits_per_pixel // 8
    pixels = np.frombuffer(raw, dtype=np.uint8).reshape(height, bytes_per_line)
    pixels = pixels[:, : width * channels].reshape(height, width, channels)

    # The masks tell us the channel order; for the common 24/32-bit TrueColor
    # case this resolves to BGR or BGRA as stored.
    if channels >= 3:
        if red_mask == 0x00FF0000:
            rgb = pixels[:, :, [2, 1, 0]]
        else:
            rgb = pixels[:, :, [0, 1, 2]]
    else:
        rgb = np.repeat(pixels[:, :, :1], 3, axis=2)

    return Image.fromarray(np.ascontiguousarray(rgb), "RGB")


def find_window(pattern: str) -> str | None:
    """Return the id of the client window whose name contains pattern.

    The same title appears twice in the tree on this desktop: once for the
    compositor's decoration frame and once for the real client. Grabbing the
    decoration frame fails with BadMatch on X_GetImage - which is why a
    recording can come out with one half completely blank - so the client
    window has to be preferred explicitly rather than taking the first match.
    """
    try:
        tree = subprocess.run(["xwininfo", "-root", "-tree"],
                              capture_output=True, text=True, timeout=10).stdout
    except Exception:
        return None

    fallback = None
    for line in tree.splitlines():
        if pattern.lower() not in line.lower():
            continue
        # "0x4a00007 \"RViz\": (\"rviz2\" \"RViz\")  1200x800+10+10  +10+10"
        parts = line.strip().split()
        if not parts or not parts[0].startswith("0x"):
            continue
        lowered = line.lower()
        window_id = parts[0]
        if "mutter" in lowered or "decoration" in lowered:
            continue
        # Prefer the window whose WM_CLASS names the application itself.
        if f'("{pattern.lower()}"' in lowered:
            return window_id
        if fallback is None:
            fallback = window_id
    return fallback


def grab(window_id: str | None, out: str) -> None:
    target = ["-id", window_id] if window_id else ["-root"]
    with open(out, "wb") as fh:
        subprocess.run(["xwd", "-silent", *target], stdout=fh, check=True,
                       timeout=20)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="list window names and ids")
    ap.add_argument("--window", default=None, help="substring of the window title")
    ap.add_argument("--out", default=None, help="write a single PNG here")
    ap.add_argument("--record", default=None, metavar="WINDOW",
                    help="record this window repeatedly")
    ap.add_argument("--out-dir", default=None, help="frame directory for --record")
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()

    if args.list:
        tree = subprocess.run(["xwininfo", "-root", "-tree"],
                              capture_output=True, text=True, timeout=10).stdout
        for line in tree.splitlines():
            if '"' in line:
                print(line.strip())
        return 0

    if args.record:
        os.makedirs(args.out_dir, exist_ok=True)
        window_id = find_window(args.record)
        if window_id is None:
            print(f"no window matching {args.record!r}", file=sys.stderr)
            return 1
        print(f"recording window {window_id} ({args.record}) at {args.fps} fps")

        interval = 1.0 / args.fps
        deadline = time.time() + args.seconds
        index = 0
        tmp = os.path.join(args.out_dir, ".tmp.xwd")
        while time.time() < deadline:
            started = time.time()
            try:
                grab(window_id, tmp)
                read_xwd(tmp).save(os.path.join(args.out_dir, f"f{index:05d}.png"))
                index += 1
            except Exception as exc:                       # keep recording
                print(f"  frame {index} dropped: {exc}", file=sys.stderr)
            time.sleep(max(0.0, interval - (time.time() - started)))
        if os.path.exists(tmp):
            os.remove(tmp)
        print(f"captured {index} frames into {args.out_dir}")
        return 0 if index else 1

    window_id = find_window(args.window) if args.window else None
    if args.out is None:
        print("nothing to do: pass --out, --record or --list", file=sys.stderr)
        return 2
    grab(window_id, "/tmp/.single.xwd")
    read_xwd("/tmp/.single.xwd").save(args.out)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
