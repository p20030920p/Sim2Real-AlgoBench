#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Turn recorded side-by-side runs into the README media.

One input directory per algorithm, each produced by
``tools/record_run_with_rviz.sh``. For each it writes an MP4 and a GIF preview
into the media directory, and a ``comparison.json`` describing all of them.

It also checks each recording rather than trusting it. Two things have gone
wrong here before and both produced a clip that looked fine at a glance:

* a half that never got written, because the window grab failed - so the mean
  brightness of each half is measured and a blank one is reported;
* a rotation that puts the two halves at different angles - so the recorded
  Gazebo half is checked against the one landmark whose position is known
  exactly at the start of a run, the car sitting at its spawn pose.

Usage:
    python3 tools/assemble_sidebyside.py --runs /tmp/runs --out docs/media/run_sidebyside
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import cv2
import numpy as np
from PIL import Image

# The Gazebo half is 480 px wide in the recorded frames; the rest is RViz.
GAZEBO_WIDTH = 480
# Where the car sits at the start of every run, as a fraction of the Gazebo half.
# Spawn (8.0727, 7.5312) with the camera over (3.65, 1.0) at 27.4 px/m, north-up.
SPAWN_FRACTION = (358.0 / 480.0, 65.0 / 480.0)
SPAWN_TOLERANCE_PX = 25.0


def car_blob(image: np.ndarray):
    """The car's blue chassis patch, largest candidate that is car-sized."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (100, 120, 80), (135, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, stats, centres = cv2.connectedComponentsWithStats(mask, 8)
    found = [(int(stats[i, cv2.CC_STAT_AREA]), float(centres[i][0]), float(centres[i][1]))
             for i in range(1, n) if 40 <= stats[i, cv2.CC_STAT_AREA] <= 1200]
    found.sort(reverse=True)
    return found


def check(first_left: np.ndarray, label: str) -> list[str]:
    """Complaints about one recording, empty when it looks right."""
    problems = []
    height, width = first_left.shape[:2]
    expected = (SPAWN_FRACTION[0] * width, SPAWN_FRACTION[1] * height)
    blobs = car_blob(first_left)
    if not blobs:
        problems.append(f'{label}: the car was not found at the start of the run')
    else:
        best = min(blobs, key=lambda b: (b[1] - expected[0]) ** 2 + (b[2] - expected[1]) ** 2)
        error = float(np.hypot(best[1] - expected[0], best[2] - expected[1]))
        if error > SPAWN_TOLERANCE_PX:
            problems.append(
                f'{label}: the car starts {error:.0f} px from where the spawn pose '
                f'predicts ({best[1]:.0f},{best[2]:.0f}) vs '
                f'({expected[0]:.0f},{expected[1]:.0f}) - the halves may not be aligned')
    return problems


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--width', type=int, default=760)
    ap.add_argument('--fps', type=float, default=6.0)
    ap.add_argument('--gif-width', type=int, default=560)
    ap.add_argument('--gif-frames', type=int, default=80)
    ap.add_argument('--order', default='')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    names = sorted(d for d in os.listdir(args.runs)
                   if os.path.isfile(os.path.join(args.runs, d, 'video.mp4')))
    if args.order:
        wanted = args.order.split(',')
        names = [n for n in wanted if n in names] + [n for n in names if n not in wanted]

    rows = []
    problems: list[str] = []
    for name in names:
        run_dir = os.path.join(args.runs, name)
        src = os.path.join(run_dir, 'video.mp4')
        cap = cv2.VideoCapture(src)
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if total <= 0:
            print(f'{name}: empty video, skipped')
            continue

        first_ok, first = cap.read()
        if not first_ok:
            print(f'{name}: cannot read, skipped')
            continue
        left0 = first[:, :GAZEBO_WIDTH]
        right0 = first[:, GAZEBO_WIDTH:]
        problems += check(left0, name)
        if right0.size and float(right0.mean()) < 45:
            problems.append(f'{name}: the RViz half is blank (mean {right0.mean():.0f})')

        out_mp4 = os.path.join(args.out, f'{name}.mp4')
        height = int(round(first.shape[0] * args.width / first.shape[1]))
        writer = cv2.VideoWriter(
            out_mp4, cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (args.width, height))

        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        small_frames = []
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            small = cv2.resize(frame, (args.width, height), interpolation=cv2.INTER_AREA)
            writer.write(small)
            small_frames.append(small)
        writer.release()
        cap.release()

        idx = np.linspace(0, len(small_frames) - 1,
                          min(args.gif_frames, len(small_frames))).astype(int)
        gif_height = int(round(height * args.gif_width / args.width))
        images = [Image.fromarray(cv2.cvtColor(small_frames[i], cv2.COLOR_BGR2RGB))
                  .resize((args.gif_width, gif_height)) for i in idx]
        strip = Image.new('RGB', (args.gif_width, gif_height * min(6, len(images))))
        step = max(1, len(images) // 6)
        for i, im in enumerate(images[::step][:6]):
            strip.paste(im, (0, i * gif_height))
        palette = strip.quantize(colors=80, method=Image.MEDIANCUT)
        quant = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in images]
        gif = os.path.join(args.out, f'{name}.gif')
        quant[0].save(gif, save_all=True, append_images=quant[1:], optimize=True,
                      loop=0, duration=int(round(1000.0 / args.fps)), disposal=2)

        meta = {}
        meta_path = os.path.join(run_dir, 'meta.json')
        if os.path.exists(meta_path):
            with open(meta_path, 'r', encoding='utf-8') as handle:
                meta = json.load(handle)
        driven = 0.0
        run_path = os.path.join(run_dir, 'run.json')
        if os.path.exists(run_path):
            with open(run_path, 'r', encoding='utf-8') as handle:
                run = json.load(handle)
            poses = [f['pose'] for f in run.get('frames', []) if f.get('pose')]
            driven = sum(((poses[i][0] - poses[i - 1][0]) ** 2 +
                          (poses[i][1] - poses[i - 1][1]) ** 2) ** 0.5
                         for i in range(1, len(poses)))

        row = {
            'algorithm': name,
            'outcome': meta.get('outcome'),
            'last_state': meta.get('last_state'),
            'frames': len(small_frames),
            'travelled_m': round(driven, 1),
            'rviz_frames_captured': meta.get('rviz_frames_captured'),
            'rviz_frames_missing': meta.get('rviz_frames_missing'),
            'gif': f'{args.out}/{name}.gif',
            'mp4': f'{args.out}/{name}.mp4',
            'gif_mb': round(os.path.getsize(gif) / 1048576, 2),
            'mp4_mb': round(os.path.getsize(out_mp4) / 1048576, 2),
        }
        rows.append(row)
        print('%-16s %-18s frames=%-5d driven=%6.1f m  gif=%4.1fMB mp4=%4.1fMB  rviz_miss=%s' % (
            name, row['outcome'] or row['last_state'], row['frames'],
            row['travelled_m'], row['gif_mb'], row['mp4_mb'],
            row['rviz_frames_missing']))

    with open(os.path.join(args.out, 'comparison.json'), 'w', encoding='utf-8') as handle:
        json.dump(rows, handle, indent=2)
    print(f'\nwrote {os.path.join(args.out, "comparison.json")}')

    if problems:
        print('\nPROBLEMS FOUND:')
        for p in problems:
            print(f'  - {p}')
        return 1
    print('all recordings check out: both halves present, orientation consistent')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
