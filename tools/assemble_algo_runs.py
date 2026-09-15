#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Build the planner-comparison media from recorded competition runs.

Each run comes from ``tools/record_autonomy_run.py``: one complete autonomous
race in Gazebo with the global planner served by a different algorithm, and the
same scenario, costmap, controller and behaviour tree every time. This turns
those frame directories into the GIFs and the summary table the README embeds.

Usage:
    python3 tools/assemble_algo_runs.py --runs /tmp/race_run/runs \
        --out docs/media/algorithms
"""

from __future__ import annotations

import argparse
import json
import os

from PIL import Image


def outcome_of(meta: dict) -> str:
    if meta.get('outcome') == 'COMPLETE':
        return 'COMPLETE'
    state = meta.get('last_state') or ''
    if state in ('SEARCH_EXHAUSTED', 'FAILED'):
        return 'SEARCH_EXHAUSTED'
    return state or 'NO RUN'


def load_run(run_dir: str) -> dict | None:
    meta_path = os.path.join(run_dir, 'meta.json')
    run_path = os.path.join(run_dir, 'run.json')
    if not os.path.exists(meta_path):
        return None
    with open(meta_path, 'r', encoding='utf-8') as handle:
        meta = json.load(handle)
    run = None
    if os.path.exists(run_path):
        with open(run_path, 'r', encoding='utf-8') as handle:
            run = json.load(handle)
    video = os.path.join(run_dir, 'video.mp4')
    meta['_dir'] = run_dir
    meta['_video'] = video if os.path.exists(video) else None
    meta['_run'] = run
    return meta


def travelled(run: dict | None) -> float:
    if not run:
        return 0.0
    pts = [f['pose'] for f in run.get('frames', []) if f.get('pose')]
    return sum(((pts[i][0] - pts[i - 1][0]) ** 2 +
                (pts[i][1] - pts[i - 1][1]) ** 2) ** 0.5
               for i in range(1, len(pts)))


def _read_video(path: str) -> list:
    import cv2
    cap = cv2.VideoCapture(path)
    frames = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frames.append(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
    cap.release()
    return frames


def make_gif(meta: dict, out_path: str, width: int, fps: float) -> None:
    if not meta.get('_video'):
        return
    images = _read_video(meta['_video'])
    if not images:
        return
    # A race runs at a fraction of real time here and the state machine spends
    # long stretches scanning, so sample evenly and cap the clip length.
    images = _evenly(images, 90)
    if width and images[0].width != width:
        images = [im.resize((width, int(round(im.height * width / im.width))),
                            Image.LANCZOS) for im in images]

    strip = Image.new('RGB', (images[0].width, images[0].height * min(6, len(images))))
    step = max(1, len(images) // 6)
    for i, im in enumerate(images[::step][:6]):
        strip.paste(im, (0, i * im.height))
    palette = strip.quantize(colors=96, method=Image.MEDIANCUT)
    quant = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in images]
    quant[0].save(out_path, save_all=True, append_images=quant[1:],
                  optimize=True, loop=0,
                  duration=int(round(1000.0 / fps)), disposal=2)


def _evenly(items: list, count: int) -> list:
    if len(items) <= count:
        return items
    stride = len(items) / float(count)
    return [items[min(len(items) - 1, int(i * stride))] for i in range(count)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True, help='directory of per-algorithm run dirs')
    ap.add_argument('--out', required=True, help='output media directory')
    ap.add_argument('--width', type=int, default=420)
    ap.add_argument('--fps', type=float, default=9.0)
    ap.add_argument('--gif-dir-name', default='algorithms')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    names = sorted(d for d in os.listdir(args.runs)
                   if os.path.isfile(os.path.join(args.runs, d, 'meta.json')))
    rows = []
    for name in names:
        meta = load_run(os.path.join(args.runs, name))
        if meta is None:
            continue
        gif = os.path.join(args.out, f'{name}.gif')
        make_gif(meta, gif, args.width, args.fps)
        row = {
            'algorithm': name,
            'outcome': outcome_of(meta),
            'frames': len(meta.get('_run', {}).get('frames', [])) if meta.get('_run') else 0,
            'gif': gif,
            'gif_mb': os.path.getsize(gif) / 1048576 if os.path.exists(gif) else 0.0,
            'plan_poses': meta.get('plan_poses', 0),
            'travelled_m': travelled(meta.get('_run')),
        }
        rows.append(row)
        print('%-16s %-18s frames=%-5d plan=%-4d travelled=%.1f m  gif=%.1f MB' % (
            row['algorithm'], row['outcome'], row['frames'],
            row['plan_poses'], row['travelled_m'], row['gif_mb']))

    with open(os.path.join(args.out, 'comparison.json'), 'w', encoding='utf-8') as handle:
        json.dump(rows, handle, indent=2)
    print(f'\nwrote {os.path.join(args.out, "comparison.json")}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
