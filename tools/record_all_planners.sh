#!/usr/bin/env bash
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
#
# Record the side-by-side clip for every registered global planner.
#
# Each run is the complete task - one start signal, AMCL, Nav2 planning and
# following the search viewpoints, the green-marker detection, the finish-pad
# approach and the 3-second hold - with only the global planner swapped. The
# clip is Gazebo on the left (no overlay: what the robot did) and RViz on the
# right (the map, costmap, scan, the planned path and the robot).
#
# The Gazebo half is rotated 180 degrees to match the world frame the map and
# RViz use. That is measured, not assumed: a marker at world (6.65, 4.00)
# renders at pixel (159, 153) with the camera over the arena centre (240, 240).
#
# Usage:
#   tools/record_all_planners.sh /tmp/runs [algorithm ...]
#
# Each run takes minutes: Gazebo is at a fraction of real time on a
# software-rendered VM.

set +u

OUT_ROOT="${1:-/tmp/race_planners}"
shift || true
ALGORITHMS=("$@")
if [ ${#ALGORITHMS[@]} -eq 0 ]; then
  # JPS is left out deliberately: it fails on the race map, so its clip would
  # show a robot that never moves. Add it explicitly to record that.
  ALGORITHMS=(dijkstra astar weighted_astar gbfs theta_star d_star_lite)
fi

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$OUT_ROOT"

for algo in "${ALGORITHMS[@]}"; do
  echo "=================== $algo ==================="
  "$REPO/tools/record_run_with_rviz.sh" "$algo" "$OUT_ROOT/$algo" \
      "${RACE_RECORD_SECONDS:-1200}" || echo "$algo: run failed"
  echo
done

echo "ALL DONE"
