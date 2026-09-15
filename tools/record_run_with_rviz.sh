#!/usr/bin/env bash
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
#
# Record one complete autonomous race, from the start signal to the finish,
# with the planner's own path drawn into the world and RViz alongside.
#
# Both overlays are correct by construction rather than by calibration:
#
#   * the path is spawned into the Gazebo world as flat markers generated from
#     the path Nav2 published, so Gazebo renders it in the same projection as
#     the arena and the car. No world -> pixel mapping is involved, which
#     matters because every attempt to solve one here was metres out;
#   * RViz draws the same path from the same topic through TF, and shows the
#     robot, the costmap and the scan.
#
# The result is a clip where the left half is what the simulator did and the
# right half is what the navigation stack believed, frame for frame.
#
# Usage:
#   tools/record_run_with_rviz.sh astar /tmp/run_astar [seconds]
#
# Notes that cost time:
#   * ROS_DOMAIN_ID is set on purpose; other simulations on the default domain
#     publish their own /tf and the costmap then cannot connect map to
#     base_footprint, so Nav2 never activates.
#   * The stack runs in its own process group. An orphaned race_metrics keeps
#     writing to the shared summary CSV.
#   * The start signal must not be sent until AMCL has its initial pose and the
#     navigation nodes are active, otherwise bt_navigator rejects every goal
#     and the robot never leaves the start.

set +u

ALGO="${1:?usage: record_run_with_rviz.sh <algorithm> <out_dir> [seconds]}"
OUT="${2:?usage: record_run_with_rviz.sh <algorithm> <out_dir> [seconds]}"
SECONDS_TO_RECORD="${3:-1500}"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGS="$OUT/logs"
mkdir -p "$OUT" "$LOGS"
rm -f "$OUT"/*.png "$OUT"/*.json "$OUT"/*.mp4

cd "$REPO"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

export ROS_LOG_DIR="$LOGS/roslog"
export ROS_DOMAIN_ID="${RACE_DOMAIN_ID:-42}"
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share:$REPO/install/race_gazebo/share/race_gazebo/models"
export RACE_GLOBAL_PLANNER="$ALGO"
export RACE_REPORT_DIR="$REPO/reports"
mkdir -p "$ROS_LOG_DIR"

teardown() {
  [ -n "${PGID:-}" ] && { kill -TERM -- "-$PGID" 2>/dev/null; sleep 5; kill -KILL -- "-$PGID" 2>/dev/null; }
  [ -n "${RVIZ_PID:-}" ] && kill "$RVIZ_PID" 2>/dev/null
  [ -n "${BRIDGE_PID:-}" ] && kill "$BRIDGE_PID" 2>/dev/null
  sleep 3
}

echo "[$(date +%T)] $ALGO: launching the competition stack"
setsid ros2 launch race_navigation competition.launch.py \
    headless:=true stress:=false nav_rviz:=false render_engine:=ogre \
    > "$LOGS/stack.log" 2>&1 &
PGID=$(ps -o pgid= -p $! | tr -d ' ')

up=0
for _ in $(seq 1 120); do
  if grep -qa "waiting for one-button start" "$LOGS/stack.log" 2>/dev/null &&
     grep -qa "Managed nodes are active" "$LOGS/stack.log" 2>/dev/null &&
     grep -qa "initialPoseReceived" "$LOGS/stack.log" 2>/dev/null; then
    up=1; break
  fi
  sleep 2
done
if [ "$up" != 1 ]; then
  echo "[$(date +%T)] $ALGO: stack never became ready"; teardown; exit 1
fi
sleep 12

echo "[$(date +%T)] $ALGO: top camera"
ros2 run ros_gz_sim create -file "$REPO/tools/topcam.sdf" -name topcam \
    -x 3.65 -y 1.0 -z 10.5 -P 1.5708 >> "$LOGS/stack.log" 2>&1
ros2 run ros_gz_bridge parameter_bridge \
    "/top_view@sensor_msgs/msg/Image[gz.msgs.Image" \
    > "$LOGS/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 8

echo "[$(date +%T)] $ALGO: RViz"
ros2 run rviz2 rviz2 -d "$REPO/tools/live_run.rviz" > "$LOGS/rviz.log" 2>&1 &
RVIZ_PID=$!
sleep 25

echo "[$(date +%T)] $ALGO: recorder"
python3 "$REPO/tools/record_run_sidebyside.py" --algorithm "$ALGO" \
    --out-dir "$OUT" --seconds "$SECONDS_TO_RECORD" --fps 4 \
    > "$LOGS/record.log" 2>&1 &
REC=$!
sleep 10

echo "[$(date +%T)] $ALGO: one start signal"
python3 "$REPO/tools/send_start_signal.py" >> "$LOGS/record.log" 2>&1

wait "$REC"
echo "[$(date +%T)] $ALGO: recording finished"
teardown
echo "[$(date +%T)] $ALGO: done"
cat "$OUT/meta.json" 2>/dev/null
