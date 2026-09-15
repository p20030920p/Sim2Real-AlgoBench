#!/usr/bin/env bash
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
#
# Record one complete autonomous race per global planner, in the same scenario.
#
# For each algorithm in the registry the full competition stack is started
# (Gazebo, AMCL, Nav2, vision, state machine) with only the global planner
# swapped, then one start signal is sent and the whole run is recorded from a
# camera above the arena. The result is directly comparable: same map, same
# costmap, same controller, same behaviour tree, same start pose.
#
# Usage:
#   tools/run_algorithm_comparison.sh [out_dir] [algorithm ...]
#
# Notes that cost time to rediscover:
#   * ROS_DOMAIN_ID is set here on purpose. Other simulations on the default
#     domain publish their own /tf, which leaves the costmap unable to connect
#     'map' to 'base_footprint' and Nav2 never activates.
#   * The whole stack runs in its own process group. An orphaned race_metrics
#     keeps writing to the shared summary CSV, so a later run's report gains
#     rows it never produced.
#   * Gazebo runs at roughly 0.3-0.5 of real time on a software-rendered VM,
#     so a 100 s race takes several hundred seconds of wall clock.

set +u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_ROOT="${1:-/tmp/race_algo_runs}"
shift || true
ALGORITHMS=("$@")
if [ ${#ALGORITHMS[@]} -eq 0 ]; then
  ALGORITHMS=(astar dijkstra weighted_astar gbfs theta_star d_star_lite)
fi

DOMAIN="${RACE_DOMAIN_ID:-42}"
RECORD_SECONDS="${RACE_RECORD_SECONDS:-1500}"

mkdir -p "$OUT_ROOT"
cd "$REPO"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

export ROS_LOG_DIR="${RACE_LOG_DIR:-$OUT_ROOT/roslog}"
export ROS_DOMAIN_ID="$DOMAIN"
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share:$REPO/install/race_gazebo/share/race_gazebo/models"
export RACE_REPORT_DIR="$REPO/reports"
mkdir -p "$ROS_LOG_DIR"

# Camera model: 3.65 m above the arena centre looking straight down.
CAM_SDF="$OUT_ROOT/topcam.sdf"
cat > "$CAM_SDF" <<'SDF'
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="topcam">
    <static>true</static>
    <pose>3.65 1.0 10.5 0 1.5708 0</pose>
    <link name="link">
      <sensor name="view" type="camera">
        <topic>/top_view</topic>
        <update_rate>10</update_rate>
        <camera>
          <horizontal_fov>1.3962634</horizontal_fov>
          <image><width>480</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>60.0</far></clip>
        </camera>
        <always_on>true</always_on>
        <visualize>true</visualize>
      </sensor>
    </link>
  </model>
</sdf>
SDF

run_one() {
  local algo="$1" out="$2"
  local log="$OUT_ROOT/logs"
  mkdir -p "$out" "$log"
  rm -f "$out"/*.png "$out"/*.json "$out"/*.mp4

  export RACE_GLOBAL_PLANNER="$algo"

  # Bringing the stack up is not always successful first time: AMCL sometimes
  # never takes its initial pose, so it never publishes map -> odom, the global
  # costmap refuses to activate, and the run cannot start. Retrying is cheaper
  # than diagnosing it, and the retry is invisible in the output.
  local attempt up comp pgid
  for attempt in 1 2 3; do
    echo "[$(date +%T)] $algo: launching stack (attempt $attempt)"
    rm -f "$log/comp_$algo.log"
    setsid ros2 launch race_navigation competition.launch.py \
        headless:=true stress:=false nav_rviz:=false render_engine:=ogre \
        > "$log/comp_$algo.log" 2>&1 &
    comp=$!
    pgid=$(ps -o pgid= -p "$comp" | tr -d ' ')

    # Wait for the autonomy node to announce itself AND for the navigation stack
    # to finish activating. Signalling on the announcement alone races the
    # lifecycle manager: the signal is accepted, but bt_navigator is still
    # inactive, so every goal is rejected and the robot never leaves the start.
    up=0
    for _ in $(seq 1 110); do
      if grep -qa "waiting for one-button start" "$log/comp_$algo.log" 2>/dev/null &&
         grep -qa "Managed nodes are active" "$log/comp_$algo.log" 2>/dev/null &&
         grep -qa "initialPoseReceived" "$log/comp_$algo.log" 2>/dev/null; then
        up=1; break
      fi
      sleep 2
    done
    if [ "$up" = 1 ]; then break; fi

    echo "[$(date +%T)] $algo: stack did not become ready, retrying"
    kill -TERM -- "-$pgid" 2>/dev/null; sleep 5
    kill -KILL -- "-$pgid" 2>/dev/null; sleep 8
  done
  if [ "$up" != 1 ]; then
    echo "[$(date +%T)] $algo: giving up after $attempt attempts"
    return 1
  fi
  # Let AMCL publish map -> odom and the costmaps settle.
  sleep 15

  echo "[$(date +%T)] $algo: camera + bridge"
  ros2 run ros_gz_sim create -file "$CAM_SDF" -name topcam \
      -x 3.65 -y 1.0 -z 10.5 -P 1.5708 >> "$log/comp_$algo.log" 2>&1
  ros2 run ros_gz_bridge parameter_bridge \
      "/top_view@sensor_msgs/msg/Image[gz.msgs.Image" \
      > "$log/bridge_$algo.log" 2>&1 &
  local bridge=$!
  # Wait for the camera topic to exist rather than guessing a delay.
  local cam=0
  for _ in $(seq 1 30); do
    if timeout 8 ros2 topic list 2>/dev/null | grep -qx "/top_view"; then
      cam=1; break
    fi
    sleep 2
  done
  if [ "$cam" != 1 ]; then
    echo "[$(date +%T)] $algo: /top_view never appeared; recording without video"
  fi
  sleep 3

  echo "[$(date +%T)] $algo: recording"
  python3 "$REPO/tools/record_autonomy_run.py" --algorithm "$algo" \
      --out-dir "$out" --seconds "$RECORD_SECONDS" --fps 3 --width 520 \
      > "$log/rec_$algo.log" 2>&1 &
  local rec=$!
  # The recorder must be subscribed before the signal, or the first seconds of
  # the run - which is when the robot starts moving - are simply not recorded.
  sleep 12

  echo "[$(date +%T)] $algo: one start signal"
  python3 "$REPO/tools/send_start_signal.py" >> "$log/rec_$algo.log" 2>&1

  wait "$rec"
  echo "[$(date +%T)] $algo: recording finished"

  kill "$bridge" 2>/dev/null
  kill -TERM -- "-$pgid" 2>/dev/null
  sleep 6
  kill -KILL -- "-$pgid" 2>/dev/null
  sleep 4
  cat "$out/meta.json" 2>/dev/null
  echo
}

for algo in "${ALGORITHMS[@]}"; do
  echo "=================== $algo ==================="
  run_one "$algo" "$OUT_ROOT/$algo" || true
done

echo "ALL DONE"
