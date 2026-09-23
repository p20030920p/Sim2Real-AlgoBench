<div align="center">

# Sim2Real-AlgoBench

**A three-wheeled omnidirectional robot autonomy benchmark — one task, one interface contract, one metric set, seven interchangeable global planners.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-8-F58113?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![Algorithms](https://img.shields.io/badge/algorithms-7%20registered-brightgreen)](#algorithm-library)

[The task](#the-task) &nbsp;•&nbsp; [Algorithm library](#algorithm-library) &nbsp;•&nbsp; [Demos](#demos) &nbsp;•&nbsp; [Quick start](#quick-start) &nbsp;•&nbsp; [Docs](#docs)

*English &nbsp;|&nbsp; [中文](README.zh-CN.md)*

</div>

<p align="center">
  <img src="docs/images/07_finish_reached.png" width="820" alt="Autonomous search and finish-pad approach"/>
</p>

<p align="center">
  <em>Autonomous search of the saved map, green-marker detection, finish-pad approach. No goal pose is sent by hand.</em>
</p>

## The task

The robot is given a start pose and one start signal. It searches a known map for a green A4 marker on a wall, drives onto the yellow pad in front of the marker, and stays still for 3 seconds.

The run ends when the marker is detected, not when a coordinate is reached. Perception is therefore on the critical path, which is what makes the comparison worth running.

## Where this comes from

The task, the arena and the scoring rules come from a comprehensive robotics competition, and the
same stack was run at the event on the physical vehicle and completed the task. What is in this
repository is the simulation side: the arena, the task state machine, the metric set and the planner
comparison, all of which run end to end in Gazebo. The hardware bring-up and calibration record is
not published here.

## Algorithm library

A planner is a subclass plus one registry entry: the Nav2 plugin, its parameters and one behaviour
tree per algorithm are generated from that entry, so a comparison changes the algorithm and nothing
else. Every run writes a JSON report, so two planners are compared on the same metrics.

The seven global planners are interchangeable Nav2 plugins. The active one is chosen by one line:

```yaml
# src/algo_bringup/config/algo_registry.yaml
active:
  planner: 1
```

The algorithms live in `algo_core`, which has no ROS dependency and can be unit tested on its own. `algo_nav2_plugins` wraps them behind a single `nav2_core::GlobalPlanner` adapter. `algo_bringup` holds the index, the parameters and the behaviour trees.

One planning call each, CPU only, on the development laptop — a thin-and-light Huawei MateBook 14 (Intel Core i5-1240P) — on the race map (294 × 294 at 0.05 m), planning from `(-3.0, -5.0)` to `(10.0, 7.0)`. The cell counts are the comparison that survives a change of machine; the milliseconds are indicative.

| Idx | Algorithm | Path | Poses | Cells expanded | Time |
| :---: | :--- | :---: | ---: | ---: | ---: |
| 0 | Dijkstra | yes | 21 | 55,578 | 5.0 ms |
| 1 | **A\*** | yes | 23 | 26,442 | 5.3 ms |
| 2 | Weighted A\* | yes | 29 | 9,025 | 1.4 ms |
| 3 | GBFS | yes | 22 | 3,127 | 0.6 ms |
| 4 | JPS | **no** | — | 5 | 0.1 ms |
| 5 | **Theta\*** | yes | **8** | 20,694 | 30.5 ms |
| 6 | **D\* Lite** | yes | 19 | 375 | 38.7 ms |
| 7 | Nav2 Theta\* | yes | 394 | — | — |

A\* expands 26,442 cells; D\* Lite expands 375, because it keeps its search between calls and only repairs what changed. Theta\* returns 8 poses where A\* returns 23; the difference is the grid staircase that any-angle planning removes. JPS is registered but does not plan on this map; the state of its pruning is in [`docs/ROADMAP.md`](docs/ROADMAP.md).

All registered planners stay loaded at once, and any of them can be selected per request:

| Selection | Mechanism |
| :--- | :--- |
| Per request | `ComputePathToPose` carries a `planner_id` field |
| Per situation | one behaviour tree per algorithm, chosen through `behavior_tree` |
| Outside Nav2 | `algo_core::Registry::instance().create("theta_star")` |

## Demos

### The task, with the planner's plan beside it

Gazebo on the left, RViz on the right, one complete run from the start signal to the 3-second hold. The left half is the simulator's own top-down camera, with no overlay; the right half is RViz drawing the same moment from the same topics — the saved map, the global costmap, the live LiDAR scan, the planned path in red, and the robot model. Nothing is composited or re-timed: both halves come from one run, and the state in the header comes from `/race/state`.

The top camera's image axes are not the world axes the map, the path and RViz use, so one half has to be turned before the two can be compared. The marker measurements that establish the rotation, and the rest of the recording setup, are in [`docs/RECORDING.md`](docs/RECORDING.md).

```bash
tools/record_all_planners.sh /tmp/planners        # every registered planner
```

Showing both halves is the point: Gazebo shows what the robot did, RViz shows what the navigation stack believed and the path it committed to. When they disagree, that is the interesting case.

### Every registered planner, same task, same everything else

| Global planner | Outcome | Driven | Clip |
| :--- | :---: | ---: | :---: |
| **Weighted A\*** | `COMPLETE` | 55.2 m | ![weighted_astar](docs/media/run_sidebyside/weighted_astar.gif) |
| **A\*** | `COMPLETE` | 36.6 m | ![astar](docs/media/run_sidebyside/astar.gif) |
| **Dijkstra** | `COMPLETE` | 33.1 m | ![dijkstra](docs/media/run_sidebyside/dijkstra.gif) |
| **D\* Lite** | `COMPLETE` | 35.4 m | ![d_star_lite](docs/media/run_sidebyside/d_star_lite.gif) |
| **Theta\*** | `COMPLETE` | 76.0 m | ![theta_star](docs/media/run_sidebyside/theta_star.gif) |
| **GBFS** | `COMPLETE` | 178.9 m | ![gbfs](docs/media/run_sidebyside/gbfs.gif) |
| **JPS** | `COMPLETE` | 143.9 m | ![jps](docs/media/run_sidebyside/jps.gif) |

All seven finish the task. Distance driven is not a ranking: it counts every replan and every viewpoint revisit the mission asked for, so GBFS's 178.9 m reflects how often it was sent back and forth rather than a bad path. The offline table above, measured on a single planning call, is the ranking; these clips are for seeing behaviour.

The clips are MP4 at `docs/media/run_sidebyside/<algorithm>.mp4`. To reproduce one run directly:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
ros2 launch race_navigation competition.launch.py headless:=true stress:=false
python3 tools/send_start_signal.py        # the one start signal
ros2 topic echo /race/state               # watch it finish
```

### Search shape

`docs/media/search_2d.gif` animates the order in which each planner expanded cells, from `algo_plan_dump`'s record of the real searches. The colour is **expansion order, normalised per panel**, so two panels of the same colour are at the same fraction of their own search, not at the same amount of work; the cell count printed under each panel is the comparison.

![Six planner searches side by side](docs/media/search_2d.gif)

<p align="center">
  <sub><a href="docs/media/search_2d.mp4">Download (MP4)</a> &nbsp;·&nbsp; rendered with <code>tools/render_planning_demo.py</code></sub>
</p>

### Dynamic-obstacle world

The same task with two moving obstacles sweeping the corridors. `stress:=true` selects it, loading `competition_stress.world` instead of `competition_world.world`. The planner comparison above passes `stress:=false`, so all seven planners face the same arena and the same costmap.

<p align="center">
  <sub><a href="docs/media/dynamic_obstacle.mp4">dynamic_obstacle.mp4 (1920 × 1080 at 60 fps)</a> &nbsp;·&nbsp; convert with <code>python3 tools/make_gif.py</code></sub>
</p>

## Gallery

| SLAM mapping | Map saved |
| :---: | :---: |
| ![SLAM mapping](docs/images/01_mapping.png) | ![Saved PGM map](docs/images/02_map_saved.png) |
| **Dynamic-obstacle world** | **TF tree** |
| ![Two moving obstacles in the arena](docs/images/05_stress_world.png) | ![TF tree](docs/images/06_tf_tree.png) |

## Quick start

```bash
git clone https://github.com/p20030920p/Sim2Real-AlgoBench.git
cd Sim2Real-AlgoBench
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
```

Run the full task:

```bash
ros2 launch race_navigation competition.launch.py headless:=false stress:=true
ros2 run race_control race_start_key          # press space or enter once
ros2 topic echo /race/state                   # observe the state machine
```

Run the planners alone, without the scenario. Every registered algorithm is loaded, and `planner_index` selects which one the terminal table reports as active:

```bash
ros2 launch algo_bringup algo_planner_bench.launch.py planner_index:=4
```

## Results

One baseline run in the nominal world, kept as a regression reference:

| Outcome | Time | First detection | Path | Collisions | Wall clearance |
| :---: | ---: | ---: | ---: | ---: | ---: |
| `COMPLETE` | 90.917 s | 63.033 s | 31.303 m | 0 | 0.3658 m |

This is a single run on the development machine — Ubuntu in a VM with software rendering — so the wall-clock times are a floor rather than a best case, and it is not a benchmark result. Each run is written to `reports/` as a JSON file and a readable summary. The recorded demo above is a separate run of the same stack (`COMPLETE`, 101.295 s, 27.059 m, 0 collisions).

## Layout

`src/algo_core` holds the algorithms and has no ROS dependency. `src/algo_nav2_plugins` adapts them to `nav2_core::GlobalPlanner`, and `src/algo_bringup` holds the index, the generated parameters and the behaviour trees.

The scenario around them: `race_description` (URDF, meshes, sensors, `ros2_control`), `race_gazebo` (competition map, nominal and dynamic-obstacle worlds), `race_bringup` (Gazebo, robot, controllers, bridge, RViz), `race_navigation` (SLAM, AMCL, Nav2, launch entry), `race_vision` (green A4 marker detection) and `race_control` (one-button start, state machine, mux, metrics).

## Docs

| Document | Contents |
| :--- | :--- |
| [`docs/ALGORITHM_PLUGINS.md`](docs/ALGORITHM_PLUGINS.md) | Adding an algorithm, the selection mechanism, simulation and vehicle use |
| [`docs/RECORDING.md`](docs/RECORDING.md) | The two-view recording setup, and how the camera axes were measured |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | What is next, and the open JPS discrepancy |
| [`tools/record_all_planners.sh`](tools/record_all_planners.sh) | Records the side-by-side clips, one run per global planner |
| [`tools/assemble_sidebyside.py`](tools/assemble_sidebyside.py) | Builds those clips into the media, and checks each one |
| [`tools/render_planning_demo.py`](tools/render_planning_demo.py) | Renders the search-shape figure from `algo_plan_dump` output |
| [`tools/make_gif.py`](tools/make_gif.py) | Converts a screen recording into a GIF |

`tools/assemble_sidebyside.py` does not trust the recordings: it measures the mean brightness of each half, so a half that never got written is reported rather than published, and it checks the car against its spawn pose, so two halves left at different angles are caught. Both of those shipped as silent defects once.

### Adding an algorithm

1. Write a subclass of `algo_core::GridPlanner`.
2. Register it with `ALGO_CORE_REGISTER`.
3. Add an entry to `algo_registry.yaml`.

The Nav2 plugin and the behaviour trees are generated from the registry, so no other file needs to change.

### Velocity output

A planner returns a path; it does not command the base. Velocity is published on `/cmd_vel_nav` or `/cmd_vel_final`, and `twist_priority_mux` selects one of the two to publish on `/cmd_vel`.

### On the vehicle

The plugins publish standard `geometry_msgs/Twist` including `linear.y`, so the same binary runs in Gazebo and on the vehicle. Two settings change: `min_y_velocity_threshold` in `controller_server` must allow lateral motion, and the costmap inflation radius needs re-tuning for the real LiDAR.

## Roadmap

[`docs/ROADMAP.md`](docs/ROADMAP.md).

## Acknowledgement

The simulation, the baseline and the original Chinese documentation were written by [zfyyyyy](https://github.com/zfyyyyy). The interchangeable planner library (`algo_core`, `algo_nav2_plugins`, `algo_bringup`), the recording and self-check tooling behind the comparison clips, and the English documentation were added on top of it.

This repository uses ROS 2, Nav2, SLAM Toolbox, Gazebo and OpenCV. If you use this work, please cite Nav2 ([Marathon 2, IROS 2020](https://arxiv.org/abs/2003.00368)), SLAM Toolbox ([JOSS 6(61):2783, 2021](https://joss.theoj.org/papers/10.21105/joss.02783)) and Theta\* ([JAIR 39:533–579, 2010](https://www.jair.org/index.php/jair/article/view/10676)).
