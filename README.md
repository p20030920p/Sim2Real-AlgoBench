<div align="center">

# Sim2Real-AlgoBench

**A three-wheeled omnidirectional robot autonomy benchmark — one task, one interface contract, seven interchangeable global planners.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-8-F58113?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/license-MIT-3DA639)](LICENSE)
[![Algorithms](https://img.shields.io/badge/algorithms-7%20registered-brightgreen)](#algorithm-library)

[Task](#the-task) &nbsp;•&nbsp; [Algorithm library](#algorithm-library) &nbsp;•&nbsp; [Demos](#demos) &nbsp;•&nbsp; [Quick start](#quick-start)

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

## Algorithm library

A planner is a subclass plus one registry entry: the Nav2 plugin, its parameters and one behaviour tree per algorithm are generated from that entry, so swapping the plugin changes the algorithm and nothing else.

```yaml
# src/algo_bringup/config/algo_registry.yaml
active:
  planner: 1
```

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

A\* expands 26,442 cells; D\* Lite expands 375, because it keeps its search between calls and only repairs what changed. Theta\* returns 8 poses where A\* returns 23; the difference is the grid staircase that any-angle planning removes.

All registered planners stay loaded at once, and any of them can be selected per request:

| Selection | Mechanism |
| :--- | :--- |
| Per request | `ComputePathToPose` carries a `planner_id` field |
| Per situation | one behaviour tree per algorithm, chosen through `behavior_tree` |
| Outside Nav2 | `algo_core::Registry::instance().create("theta_star")` |

JPS is registered but does not work. It returns `NO_VALID_PATH` on the race map where A\* finds a route with the same cost model, so its pruning is incorrect. Do not use it for reported results.

## Demos

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

The clips are MP4 at `docs/media/run_sidebyside/<algorithm>.mp4`. To reproduce
one run directly:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
ros2 launch race_navigation competition.launch.py headless:=true stress:=false
python3 tools/send_start_signal.py        # the one start signal
ros2 topic echo /race/state               # watch it finish
```

### Search shape, and how to read the colours

`docs/media/search_2d.gif` animates the order in which each planner expanded cells, from `algo_plan_dump`'s record of the real searches. The colour is expansion order, normalised per panel, so the same colour in two panels is the same fraction of two different searches; the cell count printed under each panel is the comparison.

![Six planner searches side by side](docs/media/search_2d.gif)

<p align="center">
  <sub><a href="docs/media/search_2d.mp4">Download (MP4)</a> &nbsp;·&nbsp; rendered with <code>tools/render_planning_demo.py</code></sub>
</p>

### Dynamic-obstacle world

The same task with two moving obstacles sweeping the corridors. `stress:=true` selects it, loading `competition_stress.world` instead of `competition_world.world`. The planner comparison above passes `stress:=false`, so all seven planners face the same arena and the same costmap. The recording of a moving-obstacle run is kept here as a download:

<p align="center">
  <sub><a href="docs/media/dynamic_obstacle.mp4">dynamic_obstacle.mp4 (1920 × 1080 at 60 fps)</a></sub>
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
