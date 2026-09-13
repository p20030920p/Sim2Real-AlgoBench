<div align="center">

# Sim2Real-AlgoBench

**A three-wheeled omnidirectional robot autonomy benchmark — one task, one interface contract, one metric set, scored in simulation and on hardware.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-8-F58113?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/license-MIT-3DA639)](LICENSE)
[![Algorithms](https://img.shields.io/badge/algorithms-7%20registered-brightgreen)](#algorithm-library)

[Task](#the-task) &nbsp;•&nbsp; [Algorithm library](#algorithm-library) &nbsp;•&nbsp; [Quick start](#quick-start) &nbsp;•&nbsp; [Results](#results) &nbsp;•&nbsp; [Docs](#docs)

*English &nbsp;|&nbsp; [中文](README.zh-CN.md)*

</div>

<p align="center">
  <img src="docs/images/07_finish_reached.png" width="820" alt="Autonomous search and finish-pad approach"/>
</p>

<p align="center">
  <em>Autonomous search of the saved map, green-marker detection, finish-pad approach. No goal pose is sent by hand.</em>
</p>

## The task

The robot gets a published start pose and exactly one start signal. It must search a known map for a green A4 wall marker, drive onto the yellow pad in front of it, and hold still for 3 s. The run ends on a **perceptual** condition, not a geometric one — which is what makes the simulation-to-hardware comparison meaningful.

## Algorithm library

Global planners are interchangeable Nav2 plugins behind one interface. **Switching algorithm is editing one number:**

```yaml
# src/algo_bringup/config/algo_registry.yaml
active:
  planner: 1        # >>> the only line to change <<<
```

Layered so the algorithms never touch ROS: `algo_core` (pure C++, unit-testable offline) → `algo_nav2_plugins` (one `nav2_core::GlobalPlanner` adapter) → `algo_bringup` (the index, parameters, behaviour trees).

Measured on the race map (294 × 294 @ 0.05 m), planning `(-3.0, -5.0)` → `(10.0, 7.0)`:

| Idx | Algorithm | Path | Poses | Note |
| :---: | :--- | :---: | ---: | :--- |
| 0 | Dijkstra | ✅ | 21 | optimal baseline |
| 1 | **A\*** | ✅ | 23 | default |
| 2 | Weighted A\* | ✅ | 29 | greedier, faster |
| 3 | GBFS | ✅ | 22 | fastest to a solution |
| 4 | JPS | ❌ | — | **known defect** |
| 5 | **Theta\*** | ✅ | **8** | any-angle |
| 6 | **D\* Lite** | ✅ | 19 | incremental replanning |
| 7 | Nav2 Theta\* | ✅ | 394 | incumbent baseline |

Theta\* covers the route in **8 poses against A\*'s 23** — any-angle planning removing the staircase, as a number.

All algorithms stay loaded, so switching costs no restart:

| Path | Mechanism |
| :--- | :--- |
| Per request | `ComputePathToPose` carries a `planner_id` field |
| Per situation | one behaviour tree per algorithm, selected via `behavior_tree` |
| Standalone | `algo_core::Registry::instance().create("theta_star")` |

> JPS reports `NO_VALID_PATH` on the race map although A\* finds a route under the same cost model, so its pruning is wrong. It is flagged in the registry and **must not be used for reported results**.

## Demonstration

<p align="center">
  <img src="docs/media/dynamic_obstacle.gif" width="720" alt="Stress-world run: Gazebo on the left, the replanned path in RViz on the right"/>
</p>

<p align="center">
  <em>Stress world. Left: Gazebo, with <code>dynamic_obstacle_1/2</code> in the entity tree. Right: the path Nav2 replans as they move.</em><br/>
  <sub>GIF is a 640 px / 8 fps preview — <a href="docs/media/dynamic_obstacle.mp4">download the original (1920 × 1080, 60 fps)</a>.</sub>
</p>

## Gallery

| SLAM mapping | Map saved |
| :---: | :---: |
| ![SLAM mapping](docs/images/01_mapping.png) | ![Saved PGM map](docs/images/02_map_saved.png) |
| **Stress world** | **TF tree** |
| ![Moving obstacles and varying illumination](docs/images/05_stress_world.png) | ![TF tree](docs/images/06_tf_tree.png) |

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

Benchmark the algorithms alone, without the scenario:

```bash
ros2 launch algo_bringup algo_planner_bench.launch.py planner_index:=4
```

## Results

One nominal-world baseline run, kept as a regression reference:

| Outcome | Time | First detection | Path | Collisions | Wall clearance |
| :---: | ---: | ---: | ---: | ---: | ---: |
| `COMPLETE` | 90.917 s | 63.033 s | 31.303 m | 0 | 0.3658 m |

A single run on the machine of the time — a regression baseline, not a reportable result. Per-run records land in `reports/`.

## Layout

```text
src/
├── algo_core/            # algorithms, no ROS dependency
├── algo_nav2_plugins/    # Nav2 plugin adapters
├── algo_bringup/         # algorithm index, parameters, behaviour trees
├── race_description/     # URDF, meshes, sensors, ros2_control
├── race_gazebo/          # competition map, nominal & stress worlds
├── race_bringup/         # Gazebo, robot, controllers, bridge, RViz
├── race_navigation/      # SLAM, AMCL, Nav2 configuration, launch entry
├── race_vision/          # green A4 marker detection
└── race_control/         # one-button start, state machine, mux, metrics
```

## Docs

| Document | Contents |
| :--- | :--- |
| [`docs/ALGORITHM_PLUGINS.md`](docs/ALGORITHM_PLUGINS.md) | The algorithm library: adding an algorithm in three steps, selection, simulation and hardware use |
| [`tools/make_gif.py`](tools/make_gif.py) | Regenerates the demo GIF from the source recording |

**Adding an algorithm** — implement `algo_core::GridPlanner`, self-register with `ALGO_CORE_REGISTER`, add one entry to `algo_registry.yaml`. The plugin class and behaviour trees are generated; nothing else changes. Contributions must not publish to `/cmd_vel` directly — write to `/cmd_vel_nav` or `/cmd_vel_final` and let the mux arbitrate.

**On hardware** the plugins need no changes: they emit standard `geometry_msgs/Twist` including `linear.y`, so the existing `/cmd_vel` → `TwistStamped` → `omni_drive_controller` chain is untouched. Enable lateral velocity in `controller_server`, and re-tune the costmap inflation radius for real sensor noise.

## Roadmap

- [x] Baseline: SLAM Toolbox, AMCL, Theta\*, MPPI Omni, HSV marker detection
- [x] Interchangeable planners: Dijkstra, A\*, Weighted A\*, GBFS, Theta\*, D\* Lite
- [ ] JPS — implemented, **broken on the race map**
- [ ] Sampling planners: RRT, RRT\*, Informed RRT\*, RRT-Connect
- [ ] Controller adapters: DWA, APF, RPP, LQR, MPC
- [ ] Hardware bring-up and calibration record

## Acknowledgement

The simulation, baseline and original Chinese documentation were developed by [zfyyyyy](https://github.com/zfyyyyy). Built on ROS 2, Nav2, SLAM Toolbox, Gazebo and OpenCV.

If you use this work, please cite Nav2 ([Marathon 2, IROS 2020](https://arxiv.org/abs/2003.00368)), SLAM Toolbox ([JOSS 6(61):2783, 2021](https://joss.theoj.org/papers/10.21105/joss.02783)) and Theta\* ([JAIR 39:533–579, 2010](https://www.jair.org/index.php/jair/article/view/10676)).

## License

[MIT](LICENSE).
