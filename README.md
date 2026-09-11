# Sim2Real-AlgoBench

**A three-wheeled omnidirectional robot autonomy benchmark built on ROS 2 Jazzy and Gazebo Sim 8.** One fixed task, one fixed interface contract, one fixed metric set — evaluated twice: in simulation and on a physical robot.

<p align='center'>
    <img src="docs/images/07_finish_reached.png" alt="Autonomous search and finish-pad approach" width="800"/>
</p>

<p align='center'>
    <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"/></a>
    <img src="https://img.shields.io/badge/ROS%202-Jazzy-blue.svg" alt="ROS 2 Jazzy"/>
    <img src="https://img.shields.io/badge/Ubuntu-24.04-orange.svg" alt="Ubuntu 24.04"/>
    <img src="https://img.shields.io/badge/Gazebo%20Sim-8-lightgrey.svg" alt="Gazebo Sim 8"/>
    <img src="https://img.shields.io/badge/status-active-brightgreen.svg" alt="Status: active"/>
</p>

**中文文档：[README.zh-CN.md](README.zh-CN.md)**

---

## Menu

- [**The task**](#the-task)
- [**Features**](#features)
- [**System overview**](#system-overview)
- [**Package layout**](#package-layout)
- [**Dependency**](#dependency)
- [**Build**](#build)
- [**Quick start**](#quick-start)
- [**Re-mapping**](#re-mapping)
- [**Topics and actions**](#topics-and-actions)
- [**TF tree**](#tf-tree)
- [**Parameters**](#parameters)
- [**Results**](#results)
- [**Roadmap**](#roadmap)
- [**Adding a new algorithm**](#adding-a-new-algorithm) (must read)
- [**Troubleshooting**](#troubleshooting)
- [**Before running on a physical robot**](#before-running-on-a-physical-robot)
- [**Acknowledgement**](#acknowledgement)
- [**License**](#license)

## The task

The robot is placed at a published start pose on a known map and receives exactly one start signal. It must then **autonomously search** the map for a **green A4 marker** mounted on a wall, drive into the **yellow finish pad** in front of that marker, and stay still for **3 seconds**.

No goal pose is published to Nav2 by hand, and no finish coordinate is hard-coded. Detection is what ends the run, not geometry — which is what makes the simulation-to-hardware comparison meaningful, because perception is where the two domains differ most.

## Features

- Colored SolidWorks competition map converted to Gazebo DAE/SDF models;
- Three-wheeled omnidirectional chassis: URDF/Xacro, wheel joints, LiDAR, camera and `ros2_control`;
- SLAM Toolbox mapping and Nav2 Map Saver for map persistence;
- Prior grid map, published start pose, AMCL online localization;
- Safe search viewpoints generated from the free-space connected component of the start pose;
- Theta* any-angle global planning;
- MPPI `Omni` local control with independent longitudinal, lateral and rotational motion;
- Real-time LiDAR costmap updates handling both static and moving obstacles;
- OpenCV green A4 marker detection;
- On detection, Nav2 is cancelled automatically and the node switches to visual centring, LiDAR ranging and stop;
- `/cmd_vel_nav` and `/cmd_vel_final` arbitration, guaranteeing a **single publisher** on `/cmd_vel`;
- One-button start with space or enter;
- Automatic reporting of task time, first-detection time, path length, collision count and finish offset;
- A nominal world and a stress world with moving obstacles, low-traction regions, rough ground and varying illumination.

## System overview

<p align='center'>
    <img src="docs/images/08_autonomy_graph.png" alt="map_search_autonomy node graph" width="800"/>
</p>

```text
saved map + LiDAR + wheel odometry
             │
             ▼
       AMCL (map → odom)
             │
             ▼
 free-space connected-component viewpoint generation
             │ NavigateToPose action
             ▼
   Theta* any-angle global planning
             │
             ▼
     MPPI (Omni) local control
             │
             ▼
 velocity_smoother / collision_monitor
             │
             ▼
       /cmd_vel_nav ───────────┐
                               │
 camera → green marker detector │
             │                 │
             ▼                 ▼
 /target/green_board_observation   twist_priority_mux
             │                 ▲
             ▼                 │
   visual centring & approach  │
             │                 │
      /cmd_vel_final ──────────┘
                               │
                               ▼
                           /cmd_vel
                               │
                               ▼
                  Twist → TwistStamped
                               │
                               ▼
                  omni_drive_controller
                               │
                               ▼
                     three wheel joints
```

## Package layout

```text
Sim2Real-AlgoBench/
├── maps/                         # saved race_map.yaml / race_map.pgm
├── reports/                      # auto-generated run reports
├── diagnostics/                  # TF-tree and other diagnostic artefacts
├── docs/                         # figures and media
└── src/
    ├── race_description/         # URDF, meshes, sensors, ros2_control
    ├── race_gazebo/              # map models, nominal & stress worlds, moving obstacles
    ├── race_bringup/             # Gazebo, robot, controllers, bridge, RViz assembly
    ├── race_navigation/          # SLAM, AMCL, Nav2 configuration and the launch entry
    ├── race_vision/              # green A4 finish marker detection
    └── race_control/             # one-button start, autonomy state machine, mux, metrics
```

| Package | Role |
| :--- | :--- |
| `race_description` | `base_footprint`, `base_link`, three wheels, `lidar_link`, `camera_link`, collision bodies and the Gazebo `ros2_control` hardware interface. |
| `race_gazebo` | Colored competition scene, collision models, nominal world, stress world, low-traction / rough ground, varying illumination, two moving obstacles. |
| `race_bringup` | Starts Gazebo, spawns the robot, loads `joint_state_broadcaster` and `omni_drive_controller`, publishes robot TF, bridges sensors, converts `Twist` to `TwistStamped`. |
| `race_navigation` | SLAM Toolbox, Map Saver, AMCL, Nav2, Theta*, MPPI Omni, costmaps, velocity smoothing, collision monitoring and `competition.launch.py`. |
| `race_vision` | Detects the green A4 marker using HSV, green excess, CLAHE, morphology and a consecutive-frame test; publishes validity, normalized horizontal offset and area ratio. |
| `race_control` | `map_search_autonomy.py`, `twist_priority_mux.py`, `twist_to_twist_stamped.py`, `race_start_key.py`, `race_metrics.py`, `moving_obstacle.py`. |

## Dependency

| Component | Version / role |
| :--- | :--- |
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| Simulator | Gazebo Sim 8 (Harmonic series) |
| Build | colcon, CMake, ament |
| Navigation | Nav2, AMCL, Theta*, MPPI Omni |
| Mapping | SLAM Toolbox |
| Control | `ros2_control`, omnidirectional drive controller |
| Vision | OpenCV, `cv_bridge`, `image_transport` |
| Models | URDF/Xacro, STL, DAE, SDF |

## Build

```bash
git clone https://github.com/p20030920p/Sim2Real-AlgoBench.git
cd Sim2Real-AlgoBench

source /opt/ros/jazzy/setup.bash

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

Add the workspace to every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Sim2Real-AlgoBench/install/setup.bash
```

## Quick start

### 1. Launch simulation, localization, navigation, vision and reporting

```bash
ros2 launch race_navigation competition.launch.py \
  headless:=false \
  nav_rviz:=true \
  stress:=true
```

| Argument | Effect |
| :--- | :--- |
| `headless:=false` | Shows Gazebo. Keep `false` for the perception task. |
| `nav_rviz:=true` | Opens the Nav2 RViz view for observation only. |
| `stress:=true` | Enables moving obstacles, low-traction / rough ground and varying illumination. Use `stress:=false` for functional verification. |

Wait until you see:

```text
Saved-map search autonomy ready; waiting for one-button start.
Managed nodes are active
```

### 2. One-button start

In a second terminal:

```bash
ros2 run race_control race_start_key
```

Press **space or enter exactly once**. The key node publishes `/race/start=true` and exits normally; the autonomy task keeps running.

For scripted runs:

```bash
ros2 topic pub --once /race/start std_msgs/msg/Bool "{data: true}"
```

### 3. Watch the state machine

```bash
ros2 topic echo /race/state \
  --qos-reliability reliable \
  --qos-durability transient_local
```

```text
WAITING_FOR_ONE_BUTTON_START
PREPARING_MAP_SEARCH
NAVIGATING_TO_SEARCH_VIEWPOINT
SCANNING_360_FOR_GREEN_BOARD
ALIGNING_WITH_GREEN_BOARD
APPROACHING_YELLOW_FINISH_PAD
HOLDING_STILL_FOR_3_SECONDS
COMPLETE
SEARCH_EXHAUSTED
```

### 4. Reset

```bash
ros2 topic pub --once /race/reset std_msgs/msg/Bool "{data: true}"
```

This cancels the active goal, publishes zero velocity and returns to the waiting state. To restore the robot to its physical start pose, exit and relaunch the whole stack.

### Gallery

| SLAM mapping | Map saved |
| :---: | :---: |
| ![SLAM mapping](docs/images/01_mapping.png) | ![Saved PGM map](docs/images/02_map_saved.png) |

| Nav2 waypoint debug | One-button autonomous search |
| :---: | :---: |
| ![Nav2 waypoint navigation](docs/images/03_nav2_navigation.png) | ![Autonomous search](docs/images/04_autonomy_simulation.png) |

| Stress world | TF tree |
| :---: | :---: |
| ![Moving obstacles and varying illumination](docs/images/05_stress_world.png) | ![TF tree](docs/images/06_tf_tree.png) |

[▶ Watch the moving-obstacle avoidance video](docs/media/dynamic_obstacle.mp4)

## Re-mapping

When the fixed obstacle layout changes, start the mapping chain:

```bash
ros2 launch race_navigation mapping.launch.py \
  headless:=false rviz:=true
```

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Sim2Real-AlgoBench/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Once the map is complete, save it:

```bash
mkdir -p ~/Sim2Real-AlgoBench/maps

ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
"{map_topic: map, map_url: $HOME/Sim2Real-AlgoBench/maps/race_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

This produces `maps/race_map.pgm` and `maps/race_map.yaml`.

## Topics and actions

| Interface | Type / role |
| :--- | :--- |
| `/race/start` | `std_msgs/Bool` — one-button start |
| `/race/reset` | `std_msgs/Bool` — task reset |
| `/race/state` | `std_msgs/String` — state-machine state |
| `/race/complete` | `std_msgs/Bool` — completion flag |
| `/race/direct_control` | velocity-arbitration selection signal |
| `/map` | `nav_msgs/OccupancyGrid` — static map |
| `/scan` | `sensor_msgs/LaserScan` |
| `/amcl_pose` | AMCL pose estimate |
| `/camera/image_raw` | raw camera image |
| `/target/green_board_observation` | green marker detection result |
| `/navigate_to_pose` | Nav2 navigation action |
| `/cmd_vel_nav` | Nav2 velocity command |
| `/cmd_vel_final` | visual-servoing velocity command |
| `/cmd_vel` | arbitrated chassis command — **single publisher** |
| `/omni_drive_controller/cmd_vel` | `TwistStamped` controller input |
| `/omni_drive_controller/odom` | omnidirectional controller odometry |
| `/tf`, `/tf_static` | dynamic and static transforms |

## TF tree

<p align='center'>
    <img src="docs/images/06_tf_tree.png" alt="Robot TF tree" width="600"/>
</p>

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── lidar_link
            ├── camera_link
            ├── front_wheel_link
            ├── left_wheel_link
            └── right_wheel_link
```

AMCL publishes `map → odom`; the omnidirectional controller publishes `odom → base_footprint`; `robot_state_publisher` publishes the body transforms from the URDF and `/joint_states`.

## Parameters

| Parameter | Value |
| :--- | :--- |
| MPPI Omni | `vx_max = 0.90 m/s`, `vy_max = 0.65 m/s`, `wz_max = 1.30 rad/s` |
| Terminal visual approach | up to ≈0.46 m/s |
| Search viewpoints | ≈1.0 m spacing, 0.65 m obstacle clearance, ≤8 viewpoints |
| Search time limit | ≈285 s, then `SEARCH_EXHAUSTED` |
| Terminal barrier distance | ≈0.34 m |
| Completion criterion | finish conditions satisfied and 3 s stationary |

## Results

One complete nominal-world run of the baseline, kept as a regression reference:

| Metric | Value |
| :--- | :--- |
| Outcome | `COMPLETE` |
| Completion time | 90.917 s |
| First marker detection | 63.033 s |
| Path length | 31.303 m |
| Collisions | 0 |
| Terminal wall clearance | 0.3658 m |
| Marker horizontal error | −0.0134 |

> This is a **single run** under the simulation parameters and machine of the time. It is a regression baseline, not a reportable benchmark result. Per-run records are written to `reports/` as `*.json` plus a readable `*.md`, with `reports/race_summary.csv` across runs.

## Roadmap

Algorithm slots, grouped by what they replace. `[x]` = baseline in place, `[ ]` = open.

**Baseline**

- [x] Mapping — SLAM Toolbox
- [x] Localization — AMCL
- [x] Global planning — Theta* (`nav2_theta_star_planner`)
- [x] Local control — MPPI, `Omni` motion model
- [x] Perception — HSV + green-excess threshold detector

**Simulation track**

- [ ] Mapping / SLAM — `<algorithm>`
- [ ] Localization — `<algorithm>`
- [ ] Global planning — `<algorithm>`
- [ ] Local control — `<algorithm>`
- [ ] Perception — `<algorithm>`
- [ ] Mission logic — `<algorithm>`

**Physical track**

- [ ] Hardware bring-up and calibration record
- [ ] Mapping / SLAM — `<algorithm>`
- [ ] Localization — `<algorithm>`
- [ ] Global planning — `<algorithm>`
- [ ] Local control — `<algorithm>`
- [ ] Perception — `<algorithm>`

## Adding a new algorithm

**Read this before opening a pull request.** The point of this repository is that an algorithm can be swapped in *one* category while every other category keeps its baseline implementation, so results stay comparable.

1. **Pick one category** — mapping, localization, global planning, local control, perception or mission logic.
2. **Satisfy that category's interface** (see [Topics and actions](#topics-and-actions)). Add a package under `src/`, or a new workspace if the algorithm does not belong to this stack at all.
3. **Never publish to `/cmd_vel` directly.** Write to `/cmd_vel_nav` or `/cmd_vel_final`; the mux arbitrates. One publisher on `/cmd_vel` is a contract, not a detail.
4. **Do not modify the baseline packages** to make a contribution work. If that seems necessary, the interface is wrong — please open an issue instead.
5. **Report the same metrics** as the baseline (`race_metrics`), and state any deviation from the protocol.
6. **Update the [Roadmap](#roadmap)** and open a pull request naming the category you replaced.

Metrics reported for every run, in either domain: completion time, time to first detection, path length, collision count, terminal speed, terminal wall clearance, marker horizontal error, and outcome.

## Troubleshooting

**`/cmd_vel` publishes at a normal rate but the robot does not move.**
`ros2 topic hz` only proves messages are flowing. Check the values and the controller state:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /omni_drive_controller/cmd_vel
ros2 control list_controllers
```

**RViz reports TF or message-queue errors.**
Check that every node is on simulation time, and set RViz `Fixed Frame` to `map`:

```bash
ros2 topic hz /clock
ros2 run tf2_ros tf2_echo map base_footprint
```

**Nav2 keeps recovering in front of a moving obstacle.**
In the stress world a moving obstacle can transiently block a narrow passage. The autonomy node re-targets and starts another search round; it only enters `SEARCH_EXHAUSTED` near the 285 s limit.

**The green marker is not detected.**

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /target/green_board_observation
```

Vision thresholds must be re-calibrated for exposure, white balance, ambient light and actual distance on a physical robot.

**Useful diagnostics**

```bash
ros2 node list
ros2 node info /map_search_autonomy
ros2 topic list -t
ros2 topic info /cmd_vel -v
ros2 action info /navigate_to_pose
ros2 control list_hardware_interfaces
rqt_graph                       # use "Nodes/Topics (active)"

mkdir -p diagnostics && cd diagnostics
ros2 run tf2_tools view_frames
xdg-open "$(ls -t frames*.pdf | head -n 1)"

ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_link lidar_link
```

`omni_drive_controller` and `joint_state_broadcaster` should both be `active`.

## Before running on a physical robot

A passing simulation does **not** license high-speed hardware operation. The first physical runs must be done at reduced speed, in an open area, with an immediate emergency stop available, and every item below calibrated step by step:

- published start pose `x / y / yaw`;
- actual wheel radius, wheel mounting angles, wheel rotation directions and maximum stable speed;
- LiDAR and camera extrinsic offsets from `base_link`;
- camera exposure, white balance and the green threshold under venue illumination;
- robot footprint, finish-pad coverage ratio and barrier stopping distance;
- chassis hardware interface, serial/CAN and emergency stop;
- collision geometry, floor friction, braking distance and speed cap.

## Acknowledgement

The simulation, the baseline implementation and the original Chinese documentation were developed by [zfyyyyy](https://github.com/zfyyyyy). This repository builds on the ROS 2, Nav2, SLAM Toolbox, Gazebo and OpenCV ecosystems.

If you use this work, please cite the underlying packages:

- S. Macenski, F. Martín, R. White, J. Clavero. *The Marathon 2: A Navigation System.* IROS, 2020.
- S. Macenski, I. Jambrecic. *SLAM Toolbox: SLAM for the dynamic world.* JOSS, 6(61): 2783, 2021.
- K. Daniel, A. Nash, S. Koenig, A. Felner. *Theta-star: Any-Angle Path Planning on Grids.* JAIR, 39: 533–579, 2010.
- G. Williams, N. Wagener, B. Goldfain, P. Drews, J. M. Rehg, B. Boots, E. A. Theodorou. *Information Theoretic MPC for Model-Based Reinforcement Learning.* ICRA, 2017.

## License

[MIT](LICENSE).
