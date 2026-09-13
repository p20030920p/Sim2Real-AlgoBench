<div align="center">

# Sim2Real-AlgoBench

**三轮全向机器人自主导航基准 —— 一个任务、一套接口契约、一组指标，在仿真与实物上各评测一次。**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-8-F58113?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/license-MIT-3DA639)](LICENSE)
[![Algorithms](https://img.shields.io/badge/algorithms-7%20registered-brightgreen)](#算法插件库)

[任务定义](#任务定义) &nbsp;•&nbsp; [算法插件库](#算法插件库) &nbsp;•&nbsp; [快速开始](#快速开始) &nbsp;•&nbsp; [运行结果](#运行结果) &nbsp;•&nbsp; [文档](#文档)

*[English](README.md) &nbsp;|&nbsp; 中文*

</div>

<p align="center">
  <img src="docs/images/07_finish_reached.png" width="820" alt="已有地图自主导航与终点搜索"/>
</p>

<p align="center">
  <em>自主搜索已保存地图、识别绿色标志、驶入终点区域。全程未手工发送目标点。</em>
</p>

## 任务定义

机器人获得一个公布的起点和恰好一次启动信号。它必须搜索已知地图找到墙面的绿色 A4 标志，驶入其前方的黄色区域，并静止 3 秒。任务以**感知**条件终止而非几何条件——这正是仿真到实物比较具有意义的原因。

## 算法插件库

全局规划器是同一接口背后的可互换 Nav2 插件。**换算法只需要改一个数字：**

```yaml
# src/algo_bringup/config/algo_registry.yaml
active:
  planner: 1        # >>> 唯一需要改的一行 <<<
```

分层设计使算法完全不接触 ROS：`algo_core`（纯 C++，可离线单测）→ `algo_nav2_plugins`（一个 `nav2_core::GlobalPlanner` 适配器）→ `algo_bringup`（索引、参数、行为树）。

在 race 地图（294 × 294 @ 0.05 m）上从 `(-3.0, -5.0)` 规划到 `(10.0, 7.0)` 的实测结果：

| 索引 | 算法 | 结果 | 点数 | 说明 |
| :---: | :--- | :---: | ---: | :--- |
| 0 | Dijkstra | ✅ | 21 | 最优基线 |
| 1 | **A\*** | ✅ | 23 | 默认 |
| 2 | Weighted A\* | ✅ | 29 | 更贪心，更快 |
| 3 | GBFS | ✅ | 22 | 最快得到解 |
| 4 | JPS | ❌ | — | **已知缺陷** |
| 5 | **Theta\*** | ✅ | **8** | 任意角 |
| 6 | **D\* Lite** | ✅ | 19 | 增量重规划 |
| 7 | Nav2 Theta\* | ✅ | 394 | 对照基线 |

Theta\* 只用 **8 个点**走完全程，A\* 需要 23 个——任意角规划消除栅格阶梯的效果，是个数字。

所有算法同时常驻，切换不需重启：

| 路径 | 机制 |
| :--- | :--- |
| 按请求 | `ComputePathToPose` 自带 `planner_id` 字段 |
| 按情况 | 每个算法一棵行为树，通过 `behavior_tree` 选择 |
| 脱离 Nav2 | `algo_core::Registry::instance().create("theta_star")` |

> JPS 在 race 地图上返回 `NO_VALID_PATH`，而同样代价模型的 A\* 能找到路径，说明其剪枝逻辑有错。注册表中已标注，**修好前不要用它出结果**。

## 演示

<p align="center">
  <img src="docs/media/dynamic_obstacle.gif" width="720" alt="压力世界：左侧 Gazebo，右侧 RViz 中重新规划的路径"/>
</p>

<p align="center">
  <em>压力世界。左：Gazebo，实体树中可见 <code>dynamic_obstacle_1/2</code>。右：障碍物移动时 Nav2 重新规划的路径。</em><br/>
  <sub>GIF 为 640 px / 8 fps 预览 —— <a href="docs/media/dynamic_obstacle.mp4">下载原片（1920 × 1080、60 fps）</a>。</sub>
</p>

## 运行截图

| SLAM 建图 | 地图保存 |
| :---: | :---: |
| ![SLAM 建图过程](docs/images/01_mapping.png) | ![保存的 PGM 栅格地图](docs/images/02_map_saved.png) |
| **压力世界** | **TF 坐标树** |
| ![动态障碍与变化地面光照测试](docs/images/05_stress_world.png) | ![机器人 TF 坐标树](docs/images/06_tf_tree.png) |

## 快速开始

```bash
git clone https://github.com/p20030920p/Sim2Real-AlgoBench.git
cd Sim2Real-AlgoBench
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
```

运行完整任务：

```bash
ros2 launch race_navigation competition.launch.py headless:=false stress:=true
ros2 run race_control race_start_key          # 按一次空格或回车
ros2 topic echo /race/state                   # 观察状态机
```

不起整个场景，单独横评算法：

```bash
ros2 launch algo_bringup algo_planner_bench.launch.py planner_index:=4
```

## 运行结果

普通世界的一次基线运行，保留作为回归参照：

| 状态 | 完成时间 | 首次识别 | 路径长度 | 碰撞 | 末端墙距 |
| :---: | ---: | ---: | ---: | ---: | ---: |
| `COMPLETE` | 90.917 s | 63.033 s | 31.303 m | 0 | 0.3658 m |

这是当时机器上的**单次**运行，属于回归基线而非可报告的基准结果。每次运行的记录写入 `reports/`。

## 目录结构

```text
src/
├── algo_core/            # 算法本体，无 ROS 依赖
├── algo_nav2_plugins/    # Nav2 插件适配层
├── algo_bringup/         # 算法索引、参数、行为树
├── race_description/     # URDF、网格、传感器、ros2_control
├── race_gazebo/          # 比赛地图、普通/压力世界
├── race_bringup/         # Gazebo、机器人、控制器、桥接、RViz
├── race_navigation/      # SLAM、AMCL、Nav2 配置、启动入口
├── race_vision/          # 绿色 A4 标志识别
└── race_control/         # 一键启动、状态机、速度仲裁、计时报表
```

## 文档

| 文档 | 内容 |
| :--- | :--- |
| [`docs/ALGORITHM_PLUGINS.md`](docs/ALGORITHM_PLUGINS.md) | 算法插件库：三步接入新算法、选择机制、仿真与实车用法 |
| [`tools/make_gif.py`](tools/make_gif.py) | 从原始录像重新生成演示 GIF |

**接入新算法**——实现 `algo_core::GridPlanner`、用 `ALGO_CORE_REGISTER` 自注册、在 `algo_registry.yaml` 加一条。插件类和行为树会自动生成，不需要改别的。贡献算法**不得**直接向 `/cmd_vel` 发布，写入 `/cmd_vel_nav` 或 `/cmd_vel_final`，由仲裁器裁决。

**上实车**插件无需任何改动：它们输出标准的 `geometry_msgs/Twist`（含 `linear.y`），现有 `/cmd_vel` → `TwistStamped` → `omni_drive_controller` 链路完全不用碰。需在 `controller_server` 中放开横向速度，并针对真实传感器噪声重新标定代价地图膨胀半径。

## 路线图

- [x] 基线：SLAM Toolbox、AMCL、Theta\*、MPPI Omni、HSV 标志检测
- [x] 可互换规划器：Dijkstra、A\*、Weighted A\*、GBFS、Theta\*、D\* Lite
- [ ] JPS —— 已实现，但在 race 地图上**失败**
- [ ] 采样类规划器：RRT、RRT\*、Informed RRT\*、RRT-Connect
- [ ] 控制器适配层：DWA、APF、RPP、LQR、MPC
- [ ] 实车 bring-up 与标定记录

## 致谢

仿真、基线与原始中文文档由 [zfyyyyy](https://github.com/zfyyyyy) 完成。本项目建立在 ROS 2、Nav2、SLAM Toolbox、Gazebo 与 OpenCV 之上。

若在研究中引用，请引用 Nav2（[Marathon 2, IROS 2020](https://arxiv.org/abs/2003.00368)）、SLAM Toolbox（[JOSS 6(61):2783, 2021](https://joss.theoj.org/papers/10.21105/joss.02783)）与 Theta\*（[JAIR 39:533–579, 2010](https://www.jair.org/index.php/jair/article/view/10676)）。

## 许可证

[MIT](LICENSE)。
