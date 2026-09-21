<div align="center">

# Sim2Real-AlgoBench

**三轮全向机器人自主导航基准。一个任务、一套接口契约、一组指标，七种可互换的全局规划器。**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-8-F58113?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/license-MIT-3DA639)](LICENSE)
[![Algorithms](https://img.shields.io/badge/algorithms-7%20registered-brightgreen)](#算法插件库)

[任务定义](#任务定义) &nbsp;•&nbsp; [算法插件库](#算法插件库) &nbsp;•&nbsp; [演示](#演示) &nbsp;•&nbsp; [快速开始](#快速开始) &nbsp;•&nbsp; [文档](#文档)

*[English](README.md) &nbsp;|&nbsp; 中文*

</div>

<p align="center">
  <img src="docs/images/07_finish_reached.png" width="820" alt="自主搜索与终点区域靠近"/>
</p>

<p align="center">
  <em>自主搜索已保存地图、识别绿色标志、驶入终点区域。全程未手工发送目标点。</em>
</p>

## 任务定义

机器人获得一个起点和一次启动信号。它需要在已知地图中搜索墙面的绿色 A4 标志，驶入标志前方的黄色区域，并静止 3 秒。

任务在识别到标志时结束，而不是到达某个坐标时结束。感知因此在关键路径上，这也是这个对比值得做的原因。

## 项目来源

任务、场地与评分规则来自一项综合性机器人比赛。同一套栈在比赛中于实车上跑通并完赛。

本仓库保存的是这套栈的仿真侧：场地、任务状态机、指标与规划器横评，全部在 Gazebo 中端到端运行。实车的 bring-up 与标定记录未在此发布。

## 为什么把规划器拆出来

新想法通常落在全局规划器上，而比较两个规划器，通常意味着改动它们周围的整套栈。这里一个规划器就是一个子类加一条注册表条目：Nav2 插件、参数和每个算法一棵行为树都由这条条目生成，所以一次对比只改算法本身。

每次运行都会写出 JSON 报告，两个规划器因此是在同一组指标上比较，而不是在两份口头描述上比较。

## 算法插件库

七种全局规划器都是可互换的 Nav2 插件。当前使用哪一个由一行配置决定：

```yaml
# src/algo_bringup/config/algo_registry.yaml
active:
  planner: 1
```

算法本体在 `algo_core` 中，不依赖 ROS，可以单独做单元测试。`algo_nav2_plugins` 用一个 `nav2_core::GlobalPlanner` 适配器把它们接入 Nav2。`algo_bringup` 存放索引、参数和行为树。

单次规划调用、仅用 CPU，在开发用轻薄本（华为 MateBook 14，Intel Core i5-1240P）上，于 race 地图
（294 × 294，分辨率 0.05 m）从 `(-3.0, -5.0)` 规划到 `(10.0, 7.0)` 的实测结果。可跨机器比较的是展开格数，
毫秒数仅供参考。

| 索引 | 算法 | 找到路径 | 路径点数 | 展开格数 | 耗时 |
| :---: | :--- | :---: | ---: | ---: | ---: |
| 0 | Dijkstra | 是 | 21 | 55,578 | 5.0 ms |
| 1 | **A\*** | 是 | 23 | 26,442 | 5.3 ms |
| 2 | Weighted A\* | 是 | 29 | 9,025 | 1.4 ms |
| 3 | GBFS | 是 | 22 | 3,127 | 0.6 ms |
| 4 | JPS | **否** | — | 5 | 0.1 ms |
| 5 | **Theta\*** | 是 | **8** | 20,694 | 30.5 ms |
| 6 | **D\* Lite** | 是 | 19 | 375 | 38.7 ms |
| 7 | Nav2 Theta\* | 是 | 394 | — | — |

A\* 展开 26,442 格，D\* Lite 只展开 375 格，因为后者在多次调用之间保留搜索状态，只修复发生变化的部分。Theta\* 返回 8 个路径点，A\* 返回 23 个，差别来自任意角规划消除的栅格阶梯。JPS 已注册，但在这张地图上规划不出路径；它的剪枝现状见 [`docs/ROADMAP.md`](docs/ROADMAP.md)。

所有规划器同时保持加载，可以在每次请求时单独选择：

| 选择方式 | 机制 |
| :--- | :--- |
| 按请求 | `ComputePathToPose` 带有 `planner_id` 字段 |
| 按情况 | 每个算法一棵行为树，通过 `behavior_tree` 选择 |
| 脱离 Nav2 | `algo_core::Registry::instance().create("theta_star")` |

## 演示

### 完整赛题一次跑通：左边 Gazebo，右边 RViz

从**单个启动信号**开始的一次完整运行，到静止 3 秒结束。左半是仿真器自己的俯视相机，**不带任何叠加**；右半是 RViz 画的同一时刻——已有地图、全局代价地图、实时激光、红色规划路径和机器人模型。两半来自同一次运行，没有拼接也没有重新计时，顶部状态来自 `/race/state`。

俯视相机的图像坐标轴**不是**地图与 RViz 使用的世界坐标轴，所以先把其中一半转正，两半才能对比。确定旋转角度的标记物实测与其余录制设置见 [`docs/RECORDING.md`](docs/RECORDING.md)。

```bash
tools/record_all_planners.sh /tmp/planners        # 录制全部已注册规划器
```

同时展示两半的意义在于：任何一半都不够。Gazebo 说明机器人做了什么，RViz 说明导航栈相信什么、决定了走哪条路。两者不一致的时候，才是有意思的情况。

### 同一赛题，逐个规划器，其余完全相同

| 全局规划器 | 结果 | 实际行驶 | 录像 |
| :--- | :---: | ---: | :---: |
| **Weighted A\*** | `COMPLETE` | 55.2 m | ![weighted_astar](docs/media/run_sidebyside/weighted_astar.gif) |
| **A\*** | `COMPLETE` | 36.6 m | ![astar](docs/media/run_sidebyside/astar.gif) |
| **Dijkstra** | `COMPLETE` | 33.1 m | ![dijkstra](docs/media/run_sidebyside/dijkstra.gif) |
| **D\* Lite** | `COMPLETE` | 35.4 m | ![d_star_lite](docs/media/run_sidebyside/d_star_lite.gif) |
| **Theta\*** | `COMPLETE` | 76.0 m | ![theta_star](docs/media/run_sidebyside/theta_star.gif) |
| **GBFS** | `COMPLETE` | 178.9 m | ![gbfs](docs/media/run_sidebyside/gbfs.gif) |
| **JPS** | `COMPLETE` | 143.9 m | ![jps](docs/media/run_sidebyside/jps.gif) |

七个全部跑完了任务。行驶距离不是排名：它包含任务要求的每一次重规划与视点重访，所以 GBFS 的 178.9 m 反映的是它被来回派的次数，不是路径差。排名是上面那张单次规划调用的离线表；这些录像用来看行为。

MP4 在 `docs/media/run_sidebyside/<algorithm>.mp4`。直接复现一次运行：

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
ros2 launch race_navigation competition.launch.py headless:=true stress:=false
python3 tools/send_start_signal.py        # 唯一启动信号
ros2 topic echo /race/state               # 观察状态机跑完
```

### 搜索形状

`docs/media/search_2d.gif` 把每个规划器展开格子的顺序做成动画，数据来自 `algo_plan_dump` 对真实搜索的记录。颜色表示**展开顺序，且按面板各自归一化**，所以两个面板出现同一种颜色只表示各自搜索到了同样的百分比，不代表同样的工作量；要比较工作量，看每个面板下方印的格数。

![六种规划器搜索并排对比](docs/media/search_2d.gif)

<p align="center">
  <sub><a href="docs/media/search_2d.mp4">下载（MP4）</a> &nbsp;·&nbsp; 用 <code>tools/render_planning_demo.py</code> 渲染</sub>
</p>

### 动态障碍世界

同一赛道加两个移动障碍的版本，由 `stress:=true` 选择，加载 `competition_stress.world` 而不是 `competition_world.world`。上面对比统一传 `stress:=false`，让七种规划器面对同一场地与同一张代价地图。

<p align="center">
  <sub><a href="docs/media/dynamic_obstacle.mp4">dynamic_obstacle.mp4（1920 × 1080，60 fps）</a> &nbsp;·&nbsp; 用 <code>python3 tools/make_gif.py</code> 转 GIF</sub>
</p>

## 运行截图

| SLAM 建图 | 地图保存 |
| :---: | :---: |
| ![SLAM 建图过程](docs/images/01_mapping.png) | ![保存的 PGM 栅格地图](docs/images/02_map_saved.png) |
| **动态障碍世界** | **TF 坐标树** |
| ![场地中的两个移动障碍](docs/images/05_stress_world.png) | ![机器人 TF 坐标树](docs/images/06_tf_tree.png) |

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

只运行规划器，不起整个场景。此时所有已注册算法都会加载，`planner_index` 决定终端表格中显示哪一个为当前算法：

```bash
ros2 launch algo_bringup algo_planner_bench.launch.py planner_index:=4
```

## 运行结果

普通世界的一次基线运行，保留作为回归参照：

| 状态 | 完成时间 | 首次识别 | 路径长度 | 碰撞 | 末端墙距 |
| :---: | ---: | ---: | ---: | ---: | ---: |
| `COMPLETE` | 90.917 s | 63.033 s | 31.303 m | 0 | 0.3658 m |

这是开发机（Ubuntu 跑在虚拟机里、软件渲染）上的单次运行，墙钟时间只能当作下限，不是可报告的基准结果。每次运行会写入 `reports/`，包含 JSON 文件和一份可读的汇总。上面的录像是一次独立运行（`COMPLETE`，101.295 s，27.059 m，0 碰撞）。

## 目录结构

`src/algo_core` 存放算法本体，不依赖 ROS。`src/algo_nav2_plugins` 把它们适配成 `nav2_core::GlobalPlanner`，`src/algo_bringup` 存放索引、生成的参数与行为树。

其余是场景本身：`race_description`（URDF、网格、传感器、`ros2_control`）、`race_gazebo`（比赛地图、标称世界与动态障碍世界）、`race_bringup`（Gazebo、机器人、控制器、桥接、RViz）、`race_navigation`（SLAM、AMCL、Nav2、启动入口）、`race_vision`（绿色 A4 标志识别）与 `race_control`（一键启动、状态机、速度仲裁、计时报表）。

## 文档

| 文档 | 内容 |
| :--- | :--- |
| [`docs/ALGORITHM_PLUGINS.md`](docs/ALGORITHM_PLUGINS.md) | 接入新算法、选择机制、仿真与实车用法 |
| [`docs/RECORDING.md`](docs/RECORDING.md) | 双视角录制设置，以及相机坐标轴是怎么测出来的 |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | 后续工作，以及 JPS 尚未解释的差异 |
| [`tools/make_gif.py`](tools/make_gif.py) | 把录屏转换为 GIF |
| [`tools/render_planning_demo.py`](tools/render_planning_demo.py) | 根据 `algo_plan_dump` 的输出渲染规划对比动画 |

`tools/assemble_sidebyside.py` 不信任录像本身：它会测两半的平均亮度，某一半没写出来会被报告而不是被发布；也会用出生位姿核对车的位置，两半角度不一致会被抓出来。这两类问题都曾经作为静默缺陷发出去过。

### 接入新算法

1. 编写 `algo_core::GridPlanner` 的子类。
2. 用 `ALGO_CORE_REGISTER` 注册算法名。
3. 在 `algo_registry.yaml` 中新增一条记录。

Nav2 插件和行为树由注册表生成，不需要改动其他文件。

### 速度输出

规划器只返回路径，不直接控制底盘。速度发布到 `/cmd_vel_nav` 或 `/cmd_vel_final`，由 `twist_priority_mux` 选择其中一路发布到 `/cmd_vel`。

### 在实车上运行

插件发布标准的 `geometry_msgs/Twist`，包含 `linear.y`，因此仿真和实车使用同一份二进制。需要改两处设置：`controller_server` 的 `min_y_velocity_threshold` 要允许横向运动，代价地图的膨胀半径要根据实际雷达噪声重新调整。

## 路线图

见 [`docs/ROADMAP.md`](docs/ROADMAP.md)。

## 致谢

仿真、基线和原始中文文档由 [zfyyyyy](https://github.com/zfyyyyy) 编写。可互换规划器库（`algo_core`、`algo_nav2_plugins`、`algo_bringup`）、对比录像背后的录制与自检工具，以及英文文档是在其之上补充的。

本项目使用 ROS 2、Nav2、SLAM Toolbox、Gazebo 和 OpenCV。若在研究中引用，请引用 Nav2（[Marathon 2, IROS 2020](https://arxiv.org/abs/2003.00368)）、SLAM Toolbox（[JOSS 6(61):2783, 2021](https://joss.theoj.org/papers/10.21105/joss.02783)）和 Theta\*（[JAIR 39:533–579, 2010](https://www.jair.org/index.php/jair/article/view/10676)）。

## 许可证

[MIT](LICENSE)。
