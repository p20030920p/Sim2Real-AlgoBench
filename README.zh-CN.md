<div align="center">

# Sim2Real-AlgoBench

**三轮全向机器人自主导航基准。一个任务、一套接口契约、一组指标，在仿真与实车上各评测一次。**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-8-F58113?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/license-MIT-3DA639)](LICENSE)
[![Algorithms](https://img.shields.io/badge/algorithms-7%20registered-brightgreen)](#算法插件库)

[任务定义](#任务定义) &nbsp;•&nbsp; [算法插件库](#算法插件库) &nbsp;•&nbsp; [演示](#演示) &nbsp;•&nbsp; [快速开始](#快速开始) &nbsp;•&nbsp; [文档](#文档)

*[English](README.md) &nbsp;|&nbsp; 中文*

</div>

<p align="center">
  <img src="docs/images/07_finish_reached.png" width="820" alt="已有地图自主导航与终点搜索"/>
</p>

<p align="center">
  <em>自主搜索已保存地图、识别绿色标志、驶入终点区域。全程未手工发送目标点。</em>
</p>

## 任务定义

机器人获得一个起点和一次启动信号。它需要在已知地图中搜索墙面的绿色 A4 标志，驶入标志前方的黄色区域，并静止 3 秒。

任务在识别到标志时结束，而不是到达某个坐标时结束。感知因此在两个域中都处于关键路径上，这也是仿真到实物比较值得做的原因。

## 算法插件库

全局规划器是 Nav2 插件，共有七种实现可以互换。当前使用哪一个由一行配置决定：

```yaml
# src/algo_bringup/config/algo_registry.yaml
active:
  planner: 1
```

算法本体在 `algo_core` 中，不依赖 ROS，可以单独做单元测试。`algo_nav2_plugins` 用一个 `nav2_core::GlobalPlanner` 适配器把它们接入 Nav2。`algo_bringup` 存放索引、参数和行为树。

在 race 地图（294 × 294，分辨率 0.05 m）上从 `(-3.0, -5.0)` 规划到 `(10.0, 7.0)` 的实测结果：

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

A\* 展开 26,442 格，D\* Lite 只展开 375 格，因为后者在多次调用之间保留搜索状态，只修复发生变化的部分。Theta\* 返回 8 个路径点，A\* 返回 23 个，差别来自任意角规划消除的栅格阶梯。

所有规划器同时保持加载，可以在每次请求时单独选择：

| 选择方式 | 机制 |
| :--- | :--- |
| 按请求 | `ComputePathToPose` 带有 `planner_id` 字段 |
| 按情况 | 每个算法一棵行为树，通过 `behavior_tree` 选择 |
| 脱离 Nav2 | `algo_core::Registry::instance().create("theta_star")` |

JPS 已注册但不可用。它在 race 地图上返回 `NO_VALID_PATH`，而同样代价模型的 A\* 能找到路径，说明剪枝逻辑有误。请不要用它产生结果。

## 演示

### 完整赛题一次跑通：左边 Gazebo，右边 RViz

从**单个启动信号**开始的一次完整运行，到静止 3 秒结束。左半是仿真器自己的俯视相机，**不带任何叠加**；右半是 RViz 画的同一时刻——已有地图、全局代价地图、实时激光、红色规划路径和机器人模型。两半来自同一次运行，没有拼接也没有重新计时，顶部状态来自 `/race/state`。

两半必须转到同一角度，否则这个对比没有意义。俯视相机的图像坐标轴**不是**地图与 RViz 使用的世界坐标轴。

实测方法：把 1.2 m 的标记物放在已知世界坐标上，**一次只放一个**，用它与前一帧的差分在相机图像里定位——不依赖颜色，也不依赖"看起来像"：

| 标记物位置（距相机 3 m） | 相对图像中心的偏移 |
| :--- | :--- |
| 世界 **+X** | ( −0.5, −82.1) —— 图像上方 |
| 世界 **−X** | ( −9.0, +81.5) —— 图像下方 |
| 世界 **+Y** | (−82.5, +8.0) —— 图像左侧 |

相机位于场地中心，即图像 (240, 240)。3 m 对应 82 px，即 27.4 px/m，而由相机高度与视场角推算为 27.2 px/m，两者吻合，且三个点彼此自洽。

所以世界 +X 是图像上方、世界 +Y 是图像左侧，与地图和 RViz 的朝向相差一个直角。把相机那一半**顺时针旋转 90°** 即可得到 +X 向右、+Y 向上。另用一条独立证据复核：测试时车停在已知位姿 (8.0727, 7.5312)，该朝向预测它应在 (361, 61)，实测为 (359.5, 62.2)，差约 2 px。

```bash
tools/record_all_planners.sh /tmp/planners        # 录制全部已注册规划器
```

同时展示两半的意义在于：任何一半都不够。Gazebo 说明机器人做了什么，RViz 说明导航栈相信什么、决定了走哪条路。两者不一致的时候，才是有意思的情况。

同一赛题，逐个规划器，其余完全相同：

| 全局规划器 | 结果 | 实际行驶 | 录像 |
| :--- | :---: | ---: | :---: |
| **Weighted A\*** | `COMPLETE` | 55.2 m | ![weighted_astar](docs/media/run_sidebyside/weighted_astar.gif) |
| **A\*** | `COMPLETE` | 36.6 m | ![astar](docs/media/run_sidebyside/astar.gif) |
| **Dijkstra** | `COMPLETE` | 33.1 m | ![dijkstra](docs/media/run_sidebyside/dijkstra.gif) |
| **D\* Lite** | `COMPLETE` | 35.4 m | ![d_star_lite](docs/media/run_sidebyside/d_star_lite.gif) |
| **Theta\*** | `COMPLETE` | 76.0 m | ![theta_star](docs/media/run_sidebyside/theta_star.gif) |
| **GBFS** | `COMPLETE` | 178.9 m | ![gbfs](docs/media/run_sidebyside/gbfs.gif) |
| **JPS** | 未跑完 | 214.0 m | ![jps](docs/media/run_sidebyside/jps.gif) |

七个里有六个跑完了任务，JPS 没有——这与已知情况一致：它在这张图上剪枝有误，反复无法生成路径，于是任务把整个搜索预算都花在派它去永远到不了的视点上。它的录像保留下来，是因为"看它乱走"本身有意义，不是可用结果。

行驶距离同样不是排名：它包含任务要求的每一次重规划与视点重访，所以 GBFS 的 178.9 m 反映的是它被来回派的次数，不是路径差。排名是上面那张单次规划调用的离线表；这些录像用来看行为。

MP4 在 `docs/media/run_sidebyside/<algorithm>.mp4`。直接复现一次运行：

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
ros2 launch race_navigation competition.launch.py headless:=true stress:=false
python3 tools/send_start_signal.py        # 唯一启动信号
ros2 topic echo /race/state               # 观察状态机跑完
```

### 搜索形状，以及颜色怎么读

`docs/media/search_2d.gif` 把每个规划器展开格子的顺序做成动画，数据来自 `algo_plan_dump` 对真实搜索的记录。颜色表示**展开顺序，且按面板各自归一化**：该规划器展开的第一个格子是淡黄，最后一个是饱和橙。

"按面板各自归一化"正是这张图容易被误读的地方，所以说清楚：Dijkstra 面板里的橙色大约是第 55,000 个格子，而 D\* Lite 面板里的橙色大约是第 375 个。两个面板出现同一种颜色，表示它们各自搜索到了同样的**百分比**，而不是同样的工作量。要比较各规划器做了多少工作，请看每个面板下方印的格数，不要看颜色——A\* 展开 26,442 格，而 D\* Lite 展开 375 格，这正是增量式规划器存在的全部理由。

![六种规划器搜索并排对比](docs/media/search_2d.gif)

<p align="center">
  <sub><a href="docs/media/search_2d.mp4">下载（MP4）</a> &nbsp;·&nbsp; 用 <code>tools/render_planning_demo.py</code> 渲染</sub>
</p>

### 压力世界

比赛场景运行中：左侧 Gazebo，右侧 RViz。这是压力世界，场地中额外包含低附着地面、粗糙地面和两个移动障碍。

<p align="center">
  <img src="docs/media/dynamic_obstacle.gif" width="720" alt="压力世界：左侧 Gazebo，右侧 RViz 中重新规划的路径"/>
</p>

<p align="center">
  <sub><a href="docs/media/dynamic_obstacle.mp4">下载原片（MP4，1920 × 1080，60 fps）</a> &nbsp;·&nbsp; 用 <code>python3 tools/make_gif.py</code> 重新生成</sub>
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

只运行规划器，不起整个场景。此时所有已注册算法都会加载，`planner_index` 决定终端表格中显示哪一个为当前算法：

```bash
ros2 launch algo_bringup algo_planner_bench.launch.py planner_index:=4
```

## 运行结果

普通世界的一次基线运行，保留作为回归参照：

| 状态 | 完成时间 | 首次识别 | 路径长度 | 碰撞 | 末端墙距 |
| :---: | ---: | ---: | ---: | ---: | ---: |
| `COMPLETE` | 90.917 s | 63.033 s | 31.303 m | 0 | 0.3658 m |

这是当时机器上的单次运行，不是可报告的基准结果。每次运行会写入 `reports/`，包含 JSON 文件和一份可读的汇总。

## 目录结构

```text
src/
├── algo_core/            # 算法本体，不依赖 ROS
├── algo_nav2_plugins/    # Nav2 插件适配层
├── algo_bringup/         # 算法索引、参数、行为树
├── race_description/     # URDF、网格、传感器、ros2_control
├── race_gazebo/          # 比赛地图、普通世界与压力世界
├── race_bringup/         # Gazebo、机器人、控制器、桥接、RViz
├── race_navigation/      # SLAM、AMCL、Nav2 配置、启动入口
├── race_vision/          # 绿色 A4 标志识别
└── race_control/         # 一键启动、状态机、速度仲裁、计时报表
```

## 文档

| 文档 | 内容 |
| :--- | :--- |
| [`docs/ALGORITHM_PLUGINS.md`](docs/ALGORITHM_PLUGINS.md) | 接入新算法、选择机制、仿真与实车用法 |
| [`tools/make_gif.py`](tools/make_gif.py) | 把录屏转换为上面的 GIF |
| [`tools/render_planning_demo.py`](tools/render_planning_demo.py) | 根据 `algo_plan_dump` 的输出渲染规划对比动画 |

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

- [x] 基线：SLAM Toolbox、AMCL、Theta\*、MPPI Omni、HSV 标志检测
- [x] 可互换规划器：Dijkstra、A\*、Weighted A\*、GBFS、Theta\*、D\* Lite
- [ ] JPS：修正剪枝逻辑
- [ ] 采样类规划器：RRT、RRT\*、Informed RRT\*、RRT-Connect
- [ ] 控制器适配层：DWA、APF、RPP、LQR、MPC
- [ ] 实车 bring-up 与标定记录

## 致谢

仿真、基线和原始中文文档由 [zfyyyyy](https://github.com/zfyyyyy) 编写。本项目使用 ROS 2、Nav2、SLAM Toolbox、Gazebo 和 OpenCV。

若在研究中引用，请引用 Nav2（[Marathon 2, IROS 2020](https://arxiv.org/abs/2003.00368)）、SLAM Toolbox（[JOSS 6(61):2783, 2021](https://joss.theoj.org/papers/10.21105/joss.02783)）和 Theta\*（[JAIR 39:533–579, 2010](https://www.jair.org/index.php/jair/article/view/10676)）。

## 许可证

[MIT](LICENSE)。
