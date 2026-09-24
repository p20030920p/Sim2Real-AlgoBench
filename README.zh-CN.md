<div align="center">

# Sim2Real-AlgoBench

**三轮全向机器人自主导航基准。一个任务、一套接口契约、七种可互换的全局规划器。**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-8-F58113?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/license-MIT-3DA639)](LICENSE)
[![Algorithms](https://img.shields.io/badge/algorithms-7%20registered-brightgreen)](#算法插件库)

[任务定义](#任务定义) &nbsp;•&nbsp; [算法插件库](#算法插件库) &nbsp;•&nbsp; [演示](#演示) &nbsp;•&nbsp; [快速开始](#快速开始)

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

## 算法插件库

一个规划器就是一个子类加一条注册表条目：Nav2 插件、参数和该算法的行为树都由这条条目生成，所以换插件只改算法本身。

```yaml
# src/algo_bringup/config/algo_registry.yaml
active:
  planner: 1
```

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

JPS 已注册，但在这张地图上不可用：同样代价模型的 A\* 能找到路径，它却返回 `NO_VALID_PATH`。

## 演示

| 全局规划器 | 结果 | 实际行驶 | 录像 |
| :--- | :---: | ---: | :---: |
| **Weighted A\*** | `COMPLETE` | 55.2 m | ![weighted_astar](docs/media/run_sidebyside/weighted_astar.gif) |
| **A\*** | `COMPLETE` | 36.6 m | ![astar](docs/media/run_sidebyside/astar.gif) |
| **Dijkstra** | `COMPLETE` | 33.1 m | ![dijkstra](docs/media/run_sidebyside/dijkstra.gif) |
| **D\* Lite** | `COMPLETE` | 35.4 m | ![d_star_lite](docs/media/run_sidebyside/d_star_lite.gif) |
| **Theta\*** | `COMPLETE` | 76.0 m | ![theta_star](docs/media/run_sidebyside/theta_star.gif) |
| **GBFS** | `COMPLETE` | 178.9 m | ![gbfs](docs/media/run_sidebyside/gbfs.gif) |
| **JPS** | `COMPLETE` | 143.9 m | ![jps](docs/media/run_sidebyside/jps.gif) |

### 搜索形状

`docs/media/search_2d.gif` 把每个规划器展开格子的顺序做成动画。颜色表示展开顺序、且按面板各自归一化，所以要比较工作量，看每个面板下方印的格数。

![六种规划器搜索并排对比](docs/media/search_2d.gif)

<p align="center">
  <sub><a href="docs/media/search_2d.mp4">下载（MP4）</a> &nbsp;·&nbsp; 用 <code>tools/render_planning_demo.py</code> 渲染 &nbsp;·&nbsp; <a href="docs/media/dynamic_obstacle.mp4">dynamic_obstacle.mp4（1920 × 1080，60 fps）</a> —— 同一赛道加两个移动障碍的版本，由 <code>stress:=true</code> 选择</sub>
</p>

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

跑完整任务，或只跑规划器。两种方式都会加载全部已注册算法，`planner_index` 决定终端表格中显示哪一个为当前算法。

```bash
ros2 launch race_navigation competition.launch.py headless:=false stress:=true
ros2 run race_control race_start_key          # 按一次空格或回车

ros2 launch algo_bringup algo_planner_bench.launch.py planner_index:=4
```
