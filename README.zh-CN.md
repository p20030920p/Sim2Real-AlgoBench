<div align="center">

# Sim2Real-AlgoBench

**基于 ROS 2 Jazzy 与 Gazebo Sim 8 的三轮全向机器人自主导航基准**

一个固定任务 &nbsp;·&nbsp; 一套固定接口契约 &nbsp;·&nbsp; 一组固定指标 &nbsp;—&nbsp; 在仿真与实物两个域上各评测一次

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/24.04/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-8-F58113?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![Nav2](https://img.shields.io/badge/Nav2-plugins-1E7BBF)](https://docs.nav2.org/)
[![License](https://img.shields.io/badge/license-MIT-3DA639)](LICENSE)
[![Algorithms](https://img.shields.io/badge/algorithms-7%20registered-brightgreen)](#算法插件库)

[任务定义](#任务定义) &nbsp;•&nbsp; [算法插件库](#算法插件库) &nbsp;•&nbsp; [快速开始](#快速开始) &nbsp;•&nbsp; [运行结果](#运行结果) &nbsp;•&nbsp; [路线图](#路线图) &nbsp;•&nbsp; [常见问题](#常见问题)

*[English](README.md) &nbsp;|&nbsp; 中文*

</div>

---

<p align="center">
  <img src="docs/images/07_finish_reached.png" width="860" alt="已有地图自主导航与终点搜索"/>
</p>

<p align="center">
  <em>图 1 —— 机器人自主搜索已保存地图后驶入终点区域。全程未手工发送目标点，也未预先写死终点坐标。</em>
</p>

---

## 任务定义

机器人被置于已知地图上公布的起点，接收且仅接收一次启动信号。此后它必须**自主搜索**地图，寻找墙面上的**绿色 A4 标志**，驶入其前方的**黄色终点区域**，并保持静止 **3 秒**。

正式任务不依赖 RViz 手工发送目标点，也不预先写死随机终点坐标。任务以**感知**条件终止而非几何条件——这正是仿真到实物比较具有意义的原因，因为感知恰是两个域差异最大的环节。

## 算法插件库

> **这是本仓库存在的理由。** 全局规划器是可互换的：每一个都是同一接口背后的 Nav2 插件，**换算法只需要改一个数字**。

```yaml
# src/algo_bringup/config/algo_registry.yaml
active:
  planner: 1        # >>> 换算法只需要改这个数字 <<<
```

三层结构让算法与 ROS 解耦，因此同一份代码在 Gazebo 和实车上都能跑：

| 层 | 功能包 | 作用 |
| :--- | :--- | :--- |
| 算法层 | `algo_core` | 纯 C++，**零 ROS 依赖**。可离线单元测试，也可脱离 Nav2 复用。 |
| 适配层 | `algo_nav2_plugins` | 一个 `nav2_core::GlobalPlanner` 插件服务所有已注册算法。 |
| 选择层 | `algo_bringup` | 算法索引、参数生成，以及每个算法一棵行为树。 |

### 已注册算法

在 race 地图（294 × 294 @ 0.05 m）上从 `(-3.0, -5.0)` 规划到 `(10.0, 7.0)` 的实测结果：

| 索引 | 算法 | 结果 | 点数 | 说明 |
| :---: | :--- | :---: | ---: | :--- |
| 0 | Dijkstra | ✅ | 21 | 等代价基线，最优 |
| 1 | **A\*** | ✅ | 23 | 八分启发式，默认 |
| 2 | Weighted A\* | ✅ | 29 | 更贪心，次优但更快 |
| 3 | GBFS | ✅ | 22 | 最快得到解 |
| 4 | JPS | ❌ | — | **已知缺陷**，见下 |
| 5 | **Theta\*** | ✅ | **8** | 任意角，消除栅格阶梯 |
| 6 | **D\* Lite** | ✅ | 19 | 增量重规划 |
| 7 | Nav2 自带 Theta\* | ✅ | 394 | 对照基线 |

Theta\* 只用 **8 个点**走完全程，而 A\* 需要 23 个——这就是任意角规划消除栅格阶梯的直接体现，而且它是个数字。

> **已知缺陷。** JPS 在 race 地图上返回 `NO_VALID_PATH`，而同样代价模型的 A\* 能找到路径，说明其剪枝逻辑有错。它在合成地图的自测里是通过的，所以直到接上真实地图才暴露。注册表中已标注，**修好前不要用它出结果**。

### 不同情况用不同算法

所有算法同时常驻加载，所以切换算法不需要重启。三条选择路径：

| 路径 | 机制 |
| :--- | :--- |
| **按请求** | `ComputePathToPose` 自带 `planner_id` 字段，可为每个目标点单独指定算法。 |
| **按情况** | `NavigateToPose` 没有 planner id，但**有** `behavior_tree` 字段，因此为每个算法生成一棵把 `planner_id` 写死的行为树。 |
| **脱离 Nav2** | `algo_core::Registry::instance().create("theta_star")`，不需要任何 ROS 节点。 |

"什么情况用什么"的映射也写在同一个注册表里：

```yaml
selection:
  stress_world: "P6_d_star_lite"            # 移动障碍 -> 增量重规划
  long_range_m: 8.0
  long_range_planner: "P2_weighted_astar"   # 长距离空旷 -> 更贪心的搜索
  narrow_passage_planner: "P5_theta_star"   # 窄通道 -> 任意角
```

完整指南见 [`docs/ALGORITHM_PLUGINS.md`](docs/ALGORITHM_PLUGINS.md)，含"三步加一个新算法且不改动任何已有文件"。

## 演示

<p align="center">
  <img src="docs/media/dynamic_obstacle.gif" width="720" alt="压力世界运行：左侧 Gazebo，右侧 RViz 中重新规划的路径"/>
</p>

<p align="center">
  <em>图 2 —— 压力世界。左：Gazebo Sim，实体树中可见 <code>dynamic_obstacle_1</code> 与 <code>dynamic_obstacle_2</code>。右：障碍物移动时 Nav2 正在重新规划的路径。</em>
</p>

<p align="center">
  上方 GIF 为 640 px / 8 fps 的预览版，为便于页内直接播放而压缩。<br/>
  <a href="docs/media/dynamic_obstacle.mp4"><b>⬇ 下载原始录像 &nbsp;—&nbsp; 1920 × 1080、60 fps、MP4、2.5 MB</b></a><br/>
  <sub>可用 <code>python3 tools/make_gif.py</code> 从 MP4 重新生成预览。</sub>
</p>

## 功能概览

- 彩色 SolidWorks 比赛地图转换为 Gazebo DAE/SDF 模型；
- 三轮全向底盘 URDF/Xacro、轮关节、激光雷达、相机和 `ros2_control`；
- SLAM Toolbox 二维建图和 Nav2 Map Saver 地图保存；
- 已有栅格地图、已知起点和 AMCL 在线定位；
- 从起点连通区域快速生成安全搜索视点；
- Theta\* 任意角全局路径规划，另有七个可互换替代算法；
- MPPI `Omni` 局部控制，支持纵移、横移和旋转；
- 激光雷达实时更新代价地图，处理固定和动态障碍物；
- OpenCV 绿色 A4 终点标志检测；
- 发现终点后自动取消 Nav2，执行视觉居中、激光测距靠近和停车；
- `/cmd_vel_nav` 与 `/cmd_vel_final` 速度仲裁，保证最终 `/cmd_vel` 只有**一个发布者**；
- 空格或回车一键启动；
- 自动统计任务时间、首次识别时间、路径长度、碰撞次数和终点偏差；
- 普通世界与包含移动障碍、低附着区域、粗糙地面和变化光照的压力世界。

## 系统数据链

<p align="center">
  <img src="docs/images/08_autonomy_graph.png" width="820" alt="map_search_autonomy 通信图"/>
</p>

```text
保存地图 + 激光雷达 + 轮式里程计
             │
             ▼
       AMCL (map → odom)
             │
             ▼
   自由空间连通域安全搜索视点生成
             │ NavigateToPose 动作
             ▼
   全局规划器  ──  可互换（索引 0..7）
             │
             ▼
      MPPI (Omni) 局部控制
             │
             ▼
  velocity_smoother / collision_monitor
             │
             ▼
        /cmd_vel_nav ───────────┐
                                │
 相机 → 绿色标志检测             │
             │                  │
             ▼                  ▼
 /target/green_board_observation   twist_priority_mux
             │                  ▲
             ▼                  │
      视觉居中与靠近              │
             │                  │
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
                          三个轮子关节
```

## 包结构

```text
Sim2Real-AlgoBench/
├── maps/                         # 保存的 race_map.yaml / race_map.pgm
├── reports/                      # 自动生成的运行报表
├── diagnostics/                  # TF 树等诊断产物
├── docs/                         # 图片、视频与算法指南
├── tools/                        # make_gif.py —— 重新生成演示 GIF
└── src/
    ├── algo_core/                # 算法本体，无 ROS 依赖
    ├── algo_nav2_plugins/        # Nav2 插件适配层
    ├── algo_bringup/             # 算法索引、参数、行为树
    ├── race_description/         # URDF、网格、传感器与 ros2_control 描述
    ├── race_gazebo/              # 地图模型、普通/压力世界、动态障碍
    ├── race_bringup/             # Gazebo、机器人、控制器、桥接与 RViz 装配
    ├── race_navigation/          # SLAM、AMCL、Nav2 配置及完整启动入口
    ├── race_vision/              # 绿色 A4 终点标志识别
    └── race_control/             # 一键启动、自主状态机、速度仲裁和计时报表
```

| 功能包 | 作用 |
| :--- | :--- |
| `race_description` | `base_footprint`、`base_link`、三个轮子、`lidar_link`、`camera_link`、碰撞体和 Gazebo `ros2_control` 硬件接口。 |
| `race_gazebo` | 彩色比赛场景、碰撞模型、普通世界、压力世界、低附着/粗糙地面、变化光照和两个移动障碍物。 |
| `race_bringup` | 启动 Gazebo、生成机器人、加载 `joint_state_broadcaster` 与 `omni_drive_controller`、发布机器人 TF、桥接传感器，并把标准 `Twist` 转成控制器需要的 `TwistStamped`。 |
| `race_navigation` | SLAM Toolbox、Map Saver、AMCL、Nav2、Theta\*、MPPI Omni、代价地图、速度平滑、碰撞监控以及 `competition.launch.py` 完整入口。 |
| `race_vision` | 通过 HSV、绿色超额量、CLAHE、形态学和连续帧判定检测绿色 A4，输出目标是否有效、归一化水平偏差和面积比。 |
| `race_control` | `map_search_autonomy.py`、`twist_priority_mux.py`、`twist_to_twist_stamped.py`、`race_start_key.py`、`race_metrics.py`、`moving_obstacle.py`。 |

## 依赖

| 组件 | 版本 / 用途 |
| :--- | :--- |
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| 仿真器 | Gazebo Sim 8（Harmonic 系列） |
| 构建工具 | colcon、CMake、ament |
| 导航 | Nav2、AMCL、Theta\*、MPPI Omni |
| 建图 | SLAM Toolbox |
| 控制 | `ros2_control`、三轮全向控制器 |
| 视觉 | OpenCV、`cv_bridge`、`image_transport` |
| 模型 | URDF/Xacro、STL、DAE、SDF |

## 编译

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

建议将工作空间环境加入新终端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/Sim2Real-AlgoBench/install/setup.bash
```

## 快速开始

<table>
<tr><th align="left">1 —— 启动完整任务</th></tr>
<tr><td>

```bash
ros2 launch race_navigation competition.launch.py \
  headless:=false nav_rviz:=true stress:=true
```

`headless:=false` 显示 Gazebo（感知任务必需）；`nav_rviz:=true` 打开 Nav2 RViz 仅供观察；`stress:=true` 启用移动障碍、低附着地面和变化光照。功能验证可用 `stress:=false`。

等待出现 `Saved-map search autonomy ready; waiting for one-button start.`

</td></tr>
<tr><th align="left">2 —— 一键启动</th></tr>
<tr><td>

```bash
ros2 run race_control race_start_key
```

看到提示后**只按一次**空格或回车。脚本化运行可用：

```bash
ros2 topic pub --once /race/start std_msgs/msg/Bool "{data: true}"
```

</td></tr>
<tr><th align="left">3 —— 观察状态机</th></tr>
<tr><td>

```bash
ros2 topic echo /race/state \
  --qos-reliability reliable --qos-durability transient_local
```

`WAITING_FOR_ONE_BUTTON_START` → `PREPARING_MAP_SEARCH` → `NAVIGATING_TO_SEARCH_VIEWPOINT` → `SCANNING_360_FOR_GREEN_BOARD` → `ALIGNING_WITH_GREEN_BOARD` → `APPROACHING_YELLOW_FINISH_PAD` → `HOLDING_STILL_FOR_3_SECONDS` → `COMPLETE`，或 `SEARCH_EXHAUSTED`。

</td></tr>
<tr><th align="left">4 —— 单独横评算法</th></tr>
<tr><td>

```bash
ros2 launch algo_bringup algo_planner_bench.launch.py
```

只拉起地图服务器和规划服务器，但**注册全部算法**，因此可以不起整个比赛场景就在 race 地图上横评算法。不改文件也可临时覆盖索引：

```bash
ros2 launch algo_bringup algo_planner_bench.launch.py planner_index:=4
```

</td></tr>
</table>

### 复位

```bash
ros2 topic pub --once /race/reset std_msgs/msg/Bool "{data: true}"
```

取消当前导航目标、发布零速度并回到等待状态。若要把机器人恢复到物理起点，应退出并重新启动整套 launch。

## 运行截图

| SLAM 建图 | 地图保存 |
| :---: | :---: |
| ![SLAM 建图过程](docs/images/01_mapping.png) | ![保存的 PGM 栅格地图](docs/images/02_map_saved.png) |
| **Nav2 定点导航调试** | **一键自主搜索** |
| ![Nav2 定点导航调试](docs/images/03_nav2_navigation.png) | ![一键自主搜索运行状态](docs/images/04_autonomy_simulation.png) |
| **压力世界** | **TF 坐标树** |
| ![动态障碍与变化地面光照测试](docs/images/05_stress_world.png) | ![机器人 TF 坐标树](docs/images/06_tf_tree.png) |

## 重新建图

固定障碍布局发生变化时启动建图链路：

```bash
ros2 launch race_navigation mapping.launch.py headless:=false rviz:=true
```

另开终端遥控：

```bash
source /opt/ros/jazzy/setup.bash
source ~/Sim2Real-AlgoBench/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

地图完整后保存：

```bash
mkdir -p ~/Sim2Real-AlgoBench/maps

ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
"{map_topic: map, map_url: $HOME/Sim2Real-AlgoBench/maps/race_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

应生成 `maps/race_map.pgm` 与 `maps/race_map.yaml`。

## 接口参考

| 接口 | 类型 / 作用 |
| :--- | :--- |
| `/race/start` | `std_msgs/Bool`，一键启动 |
| `/race/reset` | `std_msgs/Bool`，任务复位 |
| `/race/state` | `std_msgs/String`，状态机状态 |
| `/race/complete` | `std_msgs/Bool`，完成判定 |
| `/race/direct_control` | 速度仲裁选择信号 |
| `/map` | `nav_msgs/OccupancyGrid`，静态地图 |
| `/scan` | `sensor_msgs/LaserScan`，激光扫描 |
| `/amcl_pose` | AMCL 地图位姿 |
| `/camera/image_raw` | 相机原始图像 |
| `/target/green_board_observation` | 绿板检测结果 |
| `/navigate_to_pose` | Nav2 导航 Action |
| `/compute_path_to_pose` | Nav2 规划 Action，携带 `planner_id` |
| `/cmd_vel_nav` | Nav2 速度 |
| `/cmd_vel_final` | 视觉直控速度 |
| `/cmd_vel` | 仲裁后的唯一底盘速度，**单一发布者** |
| `/omni_drive_controller/cmd_vel` | `TwistStamped` 控制器输入 |
| `/omni_drive_controller/odom` | 全向控制器里程计 |
| `/tf`、`/tf_static` | 动态与静态坐标变换 |

### TF 主链

<p align="center">
  <img src="docs/images/06_tf_tree.png" width="560" alt="机器人 TF 坐标树"/>
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

- AMCL 发布 `map → odom`；
- 全向控制器发布 `odom → base_footprint`；
- `robot_state_publisher` 根据 URDF 和 `/joint_states` 发布机器人本体坐标关系。

## 主要参数

| 参数 | 数值 |
| :--- | :--- |
| MPPI Omni | `vx_max = 0.90 m/s`、`vy_max = 0.65 m/s`、`wz_max = 1.30 rad/s` |
| 末端视觉靠近 | 最高约 0.46 m/s |
| 搜索观察点 | 约 1.0 m 采样间距、0.65 m 障碍净距、最多 8 个 |
| 搜索时限 | 约 285 s，之后进入 `SEARCH_EXHAUSTED` |
| 末端挡板目标距离 | 约 0.34 m |
| 完成判据 | 满足终点条件并保持 3 s 静止 |

## 运行结果

已有一次完整仿真基准，保留作为回归参照：

| 指标 | 数值 |
| :--- | :--- |
| 状态 | `COMPLETE` |
| 完成时间 | 90.917 s |
| 首次识别绿板 | 63.033 s |
| 路径长度 | 31.303 m |
| 碰撞次数 | 0 |
| 最终墙距 | 0.3658 m |
| 绿板水平归一化误差 | −0.0134 |

> 这是当时的仿真参数和机器上的**单次**运行，属于回归基线而非可报告的基准结果。每次运行的记录写入 `reports/`（`*.json` 加一份可读的 `*.md`），跨运行汇总为 `reports/race_summary.csv`。

## 路线图

`[x]` = 已就位，`[ ]` = 待接入。

**基线**

- [x] 建图 —— SLAM Toolbox
- [x] 定位 —— AMCL
- [x] 全局规划 —— Theta\* 加 7 个可互换替代算法
- [x] 局部控制 —— MPPI，`Omni` 运动模型
- [x] 感知 —— HSV + 绿色超额量阈值检测器

**规划器**

- [x] Dijkstra、A\*、Weighted A\*、GBFS、Theta\*、D\* Lite
- [ ] JPS —— 已实现但在 race 地图上**失败**，待调试
- [ ] RRT、RRT\*、Informed RRT\*、RRT-Connect
- [ ] Hybrid A\*、Voronoi、ACO、GA、PSO

**控制器**

- [ ] DWA、APF、RPP、PID、LQR、MPC —— 适配层尚未编写
- [ ] 通过 `controller_id` 为每个目标点选择不同控制器

**实物轨道**

- [ ] 实车 bring-up 与标定记录
- [ ] 建图 / 定位 / 规划 / 控制 / 感知在实车上验证

## 如何接入新算法

**提 PR 前请先读这一节。** 本仓库的意义在于：算法可以只替换**一个**类别，其余类别保持基线实现，从而保证结果可比。

1. **只选一个类别**——建图、定位、全局规划、局部控制、感知或任务逻辑。
2. **满足该类别的接口。** 规划器实现 `algo_core::GridPlanner` 并自注册：

   ```cpp
   class MyPlanner final : public algo_core::GridPlanner {
   public:
     std::string name() const override { return "my_planner"; }
     algo_core::PlanResult plan(const algo_core::CostGrid & grid,
                                const algo_core::Pose2D & start,
                                const algo_core::Pose2D & goal) override;
   };
   ALGO_CORE_REGISTER(algo_core::MyPlanner, "my_planner")
   ```

3. **在 `algo_registry.yaml` 加一条**——插件类和行为树都会自动生成，不需要改别的。
4. **绝不直接向 `/cmd_vel` 发布。** 写入 `/cmd_vel_nav` 或 `/cmd_vel_final`，由仲裁器裁决。`/cmd_vel` 单一发布者是契约，不是细节。
5. **不要为了让贡献跑通而修改基线包。** 若觉得非改不可，说明接口设计有问题，请先开 issue。
6. **报告与基线相同的指标**（`race_metrics`），并声明任何协议偏差。

## 常见问题

**`/cmd_vel` 有频率但车不动。**
`ros2 topic hz` 只表示消息频率，应查看实际数值与控制器状态：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /omni_drive_controller/cmd_vel
ros2 control list_controllers
```

**RViz 显示 TF 或消息队列错误。**
确认所有节点使用仿真时间，并把 RViz 的 `Fixed Frame` 设为 `map`：

```bash
ros2 topic hz /clock
ros2 run tf2_ros tf2_echo map base_footprint
```

**规划器报 `NO_VALID_PATH`（错误码 208），但地图看起来正常。**
检查 `lethal_threshold`：默认值 `253` 会把 Nav2 的内切膨胀层当作障碍，可能封死窄通道。在注册表条目里降到 `254` 即可穿过膨胀层规划。

**Nav2 在动态障碍前恢复多次。**
压力世界中的移动障碍可能暂时封锁狭窄通道。自主程序会换点并重新开始搜索轮次；若接近 285 秒仍未发现绿板才进入 `SEARCH_EXHAUSTED`。

**绿色标志无法识别。**

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /target/green_board_observation
```

实体车部署时必须根据曝光、白平衡、环境光和实际距离重新标定视觉阈值。

### 诊断命令

```bash
ros2 node list
ros2 node info /map_search_autonomy
ros2 topic list -t
ros2 topic info /cmd_vel -v
ros2 action info /navigate_to_pose
ros2 control list_hardware_interfaces
rqt_graph                       # 建议使用 "Nodes/Topics (active)"

mkdir -p diagnostics && cd diagnostics
ros2 run tf2_tools view_frames
```

`omni_drive_controller` 和 `joint_state_broadcaster` 均应为 `active`。

## 迁移到实体车前

仿真通过**并不**意味着可以高速运行实物。首次实车测试必须在降低的速度下、在空旷区域、并具备随时可用的急停的条件下进行，且逐项标定：

- 比赛公布起点的 `x / y / yaw`；
- 实际轮半径、三个轮子安装角、轮速方向和最大稳定速度；
- 雷达、相机相对 `base_link` 的安装偏移；
- 相机曝光、白平衡和真实环境绿色阈值；
- 机器人投影尺寸、终点覆盖比例和挡板停车距离；
- 实体底盘硬件接口、串口/CAN 与急停；
- 碰撞外形、地面摩擦、制动距离和速度上限。

**算法插件上实车不需要任何改动。** 它们输出标准的 `geometry_msgs/Twist`（含 `linear.y`），现有链路（`/cmd_vel` → `TwistStamped` → `omni_drive_controller`）完全不用碰。两点需要注意：在 `controller_server` 中放开横向速度（`min_y_velocity_threshold`），以及针对真实传感器噪声重新标定代价地图膨胀半径。

## 致谢

仿真、基线实现与原始中文文档由 [zfyyyyy](https://github.com/zfyyyyy) 完成。本项目建立在 ROS 2、Nav2、SLAM Toolbox、Gazebo 与 OpenCV 生态之上。

若在研究中引用，请引用所依赖的以下工作：

- S. Macenski, F. Martín, R. White, J. Clavero. *The Marathon 2: A Navigation System.* IROS, 2020.
- S. Macenski, I. Jambrecic. *SLAM Toolbox: SLAM for the dynamic world.* JOSS, 6(61): 2783, 2021.
- K. Daniel, A. Nash, S. Koenig, A. Felner. *Theta-star: Any-Angle Path Planning on Grids.* JAIR, 39: 533–579, 2010.
- G. Williams, N. Wagener, B. Goldfain, P. Drews, J. M. Rehg, B. Boots, E. A. Theodorou. *Information Theoretic MPC for Model-Based Reinforcement Learning.* ICRA, 2017.

## 许可证

[MIT](LICENSE)。
