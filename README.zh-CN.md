# Sim2Real-AlgoBench

**基于 ROS 2 Jazzy 与 Gazebo Sim 8 的三轮全向机器人自主导航基准。** 一个固定任务、一套固定接口契约、一组固定指标——在仿真与实物两个域上各评测一次。

<p align='center'>
    <img src="docs/images/07_finish_reached.png" alt="已有地图自主导航与终点搜索" width="800"/>
</p>

<p align='center'>
    <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"/></a>
    <img src="https://img.shields.io/badge/ROS%202-Jazzy-blue.svg" alt="ROS 2 Jazzy"/>
    <img src="https://img.shields.io/badge/Ubuntu-24.04-orange.svg" alt="Ubuntu 24.04"/>
    <img src="https://img.shields.io/badge/Gazebo%20Sim-8-lightgrey.svg" alt="Gazebo Sim 8"/>
    <img src="https://img.shields.io/badge/status-active-brightgreen.svg" alt="Status: active"/>
</p>

**English: [README.md](README.md)**

---

## 目录

- [**任务定义**](#任务定义)
- [**功能概览**](#功能概览)
- [**系统数据链**](#系统数据链)
- [**包结构**](#包结构)
- [**依赖**](#依赖)
- [**编译**](#编译)
- [**快速开始**](#快速开始)
- [**重新建图**](#重新建图)
- [**关键话题与动作**](#关键话题与动作)
- [**TF 主链**](#tf-主链)
- [**主要参数**](#主要参数)
- [**运行结果**](#运行结果)
- [**路线图**](#路线图)
- [**如何接入新算法**](#如何接入新算法)（必读）
- [**常见问题**](#常见问题)
- [**迁移到实体车前**](#迁移到实体车前)
- [**致谢**](#致谢)
- [**许可证**](#许可证)

## 任务定义

机器人被置于已知地图上公布的起点，接收且仅接收一次启动信号。此后它必须**自主搜索**地图，寻找安装在墙面上的**绿色 A4 标志**，驶入该标志前方的**黄色终点区域**，并保持静止 **3 秒**。

正式任务不依赖 RViz 手工发送目标点，也不预先写死随机终点坐标。任务以**感知**条件终止而非几何条件——这正是仿真到实物比较具有意义的原因，因为感知恰是两个域差异最大的环节。

## 功能概览

- 彩色 SolidWorks 比赛地图转换为 Gazebo DAE/SDF 模型；
- 三轮全向底盘 URDF/Xacro、轮关节、激光雷达、相机和 `ros2_control`；
- SLAM Toolbox 二维建图和 Nav2 Map Saver 地图保存；
- 已有栅格地图、已知起点和 AMCL 在线定位；
- 从起点连通区域快速生成安全搜索视点；
- Theta* 任意角全局路径规划；
- MPPI `Omni` 局部控制，支持纵移、横移和旋转；
- 激光雷达实时更新代价地图，处理固定和动态障碍物；
- OpenCV 绿色 A4 终点标志检测；
- 发现终点后自动取消 Nav2，执行视觉居中、激光测距靠近和停车；
- `/cmd_vel_nav` 与 `/cmd_vel_final` 速度仲裁，保证最终 `/cmd_vel` 只有**一个发布者**；
- 空格或回车一键启动；
- 自动统计任务时间、首次识别时间、路径长度、碰撞次数和终点偏差；
- 普通世界与包含移动障碍、低附着区域、粗糙地面和变化光照的压力世界。

## 系统数据链

<p align='center'>
    <img src="docs/images/08_autonomy_graph.png" alt="map_search_autonomy 通信图" width="800"/>
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
     Theta* 任意角全局路径规划
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
├── docs/                         # 图片与视频
└── src/
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
| `race_navigation` | SLAM Toolbox、Map Saver、AMCL、Nav2、Theta*、MPPI Omni、代价地图、速度平滑、碰撞监控以及 `competition.launch.py` 完整入口。 |
| `race_vision` | 通过 HSV、绿色超额量、CLAHE、形态学和连续帧判定检测绿色 A4，输出目标是否有效、归一化水平偏差和面积比。 |
| `race_control` | `map_search_autonomy.py`、`twist_priority_mux.py`、`twist_to_twist_stamped.py`、`race_start_key.py`、`race_metrics.py`、`moving_obstacle.py`。 |

## 依赖

| 组件 | 版本 / 用途 |
| :--- | :--- |
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| 仿真器 | Gazebo Sim 8（Harmonic 系列） |
| 构建工具 | colcon、CMake、ament |
| 导航 | Nav2、AMCL、Theta*、MPPI Omni |
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

### 1. 启动仿真、定位、导航、视觉和报表

```bash
ros2 launch race_navigation competition.launch.py \
  headless:=false \
  nav_rviz:=true \
  stress:=true
```

| 参数 | 作用 |
| :--- | :--- |
| `headless:=false` | 显示 Gazebo；视觉任务建议保持为 `false`。 |
| `nav_rviz:=true` | 打开 Nav2 RViz，仅用于观察。 |
| `stress:=true` | 启用移动障碍、低附着/粗糙地面和变化光照；基础功能验证可用 `stress:=false`。 |

等待出现：

```text
Saved-map search autonomy ready; waiting for one-button start.
Managed nodes are active
```

### 2. 空格或回车一键启动

另开终端：

```bash
ros2 run race_control race_start_key
```

看到提示后**只按一次**空格或回车。键盘节点发布一次 `/race/start=true` 后正常退出，自主任务继续运行。

调试阶段也可直接发布：

```bash
ros2 topic pub --once /race/start std_msgs/msg/Bool "{data: true}"
```

### 3. 观察比赛状态

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

### 4. 重置

```bash
ros2 topic pub --once /race/reset std_msgs/msg/Bool "{data: true}"
```

这会取消当前导航目标、发布零速度并回到等待状态。若要把 Gazebo 机器人恢复到物理起点，应退出并重新启动整套 launch。

### 运行截图

| SLAM 建图 | 地图保存 |
| :---: | :---: |
| ![SLAM 建图过程](docs/images/01_mapping.png) | ![保存的 PGM 栅格地图](docs/images/02_map_saved.png) |

| Nav2 定点导航调试 | 一键自主搜索 |
| :---: | :---: |
| ![Nav2 定点导航调试](docs/images/03_nav2_navigation.png) | ![一键自主搜索运行状态](docs/images/04_autonomy_simulation.png) |

| 压力世界 | TF 坐标树 |
| :---: | :---: |
| ![动态障碍与变化地面光照测试](docs/images/05_stress_world.png) | ![机器人 TF 坐标树](docs/images/06_tf_tree.png) |

[▶ 查看动态障碍辨别与避让视频](docs/media/dynamic_obstacle.mp4)

## 重新建图

固定障碍布局发生变化时启动建图链路：

```bash
ros2 launch race_navigation mapping.launch.py \
  headless:=false rviz:=true
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

## 关键话题与动作

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
| `/cmd_vel_nav` | Nav2 速度 |
| `/cmd_vel_final` | 视觉直控速度 |
| `/cmd_vel` | 仲裁后的唯一底盘速度，**单一发布者** |
| `/omni_drive_controller/cmd_vel` | `TwistStamped` 控制器输入 |
| `/omni_drive_controller/odom` | 全向控制器里程计 |
| `/tf`、`/tf_static` | 动态与静态坐标变换 |

## TF 主链

<p align='center'>
    <img src="docs/images/06_tf_tree.png" alt="机器人 TF 坐标树" width="600"/>
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

按"替换哪一类"分组的算法槽位。`[x]` = 基线已就位，`[ ]` = 待接入。

**基线**

- [x] 建图 —— SLAM Toolbox
- [x] 定位 —— AMCL
- [x] 全局规划 —— Theta*（`nav2_theta_star_planner`）
- [x] 局部控制 —— MPPI，`Omni` 运动模型
- [x] 感知 —— HSV + 绿色超额量阈值检测器

**仿真轨道**

- [ ] 建图 / SLAM —— `<算法名称>`
- [ ] 定位 —— `<算法名称>`
- [ ] 全局规划 —— `<算法名称>`
- [ ] 局部控制 —— `<算法名称>`
- [ ] 感知 —— `<算法名称>`
- [ ] 任务逻辑 —— `<算法名称>`

**实物轨道**

- [ ] 实车 bring-up 与标定记录
- [ ] 建图 / SLAM —— `<算法名称>`
- [ ] 定位 —— `<算法名称>`
- [ ] 全局规划 —— `<算法名称>`
- [ ] 局部控制 —— `<算法名称>`
- [ ] 感知 —— `<算法名称>`

## 如何接入新算法

**提 PR 前请先读这一节。** 本仓库的意义在于：算法可以只替换**一个**类别，其余类别保持基线实现，从而保证结果可比。

1. **只选一个类别**——建图、定位、全局规划、局部控制、感知或任务逻辑。
2. **满足该类别的接口**（见[关键话题与动作](#关键话题与动作)）。在 `src/` 下新增功能包；若算法根本不属于本栈，则新增一个工作空间。
3. **绝不直接向 `/cmd_vel` 发布。** 写入 `/cmd_vel_nav` 或 `/cmd_vel_final`，由仲裁器裁决。`/cmd_vel` 单一发布者是契约，不是细节。
4. **不要为了让贡献跑通而修改基线包。** 若觉得非改不可，说明接口设计有问题，请先开 issue 讨论。
5. **报告与基线相同的指标**（`race_metrics`），并声明任何协议偏差。
6. **更新[路线图](#路线图)**，并提交说明所替换类别的 Pull Request。

无论哪个域，每次运行报告的指标为：完成时间、首次识别时间、路径长度、碰撞次数、末端速度、末端墙距、绿板水平误差与结果状态。

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

**Nav2 在动态障碍前恢复多次。**
压力世界中的移动障碍可能暂时封锁狭窄通道。自主程序会换点并重新开始搜索轮次；若接近 285 秒仍未发现绿板才进入 `SEARCH_EXHAUSTED`。

**绿色标志无法识别。**

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /target/green_board_observation
```

实体车部署时必须根据曝光、白平衡、环境光和实际距离重新标定视觉阈值。

**常用诊断命令**

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
xdg-open "$(ls -t frames*.pdf | head -n 1)"

ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_link lidar_link
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

## 致谢

仿真、基线实现与原始中文文档由 [zfyyyyy](https://github.com/zfyyyyy) 完成。本项目建立在 ROS 2、Nav2、SLAM Toolbox、Gazebo 与 OpenCV 生态之上。

若在研究中引用，请引用所依赖的以下工作：

- S. Macenski, F. Martín, R. White, J. Clavero. *The Marathon 2: A Navigation System.* IROS, 2020.
- S. Macenski, I. Jambrecic. *SLAM Toolbox: SLAM for the dynamic world.* JOSS, 6(61): 2783, 2021.
- K. Daniel, A. Nash, S. Koenig, A. Felner. *Theta-star: Any-Angle Path Planning on Grids.* JAIR, 39: 533–579, 2010.
- G. Williams, N. Wagener, B. Goldfain, P. Drews, J. M. Rehg, B. Boots, E. A. Theodorou. *Information Theoretic MPC for Model-Based Reinforcement Learning.* ICRA, 2017.

## 许可证

[MIT](LICENSE)。
