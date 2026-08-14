# ROS 2 三轮全向机器人智能导航赛题仿真

基于 Ubuntu 24.04、ROS 2 Jazzy 与 Gazebo Sim 8 的三轮全向机器人完整赛题仿真项目。项目覆盖 SolidWorks 场景导入、三轮全向 `ros2_control`、SLAM 建图、已有地图定位、Nav2 全向导航、动态障碍避让、绿色终点标志识别、一键自主运行以及自动计时报表。

> 正式任务不依赖 RViz 手工发送目标点，也不预先写死随机终点坐标。机器人收到一次 `/race/start` 后，基于已有地图自主搜索，在线识别墙面绿色 A4 标志，驶入其前方黄色终点区域并静止 3 秒。

![已有地图自主导航与终点搜索](docs/images/07_finish_reached.png)

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
- `/cmd_vel_nav` 与 `/cmd_vel_final` 速度仲裁，保证最终 `/cmd_vel` 只有一个发布者；
- 空格或回车一键启动；
- 自动统计任务时间、首次识别时间、路径长度、碰撞次数和终点偏差；
- 普通世界与包含移动障碍、低附着区域、粗糙地面和变化光照的压力世界。

## 运行截图

### SLAM 建图

![SLAM 建图过程](docs/images/01_mapping.png)

### 地图保存

![保存的 PGM 栅格地图](docs/images/02_map_saved.png)

### Nav2 定点导航调试

该模式仅用于验证定位、规划和控制链路，正式任务不通过 RViz 手动发送终点。

![Nav2 定点导航调试](docs/images/03_nav2_navigation.png)

### 一键自主搜索

![一键自主搜索运行状态](docs/images/04_autonomy_simulation.png)

### 压力世界

![动态障碍与变化地面光照测试](docs/images/05_stress_world.png)

### TF 坐标树

![机器人 TF 坐标树](docs/images/06_tf_tree.png)

### 自主节点通信关系

![map_search_autonomy 通信图](docs/images/08_autonomy_graph.png)

### 动态障碍测试视频

[查看动态障碍辨别与避让视频](docs/media/dynamic_obstacle.mp4)

## 软件环境

| 组件 | 版本/用途 |
| --- | --- |
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| 仿真器 | Gazebo Sim 8（Harmonic 系列） |
| 构建工具 | colcon、CMake、ament |
| 导航 | Nav2、AMCL、Theta*、MPPI Omni |
| 建图 | SLAM Toolbox |
| 控制 | ros2_control、三轮全向控制器 |
| 视觉 | OpenCV、cv_bridge、image_transport |
| 模型 | URDF/Xacro、STL、DAE、SDF |

## 工作空间结构

```text
race_ros2_ws/
├── maps/                         # 保存的 race_map.yaml / race_map.pgm
├── reports/                      # 自动生成的比赛运行报表
├── diagnostics/                  # TF 树等诊断产物
├── docs/
│   ├── images/                   # README 运行截图
│   └── media/                    # 运行视频
└── src/
    ├── race_description/         # 机器人 URDF、网格、传感器和 ros2_control 描述
    ├── race_gazebo/              # 地图模型、普通/压力世界和动态障碍
    ├── race_bringup/             # Gazebo、机器人、控制器、桥接和 RViz 装配
    ├── race_navigation/          # SLAM、AMCL、Nav2 配置及完整启动入口
    ├── race_vision/              # 绿色 A4 终点标志识别
    └── race_control/             # 一键启动、自主状态机、速度仲裁和计时报表
```

### `race_description`

描述机器人机械结构与坐标关系，包含 `base_footprint`、`base_link`、三个轮子、`lidar_link`、`camera_link`、碰撞体和 Gazebo ros2_control 硬件接口。

### `race_gazebo`

提供彩色比赛场景、碰撞模型、普通世界、压力世界、低附着/粗糙地面、变化光照和两个移动障碍物。

### `race_bringup`

负责启动 Gazebo、生成机器人、加载 `joint_state_broadcaster` 与 `omni_drive_controller`、发布机器人 TF、桥接传感器，并把标准 `Twist` 转成控制器需要的 `TwistStamped`。

### `race_navigation`

提供 SLAM Toolbox、Map Saver、AMCL、Nav2、Theta*、MPPI Omni、代价地图、速度平滑、碰撞监控以及 `competition.launch.py` 完整入口。

### `race_vision`

通过 HSV、绿色超额量、CLAHE、形态学和连续帧判定检测绿色 A4，输出目标是否有效、归一化水平偏差和面积比。

### `race_control`

- `map_search_autonomy.py`：已有地图搜索视点、Nav2 Action 客户端、视觉接管和终点状态机；
- `twist_priority_mux.py`：在 Nav2 与视觉直控速度之间切换；
- `twist_to_twist_stamped.py`：将 `Twist` 转换为全向控制器输入；
- `race_start_key.py`：空格/回车一键启动；
- `race_metrics.py`：计时、碰撞和偏差报表；
- `moving_obstacle.py`：压力世界动态障碍运动。

## 系统数据链

```text
保存地图 + 激光雷达 + 轮式里程计
             │
             ▼
       AMCL (map → odom)
             │
             ▼
   连通区域安全搜索视点生成
             │ NavigateToPose Action
             ▼
     Theta* 全局路径规划
             │
             ▼
      MPPI Omni 局部控制
             │
             ▼
 velocity_smoother / collision_monitor
             │
             ▼
       /cmd_vel_nav ───────────┐
                               │
相机 → green_board_detector    │
             │                 │
             ▼                 ▼
/target/green_board_observation  twist_priority_mux
             │                 ▲
             ▼                 │
     视觉居中和靠近             │
             │                 │
       /cmd_vel_final ─────────┘
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

## 核心算法

### 已有地图自主搜索

1. 从 `/map` 读取已保存的占据栅格；
2. 根据比赛当天公布的已知起点初始化 AMCL；
3. 使用栅格连通域剔除墙外和不可达自由区域；
4. 按障碍净距采样候选观察点；
5. 使用向量化最远点覆盖快速选择有限搜索视点；
6. 通过 `NavigateToPose` 依次导航并原地扫描；
7. 动态障碍导致某轮视点失败时自动换点并继续下一轮，不立即永久停车；
8. 搜索时间达到约 285 秒仍未发现终点时进入 `SEARCH_EXHAUSTED`。

### 规划与避障

- 全局规划：`nav2_theta_star_planner::ThetaStarPlanner`；
- 局部控制：`nav2_mppi_controller::MPPIController`，运动模型为 `Omni`；
- 动态障碍：激光雷达写入局部/全局代价地图障碍层；
- 安全控制：速度平滑、碰撞监控和 Nav2 恢复行为。

### 终点识别与靠近

发现绿色标志后，自主节点取消当前 Nav2 目标并切换到 `/cmd_vel_final`：

1. 根据绿板水平归一化误差旋转居中；
2. 根据正前方激光距离控制靠近速度；
3. 同时满足绿板面积与挡板距离条件后停车；
4. 保持 3 秒零速后发布 `/race/complete=true`。

## 安装依赖与构建

```bash
cd ~/桌面/project/race_ros2_ws
source /opt/ros/jazzy/setup.bash

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

建议将工作空间环境加入新终端，调试时也可以每次手动执行 `source`：

```bash
source /opt/ros/jazzy/setup.bash
source ~/桌面/project/race_ros2_ws/install/setup.bash
```

## 一键运行完整比赛流程

### 1. 启动仿真、定位、导航、视觉和报表

```bash
cd ~/桌面/project/race_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch race_navigation competition.launch.py \
  headless:=false \
  nav_rviz:=true \
  stress:=true
```

启动参数：

- `headless:=false`：显示 Gazebo；视觉任务建议保持为 `false`；
- `nav_rviz:=true`：打开 Nav2 RViz，仅用于观察；
- `stress:=true`：启用移动障碍、低附着/粗糙地面和变化光照；
- 基础功能验证可使用 `stress:=false`。

等待出现：

```text
Saved-map search autonomy ready; waiting for one-button start.
Managed nodes are active
```

### 2. 空格或回车一键启动

另开终端：

```bash
cd ~/桌面/project/race_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run race_control race_start_key
```

看到提示后只按一次空格或回车。键盘节点发布一次 `/race/start=true` 后正常退出，自主任务继续运行。

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

主要状态：

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

## 重新建图

固定障碍布局发生变化时启动建图链路：

```bash
cd ~/桌面/project/race_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch race_navigation mapping.launch.py \
  headless:=false rviz:=true
```

另开终端遥控：

```bash
source /opt/ros/jazzy/setup.bash
source ~/桌面/project/race_ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

地图完整后保存：

```bash
mkdir -p ~/桌面/project/race_ros2_ws/maps

ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
"{map_topic: map, map_url: /home/zfy/桌面/project/race_ros2_ws/maps/race_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

应生成：

```text
maps/race_map.pgm
maps/race_map.yaml
```

## 关键话题与动作

| 接口 | 类型/作用 |
| --- | --- |
| `/race/start` | `std_msgs/Bool`，一键启动 |
| `/race/reset` | `std_msgs/Bool`，任务复位 |
| `/race/state` | `std_msgs/String`，状态机状态 |
| `/race/complete` | `std_msgs/Bool`，完成判定 |
| `/map` | `nav_msgs/OccupancyGrid`，静态地图 |
| `/scan` | `sensor_msgs/LaserScan`，激光扫描 |
| `/amcl_pose` | AMCL 地图位姿 |
| `/camera/image_raw` | 相机原始图像 |
| `/target/green_board_observation` | 绿板检测结果 |
| `/navigate_to_pose` | Nav2 导航 Action |
| `/cmd_vel_nav` | Nav2 速度 |
| `/cmd_vel_final` | 视觉直控速度 |
| `/race/direct_control` | 速度仲裁选择信号 |
| `/cmd_vel` | 仲裁后的唯一底盘速度 |
| `/omni_drive_controller/cmd_vel` | `TwistStamped` 控制器输入 |
| `/omni_drive_controller/odom` | 全向控制器里程计 |
| `/tf`、`/tf_static` | 动态与静态坐标变换 |

## TF 主链

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

## 诊断命令

### 节点与话题

```bash
ros2 node list
ros2 node info /map_search_autonomy
ros2 topic list -t
ros2 topic info /cmd_vel -v
ros2 topic echo /cmd_vel
ros2 topic hz /scan
ros2 action info /navigate_to_pose
```

### 控制器

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

`omni_drive_controller` 和 `joint_state_broadcaster` 均应为 `active`。

### RQT Graph

```bash
rqt_graph
```

建议使用 `Nodes/Topics (active)`，隐藏参数、TF、调试和不可达项；排查速度链时使用 `ros2 topic info ... -v` 通常比整张通信图更清晰。

### TF Tree

```bash
mkdir -p ~/桌面/project/race_ros2_ws/diagnostics
cd ~/桌面/project/race_ros2_ws/diagnostics
ros2 run tf2_tools view_frames
xdg-open "$(ls -t frames*.pdf | head -n 1)"
```

实时检查：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link lidar_link
```

## 自动计时与报表

`race_metrics` 在收到 `/race/start` 后自动计时，并记录：

- 完成时间；
- 首次识别绿色标志时间；
- 行驶路径长度；
- 碰撞次数；
- 最终速度；
- 最终墙距；
- 绿板水平偏差；
- 完成/失败状态。

报表默认写入：

```text
~/桌面/project/race_ros2_ws/reports/
```

已有一次完整仿真基准：

```text
状态：COMPLETE
完成时间：90.917 s
首次识别绿板：63.033 s
路径长度：31.303 m
碰撞次数：0
最终墙距：0.3658 m
绿板水平归一化误差：-0.0134
```

该结果对应当时的仿真参数和机器性能，只作为回归测试基线。

## 当前主要参数

- MPPI Omni：`vx_max=0.90 m/s`、`vy_max=0.65 m/s`、`wz_max=1.30 rad/s`；
- 最终视觉靠近：最高约 `0.46 m/s`；
- 搜索观察点：约 `1.0 m` 采样间距、`0.65 m` 障碍净距、最多 8 个；
- 搜索时限：约 `285 s`；
- 最终挡板目标距离：约 `0.34 m`；
- 完成判据：满足终点条件并保持 `3 s` 静止。

## 常见问题

### `/cmd_vel` 有频率但车不动

`ros2 topic hz` 只表示消息频率，应查看实际数值：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /omni_drive_controller/cmd_vel
ros2 control list_controllers
```

### RViz 显示 TF 或消息队列错误

确认所有节点使用仿真时间，并检查：

```bash
ros2 topic hz /clock
ros2 run tf2_ros tf2_echo map base_footprint
```

RViz 的 `Fixed Frame` 应设置为 `map`。

### Nav2 在动态障碍前恢复多次

压力世界中的移动障碍可能暂时封锁狭窄通道。自主程序会换点并重新开始搜索轮次；若接近 285 秒仍未发现绿板才进入 `SEARCH_EXHAUSTED`。

### 绿色标志无法识别

检查：

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /target/green_board_observation
```

实体车部署时必须根据曝光、白平衡、环境光和实际距离重新标定视觉阈值。

## 迁移到实体车前必须标定

- 比赛公布起点的 `x / y / yaw`；
- 实际轮半径、三个轮子安装角、轮速方向和最大稳定速度；
- 雷达、相机相对 `base_link` 的安装偏移；
- 相机曝光、白平衡和真实环境绿色阈值；
- 机器人投影尺寸、终点覆盖比例和挡板停车距离；
- 实体底盘硬件接口、串口/CAN 与急停；
- 碰撞外形、地面摩擦、制动距离和速度上限。

仿真通过不等于实体车可以直接高速运行。首次实车测试应降低速度，在可随时急停的空旷区域逐级标定。
