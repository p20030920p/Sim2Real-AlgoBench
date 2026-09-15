# 算法插件库

在 race 场景中并排运行多种规划算法。换算法改一个索引，同一套算法在仿真和实车上使用同一份二进制。

## 1. 结构

```
algo_core/            算法本体，不依赖 ROS。可离线单元测试，也可脱离 Nav2 使用
      ↑
algo_nav2_plugins/    Nav2 适配层，把算法暴露成 nav2_core::GlobalPlanner 插件
      ↑
algo_bringup/         选择层：算法索引、参数生成、行为树生成、启动文件
```

算法本体和 ROS 分开，是为了让算法能独立验证。`algo_core` 中没有 `rclcpp`、`costmap_2d` 和参数框架的引用，所以可以直接在单元测试里运行（`algo_core_selftest`），也可以被 Nav2 之外的代码使用。接入一个算法只需要写算法本身。

## 2. 换算法

修改 `algo_bringup/config/algo_registry.yaml` 中的 `active.planner`：

```yaml
active:
  planner: 1
```

启动后终端会打印当前生效的算法：

```
==============================================================================
  ALGORITHM INDEX
==============================================================================
    [ 0] P0_dijkstra            dijkstra
 -> [ 1] P1_astar               astar
    [ 2] P2_weighted_astar      weighted_astar
    ...
==============================================================================
  active planner: [1] P1_astar   (change active.planner in algo_registry.yaml)
==============================================================================
```

也可以用启动参数临时覆盖，不必修改文件：

```bash
ros2 launch algo_bringup algo_planner_bench.launch.py planner_index:=4
```

## 3. 选择机制

所有算法同时注册进 Nav2 并保持加载，因此切换不需要重启。有三种选择方式。

### 3.1 按请求指定

`ComputePathToPose` 动作带有 `planner_id` 字段，可以为每个目标点单独指定算法：

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{start: {...}, goal: {...}, planner_id: 'P4_jps', use_start: true}"
```

### 3.2 按情况自动选择

`NavigateToPose` 没有 `planner_id` 字段，但带有 `behavior_tree` 字段。`algo_bringup` 为每个算法生成一棵行为树，其中写入了对应的 `planner_id`：

```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="P4_jps"/>
```

任务节点把对应的文件名填入 goal 的 `behavior_tree` 即可切换算法。选择规则写在同一个注册表中：

```yaml
selection:
  by_state:
    PREPARING_MAP_SEARCH: "P1_astar"
    NAVIGATING_TO_SEARCH_VIEWPOINT: "P1_astar"
  stress_world: "P6_d_star_lite"            # 移动障碍场景使用增量重规划
  long_range_m: 8.0
  long_range_planner: "P2_weighted_astar"   # 长距离使用更贪心的搜索
  narrow_passage_planner: "P5_theta_star"   # 窄通道使用任意角规划
```

### 3.3 脱离 Nav2 使用

`algo_core` 可以不经 ROS 使用：

```cpp
#include "algo_core/registry.hpp"

auto planner = algo_core::Registry::instance().create("theta_star");
planner->configure(params);
algo_core::PlanResult r = planner->plan(grid, start, goal);
```

`algo_core_selftest` 就是这样写的，它不启动任何 ROS 节点。

## 4. 在仿真中使用

### 单独横评

```bash
ros2 launch algo_bringup algo_planner_bench.launch.py
```

该命令只启动 `map_server`、`planner_server` 和静态 TF，加载 race 地图，用于快速比较算法，不需要启动 Gazebo。

### 接入完整比赛流程

`algo_bringup` 生成的 `planner_server` 参数块可以替换
`race_navigation/config/nav2_params.yaml` 中的 `planner_server` 段，`competition.launch.py` 不需要修改。原有的 Nav2 Theta\* 基线以 `external: true` 条目（`P7_nav2_theta_star`）保留在同一张表中，因此新旧算法使用相同的地图、参数和指标。

## 5. 在实车上使用

插件发布标准的 `geometry_msgs/Twist`，包含 `linear.y`，链路为：

```
插件 → /cmd_vel → Twist → TwistStamped → omni_drive_controller → 三个轮子
```

仿真和实车使用同一份 `.so`。两者的差异只在代价地图的数据来源和标定参数，与算法本身无关。

实车上需要处理三点：

1. **横向速度**。`controller_server` 的 `min_y_velocity_threshold` 和控制器参数都要允许 `vy`，否则全向底盘不会横移。
2. **计算量**。目前实现的都是栅格搜索，规划耗时在毫秒级。后续接入采样类（RRT\*）和演化类（ACO、GA、PSO）时需要先测规划耗时，这些算法在实车上可能跟不上重规划频率。
3. **代价地图阈值**。`lethal_threshold` 默认为 253，与 NavFn 一致，即把内切膨胀层视为障碍。实车传感器噪声更大，需要重新标定膨胀半径。

## 6. 接入新算法

1. 在 `algo_core/src/` 下实现 `algo_core::GridPlanner` 的子类：

```cpp
class MyPlanner final : public algo_core::GridPlanner {
public:
  std::string name() const override { return "my_planner"; }
  algo_core::PlanResult plan(const algo_core::CostGrid & grid,
                             const algo_core::Pose2D & start,
                             const algo_core::Pose2D & goal) override;
};
```

2. 在文件末尾注册，并加入 `algo_core/CMakeLists.txt`：

```cpp
ALGO_CORE_REGISTER(algo_core::MyPlanner, "my_planner")
```

3. 在 `algo_registry.yaml` 中新增一条记录：

```yaml
  - index: 8
    id: "P8_my_planner"
    algorithm: "my_planner"
    description: "..."
    params: {cost_scale: 1.0}
```

启动后会打印 `algorithm 'my_planner' ready`，此后 `planner_id: P8_my_planner` 即可使用。插件类和行为树会自动生成。

## 7. 当前算法状态

在 race 地图（294 × 294，分辨率 0.05 m）上从 `(-3.0, -5.0)` 规划到 `(10.0, 7.0)` 的实测结果：

| 索引 | 算法 | 找到路径 | 路径点数 | 展开格数 |
| :--- | :--- | :---: | ---: | ---: |
| 0 | Dijkstra | 是 | 21 | 55,578 |
| 1 | A\*（默认） | 是 | 23 | 26,442 |
| 2 | Weighted A\* | 是 | 29 | 9,025 |
| 3 | GBFS | 是 | 22 | 3,127 |
| 4 | JPS | 否 | — | 5 |
| 5 | Theta\* | 是 | 8 | 20,694 |
| 6 | D\* Lite | 是 | 19 | 375 |
| 7 | Nav2 自带 Theta\* | 是 | 394 | — |

**JPS 不可用。** 它在 race 地图上返回 `NO_VALID_PATH`，展开 5 个格子后终止；同样代价模型的 A\* 能找到路径，说明 jump 或邻居剪枝逻辑有误。它在合成地图的自测中可以通过，因此问题只在真实地图上暴露。修好之前不要用它产生结果。

## 8. 测试

算法自测不依赖 ROS：

```bash
colcon test --packages-select algo_core
# 或直接运行
./install/algo_core/lib/algo_core/algo_core_selftest
```

自测检查四项内容：算法能找到路径、路径连接起终点、路径不穿过障碍、D\* Lite 在代价地图变化后能正确修复。

## 9. 生成演示动画

```bash
# 导出各算法在 race 地图上的搜索结果
export LD_LIBRARY_PATH=$PWD/install/algo_core/lib:$LD_LIBRARY_PATH
./install/algo_core/lib/algo_core/algo_plan_dump \
  src/race_navigation/maps/race_map.pgm /tmp/plan.bin \
  -3.0 -5.0 10.0 7.0 -3.700 -6.342 0.050 0.196 0.65

# 渲染 2D 并排对比（六种算法）
python3 tools/render_planning_demo.py /tmp/plan.bin \
  src/race_navigation/maps/race_map.pgm docs/media/search_2d.gif

```

> 注意：`docs/media/search_2d.gif` 必须由**与第 7 节同一份** `plan.bin` 渲染。
> 如果这份 dump 来自其它起点/终点或其它代价模型，图上的路径与第 7 节的展开格数就对不上。

## 9.1 完整任务录像

上一节的动画展示的是规划器本身。展示**完整赛题链路**（AMCL → Nav2 搜索 → 绿色标志识别 → 终点靠近 → 静止 3 秒）的录像由 `competition.launch.py` 产生：

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
ros2 launch race_navigation competition.launch.py headless:=true stress:=false render_engine:=ogre

# 俯视相机。注意：它的图像相对世界坐标系旋转 180°，录制时要转回来，
# 否则 Gazebo 与 RViz 两半的场地朝向相反。角度是实测的，见 README。
ros2 run ros_gz_sim create -file tools/topcam.sdf -name topcam -x 3.65 -y 1.0 -z 10.5 -P 1.5708
ros2 run ros_gz_bridge parameter_bridge "/top_view@sensor_msgs/msg/Image[gz.msgs.Image"

# 一键启动：必须用 transient-local，否则 map_search_autonomy 收不到
ros2 topic pub --once -w 1 /race/start std_msgs/msg/Bool "{data: true}"
ros2 topic echo /race/state          # WAITING -> ... -> COMPLETE
```

产物在 `reports/` 下（JSON + 可读摘要），录像在 `docs/media/run_sidebyside/`。

**注意俯视相机的朝向。** 它的图像相对地图与 RViz 使用的世界坐标系旋转了 180°。
录制程序（`tools/record_run_sidebyside.py`）会把相机那一半转回来，否则左右两半
的场地朝向相反，对比就没有意义。这个角度是实测的，见 README 里的标记物数据。

**踩过的坑：** `ros2 topic pub` 默认是 volatile，而 `/race/start` 的订阅端是 transient-local，两者 QoS 不兼容，信号发不出去——必须先等订阅者匹配上再发。

## 10. 仿真录制的环境要求

录制 Gazebo 画面时踩到的坑，记录在此以免重复排查。

**必须指定 `render_engine:=ogre`。** 默认的 `ogre2` 在本机（VMware 虚拟机 + Mesa 软件渲染）会在机器人带传感器生成时段错误，崩溃点在 `driCreateNewScreen3`：

```bash
ros2 launch race_bringup sim_ros2_control.launch.py headless:=true stress:=false render_engine:=ogre
```

**必须设置 `GZ_SIM_SYSTEM_PLUGIN_PATH`**，否则 gz sim 找不到 `gz_ros2_control-system`，世界加载失败：

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
```

**`ros_gz_sim create` 的默认参数会覆盖 SDF 里的 `<pose>`。** 模型位姿要通过命令行传入，写在 SDF 里不生效：

```bash
ros2 run ros_gz_sim create -file cam.sdf -name topcam \
  -x 3.65 -y 1.0 -z 13.0 -P 1.5708      # -P 是俯仰角，+90° 表示朝下
```

**相机在 stress 世界里不出图**，nominal 世界正常。需要录制时用 `stress:=false`。

**俯视相机参数**：位于场地中心上方 13 m、俯仰角 +90°、水平视场 1.396 rad，可完整覆盖 14.7 m 见方的比赛场地。
