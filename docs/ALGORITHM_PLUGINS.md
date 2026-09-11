# 算法插件库 / Algorithm Plugin Library

在 `race_ros2_ws` 场景中并排运行多种规划算法，**换算法只改一个索引**，并且同一套算法在仿真和实物上无需重新编译。

---

## 1. 三层架构

```
algo_core/            纯算法库，不依赖 ROS。可离线单元测试，也可脱离 Nav2 使用
      ↑
algo_nav2_plugins/    Nav2 适配层：把每个算法暴露成 nav2_core::GlobalPlanner 插件
      ↑
algo_bringup/         选择层：算法索引、参数生成、行为树生成、启动文件
```

分三层的理由是**算法本体和 ROS 解耦**：`algo_core` 里没有任何 `rclcpp`、`costmap_2d`、参数框架的痕迹，所以算法可以直接在单元测试里跑（`algo_core_selftest`），也能被非 Nav2 的代码复用。移植一个算法只需要写算法本身，不需要碰 ROS。

## 2. 换算法：改一个数字

唯一需要动的地方是 `algo_bringup/config/algo_registry.yaml` 的 `active.planner`：

```yaml
active:
  planner: 1        # >>> 换算法只需要改这个数字 <<<
```

启动后终端会打印当前生效的算法：

```
==============================================================================
  ALGORITHM INDEX
==============================================================================
    [ 0] P0_dijkstra            dijkstra
 -> [ 1] P1_astar               astar
    ...
==============================================================================
  active planner: [1] P1_astar   (change active.planner in algo_registry.yaml)
==============================================================================
```

也可以用启动参数临时覆盖，不改文件：

```bash
ros2 launch algo_bringup algo_planner_bench.launch.py planner_index:=4
```

## 3. 多接口 / 不同情况用不同算法

这是本库和"写死一个规划器"的关键区别。同一批算法同时注册进 Nav2，然后有**三条**选择路径：

### 3.1 每次请求指定（`planner_id`）

Nav2 的 `ComputePathToPose` 动作带 `planner_id` 字段。所有算法都处于加载状态，所以可以在**运行中**为每个目标点选不同算法，不需要重启：

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{start: {...}, goal: {...}, planner_id: 'P4_jps', use_start: true}"
```

已验证：同一张 race 地图上，8 个 planner_id 各自独立工作。

### 3.2 按情况自动切换（行为树）

`NavigateToPose` 没有 `planner_id` 字段，但**有 `behavior_tree` 字段**。所以 `algo_bringup` 为每个算法生成一棵行为树，里面把 `planner_id` 写死：

```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="P4_jps"/>
```

任务节点只要把对应文件名填进 goal 的 `behavior_tree` 就能换算法。生成规则在

```yaml
selection:
  by_state:                          # 按状态机状态
    PREPARING_MAP_SEARCH: "P1_astar"
    NAVIGATING_TO_SEARCH_VIEWPOINT: "P1_astar"
  stress_world: "P6_d_star_lite"     # 压力世界（移动障碍）→ 增量重规划
  long_range_m: 8.0                  # 超过这个距离 → 用更贪心的搜索
  long_range_planner: "P2_weighted_astar"
  narrow_passage_planner: "P5_theta_star"   # 窄通道 → 任意角
```

### 3.3 库接口（脱离 Nav2）

`algo_core` 本身可以不经 ROS 使用：

```cpp
#include "algo_core/registry.hpp"

auto planner = algo_core::Registry::instance().create("theta_star");
planner->configure(params);
algo_core::PlanResult r = planner->plan(grid, start, goal);
```

`algo_core_selftest` 就是这么做的——它不启动任何 ROS 节点。

## 4. 仿真里怎么用

### 独立对标（不跑整个比赛场景）

```bash
ros2 launch algo_bringup algo_planner_bench.launch.py
```

只拉起 `map_server` + `planner_server` + 静态 TF，加载 race 地图。用来快速横评算法，不用等 Gazebo。

### 接入完整比赛流程

`algo_bringup` 生成的 `planner_server` 参数块可以直接替换
`race_navigation/config/nav2_params.yaml` 里的 `planner_server:` 段，`competition.launch.py`
不用改。原基线（Nav2 自带 Theta\*）以一个 `external: true` 条目（`P7_nav2_theta_star`）保留在同一张表里，因此新旧算法是同参数、同地图、同指标对比。

## 5. 实物上怎么用

**接口层零改动。** 插件输出标准的 `geometry_msgs/Twist`（含 `linear.x/y`），走的是现有链路：

```
插件 → /cmd_vel → Twist → TwistStamped → omni_drive_controller → 三个轮子
```

仿真里跑通的 `.so`，拷到车上就是同一个文件。仿真与实物的差别只在 costmap 数据来源（Gazebo 激光 vs 真实激光）和标定参数，与算法本身无关。

实车上需要注意的三点：

1. **横向速度要放开**：`controller_server` 的 `min_y_velocity_threshold` 以及控制器自身都要允许 `vy`，否则全向底盘不会横移。
2. **算力筛选**：本库目前的算法都是栅格搜索，实测规划耗时在毫秒级，可以放心上实车。后续接入采样类（RRT\*）和演化类（ACO/GA/PSO）时必须先测规划耗时——那些算法在实车上很可能跟不上重规划频率，应划为离线/静态地图规划。
3. **代价地图阈值**：`lethal_threshold` 默认 253（与 NavFn 一致），即把内切膨胀层也当障碍。实车传感器噪声更大，需要重新标定膨胀半径，否则窄通道会判为不可通行。

## 6. 怎么加一个新算法

三步，不需要动任何已有文件：

1. 在 `algo_core/src/` 下实现一个类，继承 `algo_core::GridPlanner`：

```cpp
class MyPlanner final : public algo_core::GridPlanner {
public:
  std::string name() const override { return "my_planner"; }
  algo_core::PlanResult plan(const algo_core::CostGrid & grid,
                             const algo_core::Pose2D & start,
                             const algo_core::Pose2D & goal) override;
};
```

2. 在文件末尾自注册，并加进 `algo_core/CMakeLists.txt`：

```cpp
ALGO_CORE_REGISTER(algo_core::MyPlanner, "my_planner")
```

3. 在 `algo_registry.yaml` 加一条：

```yaml
  - index: 8
    id: "P8_my_planner"
    algorithm: "my_planner"
    description: "..."
    params: {cost_scale: 1.0}
```

启动后会看到 `algorithm 'my_planner' ready`，并且 `planner_id: P8_my_planner` 立即可用。
**插件类不用改，行为树自动生成。**

## 7. 当前算法清单与验证状态

在 race 地图（294×294 @ 0.05 m）上从 `(-3.0, -5.0)` 规划到 `(10.0, 7.0)` 的实测结果：

| 索引 | 算法 | 结果 | 路径点数 |
| :--- | :--- | :--- | :--- |
| 0 | Dijkstra | ✅ | 21 |
| 1 | A\*（默认） | ✅ | 23 |
| 2 | Weighted A\* | ✅ | 29 |
| 3 | GBFS | ✅ | 22 |
| 4 | JPS | ❌ **已知缺陷** | — |
| 5 | Theta\* | ✅ | 8 |
| 6 | D\* Lite | ✅ | 19 |
| 7 | Nav2 自带 Theta\*（对照基线） | ✅ | 394 |

Theta\* 只用 8 个点走完全程（对比 A\* 的 23 个），这就是任意角规划消除栅格阶梯的直接体现。

> **JPS 已知缺陷**：剪枝搜索在 race 地图上返回 `NO_VALID_PATH`，而同样代价模型的 A\* 能找到路径，说明 jump 或邻居剪枝逻辑有错。它在合成地图的自测里是通过的，所以直到接上真实地图才暴露。**修好前不要用它出结果。** 复现方式见 `algo_registry.yaml` 里 `P4_jps` 的 `notes`。

## 8. 测试

不依赖 ROS 的算法自测：

```bash
colcon test --packages-select algo_core
# 或直接跑
./install/algo_core/lib/algo_core/algo_core_selftest
```

它检查每个注册算法的四条不变量：能找到路径、路径连接起终点、路径不穿障碍、以及 D\* Lite 在代价地图变化后能正确增量修复。
