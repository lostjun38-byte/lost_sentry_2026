# Ego Planner 2D for ROS 2 Humble / Nav2

本包是一个用于 Nav2 集成的 2D Ego-Planner 参考轨迹生成节点。当前版本的定位是：

```text
Nav2 全局路径驱动 + 当前 Ego 轨迹短窗口阻断触发的局部动态重规划器
```

它不再把 rolling costmap 的每一帧更新都当作重规划请求。costmap 只用于更新 GridMap2D 缓存和检查当前 Ego 轨迹是否真的被动态障碍阻断；只有收到新的 Nav2 路径，或当前轨迹未来短窗口发生碰撞时，才触发 Ego 重规划。

## 设计目标

- 接收 Nav2 MPPI controller 发布的全局/局部参考路径：`ego_planner/input_path`
- 生成平滑的 2D B-spline 局部参考轨迹
- 发布给 MPPI `EgoTrajectoryCritic` 使用：`/ego_reference_trajectory`
- 处理动态障碍物：只在当前 Ego 轨迹未来短窗口被 costmap 阻断时触发局部重规划
- 避免 costmap 高频更新导致 Ego Planner 高频进入 A* / rebound 优化
- 规划失败时短时保留旧安全轨迹，必要时发布停车轨迹，避免 `/ego_reference_trajectory` 直接断流

## 节点与话题

默认节点：

```text
package: ego_planner
executable: motion_plan
node name: ego_planner
```

主要输入：

| 话题 | 类型 | 说明 |
|---|---|---|
| `ego_planner/input_path` | `nav_msgs/msg/Path` | Nav2 MPPI controller 传入的参考路径。收到新路径才触发初始规划。 |
| `global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 costmap。只更新 GridMap2D 缓存，不直接触发重规划。 |
| `trigger_plan` | `std_msgs/msg/Bool` | 手动启停。`false` 会清空 active goal 和旧轨迹缓存。 |
| `goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz 简单直线路径测试入口。 |
| `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 手动更新当前位姿。 |

主要输出：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/ego_reference_trajectory` | `ego_planner_msgs/msg/Trajectory` | 给 MPPI `EgoTrajectoryCritic` 的时间参数化参考轨迹。 |
| `visual_local_trajectory` | `nav_msgs/msg/Path` | Ego 局部轨迹可视化。 |
| `visual_global_path` | `nav_msgs/msg/Path` | 当前输入路径可视化。 |
| `trajectories` | `visualization_msgs/msg/MarkerArray` | A* rebound 路径可视化。 |
| `visual_local_obstacles` | `sensor_msgs/msg/PointCloud2` | GridMap2D 当前障碍物点云可视化。 |

## 当前重规划状态机

核心逻辑在 `src/trajectory_publisher.cpp`。

```text
收到新的 ego_planner/input_path
  -> 保存 global path
  -> should_plan = true
  -> needs_replan = true
  -> 执行一次初始 Ego 规划

costmap 更新
  -> 只更新 GridMap2D / costmap 元数据
  -> costmap_updated = true
  -> 不直接 needs_replan

timer 主循环
  -> 如果 input_path 超时，清空 active goal 和轨迹缓存
  -> 如果 needs_replan，尝试 makePlan
  -> 如果 costmap_updated，检查当前 Ego 轨迹未来短窗口是否被阻断
  -> 若阻断且 replan_cooldown 到期，触发动态重规划
  -> 发布当前有效 Ego 轨迹；失败保底时发布旧安全轨迹或停车轨迹
```

关键约束：

- costmap 更新不是重规划请求。
- 动态障碍触发条件是“当前 Ego 轨迹短窗口被阻断”。
- blocked-check 只做轻量 occupancy 查询，不做 A*、优化、全局搜索。
- rolling costmap 边界外的点在 blocked-check 阶段不会直接判定为 blocked，避免地图边界误触发。
- makePlan 成功后才替换当前发布轨迹。
- makePlan 失败不会立刻清空 `/ego_reference_trajectory`。

## 运行

单独运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ego_planner ego_planner.launch.py use_sim_time:=true
```

指定自定义参数文件：

```bash
ros2 launch ego_planner ego_planner.launch.py params_file:=/path/to/ego_planner.yaml use_sim_time:=true
```

在本工作空间的仿真导航 launch 中通常由 `pb2025_nav_bringup` include：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py use_ego_planner:=true
```

仿真导航 launch 也可以直接切换 Ego 参数文件：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py use_ego_planner:=true ego_params_file:=/path/to/ego_planner.yaml
```

编译：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ego_planner pb2025_nav_bringup
source install/setup.bash
```

## 关键参数

参数默认值主要在 `config/ego_planner.yaml` 中维护；`include/trajectory_obstacles_publisher.h` 中保留同一套 C++ 兜底默认值。`launch/ego_planner.launch.py` 只保留 `namespace`、`params_file` 和 `use_sim_time` 这类启动入口。

| 参数 | 默认值 | 建议范围 | 说明 |
|---|---:|---:|---|
| `local_planning_horizon` | `5.0` | `4.0 ~ 8.0` | Ego 每次沿输入路径截取的局部规划长度。不要明显超过 rolling costmap 有效窗口。 |
| `input_path_timeout` | `3.0` | `2.0 ~ 4.0` | 多久没有新的 `ego_planner/input_path` 后认为 Nav2 goal 不活跃，并清空旧路径/旧轨迹。 |
| `replan_cooldown` | `0.2` | `0.15 ~ 0.5` | 动态障碍触发重规划的冷却时间。`0.2s` 约等于最多 5Hz。 |
| `trajectory_collision_check_time` | `1.5` | `1.0 ~ 2.0` | 只检查当前 Ego 轨迹未来多少秒。 |
| `trajectory_collision_check_dt` | `0.1` | `0.05 ~ 0.2` | blocked-check 采样时间步长。 |
| `trajectory_collision_check_distance` | `3.0` | `2.0 ~ 4.0` | 只检查当前 Ego 轨迹前方多少米。 |
| `fallback_trajectory_max_age` | `0.5` | `0.3 ~ 1.0` | makePlan 失败时，旧轨迹最多可继续沿用多久。 |
| `make_plan_warn_time_ms` | `100.0` | `50 ~ 200` | makePlan 超过该耗时会打印警告。只用于观测，不中断规划。 |
| `path_sample_interval` | `0.4` | `0.2 ~ 0.6` | 输入路径重采样间距。越小控制点越多，规划更重。 |
| `reference_speed` | `2.0` | 按机器人能力 | 发布给 MPPI 的参考速度。会被 `max_vel` 限制。 |
| `reference_time_step` | `0.1` | `0.05 ~ 0.2` | `/ego_reference_trajectory` 时间步长。 |
| `terminal_slowdown_distance` | `0.8` | `0.5 ~ 1.5` | 接近 Ego 局部轨迹末端时的减速距离。 |
| `max_vel` | `2.5` | 按机器人能力 | Ego 参考轨迹速度上限。 |
| `max_acc` | `3.0` | 按机器人能力 | Ego 参考轨迹加速度上限。 |
| `max_jerk` | `4.0` | 按机器人能力 | Ego 参考轨迹 jerk 上限。 |

## 调参建议

### 1. 先确认 rolling costmap 尺寸

如果 costmap 大约是：

```text
432 x 300, resolution = 0.05
=> 21.6m x 15.0m
```

`local_planning_horizon` 不建议超过 `5 ~ 8m`。过大时，B-spline 或碰撞采样容易落到 rolling costmap 边界外；Ego 会把地图外查询当成占用，从而进入 A* / rebound 优化，CPU 会明显上升。

推荐起点：

```text
local_planning_horizon = 5.0
trajectory_collision_check_distance = 3.0
trajectory_collision_check_time = 1.5
```

### 2. 动态障碍反应慢

优先调这些参数：

```text
replan_cooldown: 0.2 -> 0.15
trajectory_collision_check_time: 1.5 -> 2.0
trajectory_collision_check_distance: 3.0 -> 4.0
```

注意：

- `replan_cooldown` 太小会让 Ego 更频繁进入 makePlan，CPU 会升高。
- `trajectory_collision_check_distance` 太大容易靠近 rolling costmap 边界，误触发风险增加。
- 先改一个参数，观察 CPU、makePlan 耗时和 `/ego_reference_trajectory` age。

### 3. CPU 高或偶发卡死

优先降低重规划负载：

```text
local_planning_horizon: 5.0 或更低
trajectory_collision_check_distance: 2.0 ~ 3.0
replan_cooldown: 0.3 ~ 0.5
path_sample_interval: 0.4 ~ 0.6
```

观察日志：

```text
Ego makePlan 耗时偏高
当前 Ego 轨迹短窗口被阻断
Ego makePlan 失败，但旧轨迹仍在短时有效期内且短窗口安全
发布停车轨迹
```

如果 `makePlan` 经常超过 `100ms`，说明规划问题过重，优先缩短 horizon 或增大 `path_sample_interval`。

### 4. `/ego_reference_trajectory` 仍然 age 超时

先确认是否真的断流：

```bash
ros2 topic hz /ego_reference_trajectory
```

如果频繁断流：

- 检查是否频繁出现 `input_path_timeout`
- 检查 Nav2 MPPI 是否持续发布 `ego_planner/input_path`
- 适当增大 `input_path_timeout` 到 `4.0`
- 适当增大 `fallback_trajectory_max_age` 到 `0.8 ~ 1.0`

不要单纯增大 MPPI `EgoTrajectoryCritic.max_reference_age` 来掩盖断流。`max_reference_age` 可以略放宽，但根因仍应是 Ego 持续发布有效参考轨迹。

### 5. 动态障碍误触发太多

如果障碍物没有挡住当前轨迹也频繁重规划：

```text
trajectory_collision_check_time: 1.5 -> 1.0
trajectory_collision_check_distance: 3.0 -> 2.0
replan_cooldown: 0.2 -> 0.3
```

同时检查 costmap 膨胀半径和 footprint。膨胀过大时，轨迹会更容易被判为阻断。

### 6. 轨迹太短或不够前瞻

如果 MPPI 的参考不够稳定，可以逐步增大：

```text
local_planning_horizon: 5.0 -> 6.0 -> 8.0
trajectory_collision_check_distance: 3.0 -> 4.0
```

不要一次改回 `30.0`。如果需要更长 horizon，应先扩大 rolling costmap 尺寸，并确认地图边界外查询不会成为主要碰撞来源。

## 与 MPPI EgoTrajectoryCritic 的关系

Ego Planner 输出 `/ego_reference_trajectory`，MPPI 的 `EgoTrajectoryCritic` 将该轨迹作为代价参考。当前推荐配置是：

```yaml
EgoTrajectoryCritic:
  enabled: true
  ego_trajectory_topic: "/ego_reference_trajectory"
  max_reference_age: 0.5
  threshold_to_consider: 0.7
```

原则：

- Ego 负责生成局部平滑参考轨迹，并在当前轨迹被动态障碍短窗口阻断时重规划。
- MPPI 仍然负责即时控制和局部采样优化。
- `ObstaclesCritic` 仍应开启，不能只依赖 Ego 处理所有碰撞风险。
- `max_reference_age` 是保护参数，不是解决 Ego 断流的主要手段。

## 常见问题

### costmap 一直更新，会不会一直重规划？

不会。当前版本中，costmap 更新只设置 `costmap_updated_` 并刷新 GridMap2D。只有 `isCurrentTrajectoryBlocked()` 检测到当前 Ego 轨迹未来短窗口被占用，且 `replan_cooldown` 到期，才会触发重规划。

### 没有新的 Nav2 path 时，Ego 会不会继续用旧路径规划？

不会。来自 `ego_planner/input_path` 的路径带有 `input_path_timeout`。超时后会清空：

- `has_valid_global_path_`
- `should_plan_`
- `needs_replan_`
- global path buffer
- 当前 Ego 轨迹缓存
- PlannerInterface 内部 `_plan_traj_results_`

### 规划失败会不会让 `/ego_reference_trajectory` 立刻断流？

不会。makePlan 成功后才替换当前轨迹。makePlan 失败时：

- 旧轨迹仍在 `fallback_trajectory_max_age` 内且短窗口安全：继续发布旧轨迹；
- 旧轨迹过期或已经不安全：发布停车轨迹。

### 为什么 blocked-check 不把 costmap 外点判为 blocked？

rolling costmap 会随着机器人移动而改变边界。轨迹远端落到 costmap 外，不一定代表动态障碍阻断当前短时执行轨迹。如果在 blocked-check 中把地图外点直接判 blocked，会重新引入“rolling costmap 边界驱动重规划”的问题。

## 调试命令

查看输入路径频率：

```bash
ros2 topic hz /<namespace>/ego_planner/input_path
```

查看 Ego 输出频率：

```bash
ros2 topic hz /ego_reference_trajectory
```

查看当前参数：

```bash
ros2 param get /<namespace>/ego_planner local_planning_horizon
ros2 param get /<namespace>/ego_planner replan_cooldown
ros2 param get /<namespace>/ego_planner input_path_timeout
ros2 param get /<namespace>/ego_planner trajectory_collision_check_distance
```

手动停止 Ego active goal：

```bash
ros2 topic pub --once /<namespace>/trigger_plan std_msgs/msg/Bool "{data: false}"
```

## 验收检查

修改后应满足：

- 静止无障碍时，costmap 高频更新不会导致 Ego 高频 makePlan。
- 没有新的 `ego_planner/input_path` 时，Ego 不会用旧路径无限重规划。
- 动态障碍挡住当前 Ego 轨迹未来短窗口时，Ego 按 `replan_cooldown` 触发局部重规划。
- 动态障碍没有挡住当前轨迹时，costmap 更新不会触发重规划。
- `local_planning_horizon` 默认是 `5.0`，不会明显超过 rolling costmap 可用窗口。
- makePlan 失败时，`/ego_reference_trajectory` 短时沿用旧安全轨迹或发布停车轨迹。
- `trigger_plan=false` 或 `input_path_timeout` 后不会继续发布旧轨迹。
- MPPI `EgoTrajectoryCritic` 不应频繁出现 reference trajectory age 超时。

## 参考

本项目基于 ZJU-FAST-Lab Ego-Planner 思路进行 2D 化和 ROS 2 / Nav2 集成改造：

- Original Project: https://github.com/ZJU-FAST-Lab/ego-planner
- 2D ROS2 version: https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2

Last updated: 2026-05-05
