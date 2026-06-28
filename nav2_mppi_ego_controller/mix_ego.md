# MPPI 使用 Ego-Planner Trajectory 判定方案

目标：`EgoTrajectoryCritic` 不再订阅 Ego-Planner 的可视化 `nav_msgs/msg/Path`，而是订阅 Ego-Planner 从内部 `planned_traj` 发布出来的参考轨迹消息。该 critic 只给 MPPI 候选轨迹打分，不直接输出 `cmd_vel`。

## 数据接口

新增消息包：`ego_planner_msgs`

消息：

```text
ego_planner_msgs/msg/TrajectoryPoint.msg
float32 x
float32 y
float32 z
float32 yaw
float32 velocity

ego_planner_msgs/msg/Trajectory.msg
std_msgs/Header header
float32 time_step
TrajectoryPoint[] points
```

Ego-Planner 发布：

- topic: `/ego_reference_trajectory`
- type: `ego_planner_msgs/msg/Trajectory`
- 来源：`PlannerInterface::getLocalPlanTrajResults(planned_traj)`
- `velocity`：按 `reference_speed` 对 Ego 几何轨迹做弧长重采样后写入，末端按 `terminal_slowdown_distance` 渐降到 0
- `yaw`：按重采样段方向计算
- `time_step`：由 `reference_time_step` 参数写入，默认 `0.1`

原有 `visual_local_trajectory` 仍保留为 RViz 可视化 Path，但 MPPI critic 不再用它判定。

## Critic 打分逻辑

`EgoTrajectoryCritic` 订阅 `/ego_reference_trajectory`，缓存最近一条轨迹并用 mutex 保护共享数据。

如果 `enabled=false`、轨迹点数不足、轨迹超过 `max_reference_age`，critic 直接 return，不改变原 MPPI 行为。

每个 MPPI 预测步 `i` 通过预测时间比例匹配 Ego 参考点：

```text
ref_offset = round(i * mppi_model_dt / ego_trajectory.time_step)
ref_idx = min(start_idx + ref_offset, ego_trajectory.size - 1)
```

其中 `start_idx` 是离当前机器人位置最近的 Ego 轨迹点。

单步代价：

```text
cost =
    position_weight * hypot(x_candidate - x_ref, y_candidate - y_ref)
  + yaw_weight      * abs(shortest_angular_distance(yaw_candidate, yaw_ref))
  + velocity_weight * abs(v_candidate - v_ref)
  + velocity_direction_weight
      * abs(shortest_angular_distance(velocity_heading_candidate, yaw_ref))
```

`velocity_heading_candidate` 会把 MPPI 的车体系 `vx/vy` 用候选点 yaw 转到 map/world 系后计算：

```text
velocity_x = vx * cos(yaw_candidate) - vy * sin(yaw_candidate)
velocity_y = vx * sin(yaw_candidate) + vy * cos(yaw_candidate)
velocity_heading_candidate = atan2(velocity_y, velocity_x)
```

当候选速度低于 `velocity_direction_min_speed` 时，不计算速度方向项，避免静止附近的方向噪声；此时速度大小项仍然会惩罚跟 `v_ref` 的差异。

如果候选点和参考点距离超过 `max_match_distance`，额外加入：

```text
distance_penalty_weight * (position_error - max_match_distance)
```

最终对采样步求平均后乘 `cost_weight` 和 `cost_power` 写入 `data.costs`。

## YAML 示例

```yaml
critics:
  - ObstaclesCritic
  - EgoTrajectoryCritic
  - GoalCritic
  - PathAlignCritic
  - PathFollowCritic
  - TwirlingCritic

EgoTrajectoryCritic:
  enabled: true
  cost_power: 1
  cost_weight: 12.0

  ego_trajectory_topic: "/ego_reference_trajectory"

  position_weight: 1.0
  yaw_weight: 0.01
  velocity_weight: 0.4
  velocity_direction_weight: 0.3
  velocity_direction_min_speed: 0.05

  use_curvature_cost: true
  curvature_weight: 0.15
  max_reference_curvature: 5.0
  max_candidate_curvature: 8.0

  use_acceleration_cost: false
  acceleration_weight: 0.1
  max_reference_acceleration: 5.0
  max_candidate_acceleration: 8.0

  velocity_frame: "base"

  max_reference_age: 0.5
  lookahead_time: 1.2
  threshold_to_consider: 0.7
  max_match_distance: 1.0
  reference_dt: 0.1
  max_reference_speed: 3.0
  trajectory_point_step: 1
  distance_penalty_weight: 10.0

  debug_enabled: false
  debug_log_period: 1.0
  debug_sample_candidates: 3
  debug_check_trajectory_on_callback: true
  debug_check_score_runtime: true
  debug_max_point_gap: 1.0
  debug_min_point_gap: 0.001
  debug_max_yaw_jump: 1.57
  debug_max_speed_jump: 2.0
  debug_max_curvature: 8.0
  debug_max_acceleration: 10.0
  debug_max_cost: 1000000.0
  debug_large_error_distance: 2.0
```

对全向底盘，`yaw_weight` 通常应低一些，避免强迫车体朝向贴死参考；`velocity_direction_weight`
可以相对更高，让 MPPI 更重视沿 Ego-Planner / B-spline 的切线速度方向行进。新增曲率约束利用
B-spline 的平滑性引导候选轨迹保持连续转弯；加速度约束默认关闭，适合作为弱约束逐步加权。

Ego-Planner 节点参数：

```yaml
ego_trajectory_topic: "/ego_reference_trajectory"
local_trajectory_topic: "visual_local_trajectory"
max_vel: 2.5
max_acc: 3.0
max_jerk: 4.0
path_sample_interval: 0.4
reference_speed: 2.0
reference_time_step: 0.1
terminal_slowdown_distance: 0.8
```

## 地图输入与越界诊断

Ego-Planner 当前订阅 Nav2 发布的 `nav_msgs/msg/OccupancyGrid`，默认话题：

```yaml
map_topic: "global_costmap/costmap"
input_global_path_topic: "ego_planner/input_path"
```

`nav2_mppi_ego_controller` 在 `setPlan()` 收到 Nav2 全局路径后，会把该路径转发到 `ego_planner/input_path`，供 Ego-Planner 生成内部局部轨迹。

Ego-Planner 接收地图后会按节流日志输出当前栅格地图边界：

```text
收到 OccupancyGrid 地图: width x height, resolution=..., origin=(...), bounds_x=[min,max), bounds_y=[min,max), frame=..., topic=...
```

接收路径后会检查路径点是否落在当前 OccupancyGrid 边界内。如果看到：

```text
Ego 全局参考路径有 N/M 个点超出当前 OccupancyGrid 边界
```

说明传给 Ego-Planner 的路径点已经超出当前地图覆盖范围。常见原因是 `global_costmap` 使用 rolling window，或者路径 frame 和地图 frame 不一致。当前实现只做检查和告警，不做 TF 转换；若日志显示 `path_frame != map_frame`，需要先修正输入话题或增加显式 TF 转换。

栅格坐标转换使用 OccupancyGrid 语义：

```text
grid_x = floor((world_x - origin_x) / resolution)
grid_y = floor((world_y - origin_y) / resolution)
```

地图外查询仍然会被 Ego-Planner 视为占用，但底层日志已改为汇总式输出，避免正常边界膨胀过程刷屏。

## 速度调参建议

这套方案里，速度不是由单个参数决定的，而是由四层共同决定：

1. Ego-Planner 发布的时间参考轨迹告诉 MPPI “希望多快、多远之后到哪里”。
2. MPPI 在 `vx_max/vy_max/wz_max` 和采样噪声范围内生成候选控制序列。
3. MPPI critics 根据 Ego 参考、路径进度、目标、障碍物、约束等代价选择低 cost 的控制序列。
4. velocity smoother 和底盘速度变换在 MPPI 输出后再做加速度、死区、坐标变换和叠加自旋。

因此，`vx_max` 只是可选速度上限，不是目标速度。小车实际慢时，要先判断慢发生在哪一层。

### 推荐调参顺序

第一步先看 `/ego_reference_trajectory`：

```bash
ros2 topic echo /ego_reference_trajectory --once
```

如果 `points[].velocity` 仍主要是 `0.3~0.5`，优先调 Ego 参考轨迹参数：

```yaml
# src/ego_planner/config/ego_planner.yaml
reference_speed: 2.0
reference_time_step: 0.1
terminal_slowdown_distance: 0.8
max_vel: 2.5
path_sample_interval: 0.4
```

- `reference_speed` 是当前最直接的期望巡航速度。
- `reference_time_step` 是 Ego 参考消息中相邻点的时间间隔，默认应和 MPPI `model_dt=0.05` 保持同一量级；`0.1` 表示 critic 每两个 MPPI 预测步推进一个 Ego 参考点。
- `reference_speed * reference_time_step` 是发给 MPPI 的参考点弧长间距，默认 `2.0 * 0.1 = 0.2m`。
- `terminal_slowdown_distance` 只影响局部参考末端减速。它过大时，短局部轨迹大部分都会处于降速段，车会变慢；想让车更敢跑可降到 `0.4~0.6`。
- `max_vel` 是 Ego 发布参考速度的上限，必须大于等于 `reference_speed`。
- `path_sample_interval` 影响送入 Ego B-spline 的几何路径点距。它不是最终速度，但过小会让内部 B-spline 时间分配偏慢，过大又会降低绕障轨迹细节。当前建议 `0.3~0.5`。

第二步看 `cmd_vel_controller`：

```bash
ros2 topic echo /<namespace>/cmd_vel_controller
```

如果 Ego 参考已经是 `2.0` 左右，但 `cmd_vel_controller` 仍只有 `0.3~0.5`，说明 MPPI critics 或 costmap 在压速度。按下面顺序调：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      model_dt: 0.05
      time_steps: 40
      vx_max: 2.5
      vx_min: -2.0
      vy_max: 2.0
      wz_max: 2.0
      vx_std: 0.8
      vy_std: 0.8
      wz_std: 0.8
      temperature: 0.3
      gamma: 0.07
```

- `vx_max/vy_max/wz_max` 必须覆盖期望速度。若 `reference_speed=2.0`，建议 `vx_max >= 2.3`。
- `vx_std/vy_std/wz_std` 决定采样探索范围。上限调高但 std 很小，MPPI 可能很少采到高速候选。高速配置一般保持 `0.7~1.0`。
- `temperature` 越低越偏向最低 cost 候选，越高越接近平均控制。太高会把快慢候选平均掉，输出变钝；太低可能抖。
- `gamma` 是控制能量平滑项权重。过大时会偏向小控制量，速度上不去；过小会更激进。
- `time_steps * model_dt` 是预测时域。当前 `40 * 0.05 = 2s`。速度越高，预测距离越长，local costmap 必须覆盖这段距离。

第三步调 EgoTrajectoryCritic 与其他 critic 的相对权重：

```yaml
EgoTrajectoryCritic:
  cost_weight: 12.0
  position_weight: 1.0
  velocity_weight: 0.4
  velocity_direction_weight: 0.3
  max_match_distance: 1.0
  distance_penalty_weight: 10.0
  lookahead_time: 1.2
  threshold_to_consider: 0.7

PathFollowCritic:
  cost_weight: 5.0
  offset_from_furthest: 16

ObstaclesCritic:
  repulsion_weight: 0.5
  critical_weight: 20.0
  collision_margin_distance: 0.08
  inflation_radius: 0.7
```

- 如果 MPPI 不跟 Ego 速度，适当提高 `EgoTrajectoryCritic.velocity_weight`，例如 `0.5 -> 0.8`。
- 如果 MPPI 为了贴 Ego 轨迹而绕障不自然，降低 `EgoTrajectoryCritic.cost_weight` 或 `position_weight`。
- `max_match_distance` 太小会让参考轨迹稍有偏差就触发重罚，MPPI 可能宁愿低速保守；高速时可试 `1.0~1.5`。
- `PathFollowCritic.offset_from_furthest` 越大，越鼓励预测末端往前追路径，通常会更敢往前走；但过大会在弯道和障碍附近变激进。
- `ObstaclesCritic` 和 inflation 参数过保守时，高速候选更容易被障碍代价打掉，表现就是 Ego 参考很快但 MPPI 输出慢。

第四步看 `cmd_vel`：

```bash
ros2 topic echo /<namespace>/cmd_vel
```

如果 `cmd_vel_controller` 已经快，但 `cmd_vel` 慢，问题在后处理：

```yaml
velocity_smoother:
  smoothing_frequency: 20.0
  feedback: "OPEN_LOOP"
  max_velocity: [2.5, 2.5, 3.0]
  min_velocity: [-2.5, -2.5, -3.0]
  max_accel: [5.0, 5.0, 6.0]
  max_decel: [-8.0, -8.0, -10.0]
  deadband_velocity: [0.05, 0.05, 0.05]
```

- `max_velocity/min_velocity` 是后处理速度上限，要覆盖 MPPI 输出。
- `max_accel` 太小会导致起速慢，看起来像 MPPI 不肯跑。
- `max_decel` 绝对值太小会导致停车拖。
- `deadband_velocity` 只应切掉极小速度，设太大会吞掉低速微调。

`fake_vel_transform` 只做速度坐标变换，并把 `cmd_spin` 叠加到 `angular.z`。它不会降低线速度模长；如果 `cmd_vel_controller` 和 `cmd_vel` 线速度大小接近，慢不在这里。

### costmap 对速度的影响

MPPI 控制器打分使用 controller server 持有的局部 costmap。当前局部代价地图典型配置是 rolling window：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 20.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: gimbal_yaw_fake
      rolling_window: true
      width: 10
      height: 10
      resolution: 0.05
```

高速时必须保证局部 costmap 覆盖 MPPI 预测距离。经验公式：

```text
local_costmap_half_width > reference_speed * time_steps * model_dt + braking_margin
```

当前 `2.0m/s * 40 * 0.05 = 4.0m`，`width=10m` 的半宽是 `5m`，刚好够用。如果要跑 `2.5m/s`，预测距离是 `5m`，建议把局部地图加到 `12m` 或缩短预测时域，否则高速候选容易跑出有效地图或撞上未知区域，被 obstacle/cost critic 压掉。

global costmap 主要影响全局路径和 Ego-Planner 的地图输入。当前 Ego-Planner 默认订阅：

```yaml
map_topic: "global_costmap/costmap"
```

如果 global costmap 是 rolling window，长全局路径末端经常会超出 Ego 接收到的 OccupancyGrid 边界，Ego 规划会更保守或失败。高速调参时要关注这些日志：

```text
Ego 全局参考路径有 N/M 个点超出当前 OccupancyGrid 边界
Ego 全局参考路径 frame 与 OccupancyGrid frame 不一致
Ego-Planner 本次规划未生成 trajectory
```

若这些日志频繁出现，优先修 map topic、frame、costmap 尺寸，再调速度。否则速度参数调高也会被不可行轨迹和障碍代价压回低速。

### 典型配置

保守稳定：

```yaml
reference_speed: 1.2
max_vel: 1.5
path_sample_interval: 0.3
```

```yaml
vx_max: 1.6
vy_max: 1.2
vx_std: 0.5
vy_std: 0.5
EgoTrajectoryCritic.velocity_weight: 0.4
```

常规快速：

```yaml
reference_speed: 2.0
max_vel: 2.5
path_sample_interval: 0.4
```

```yaml
vx_max: 2.5
vy_max: 2.0
vx_std: 0.8
vy_std: 0.8
max_accel: [5.0, 5.0, 6.0]
```

激进高速：

```yaml
reference_speed: 2.4
max_vel: 2.8
terminal_slowdown_distance: 0.5
```

```yaml
vx_max: 2.8
vy_max: 2.3
vx_std: 1.0
vy_std: 1.0
local_costmap.width: 12
local_costmap.height: 12
max_accel: [6.0, 6.0, 7.0]
```

激进配置必须同时检查障碍物膨胀、局部地图覆盖范围和实际底盘加速度能力。只提高参考速度和 `vx_max`，不扩大局部地图或加速度，通常只会让 MPPI 在 cost 上继续选择低速候选。

### 快速定位表

| 现象 | 优先检查 | 典型处理 |
|---|---|---|
| `/ego_reference_trajectory` 的 `velocity` 只有 `0.3~0.5` | `reference_speed`、`max_vel`、Ego 参数文件是否生效 | 提高 `reference_speed`，确认 `ego_params_file` 指向的 YAML 已更新并完全重启 |
| Ego 参考速度正常，但 `cmd_vel_controller` 慢 | MPPI critic、local costmap、障碍物代价 | 调 `velocity_weight`、`vx_std`、costmap 尺寸和 obstacle 权重 |
| `cmd_vel_controller` 快，`cmd_vel` 慢 | velocity smoother | 调 `max_velocity/max_accel/max_decel/deadband` |
| 直线快、弯道慢 | `velocity_direction_weight`、`PathAlignCritic`、`wz_max` | 适当降低方向约束或提高角速度能力 |
| 靠近目标过早变慢 | `terminal_slowdown_distance`、GoalCritic 阈值 | 减小 `terminal_slowdown_distance` 或调整 goal critic 触发距离 |
| 有障碍物时明显慢 | `ObstaclesCritic`、inflation、local costmap | 降低过强 repulsion，确认地图没有虚假障碍 |

## 短路径保护

Nav2 在接近目标或恢复后可能下发很短的局部全局路径，例如只有几个 pose，且相邻 pose 间距小于 `0.1m`。Ego-Planner 原离散化逻辑会丢弃所有短线段终点，导致整条路径被压成 1 个点；随后 B-spline 参数化会打印：

```text
[B-spline]:point set have only 1 points.
```

并触发 Eigen 断言，使 `motion_plan` 进程 abort，表现为 `/ego_reference_trajectory` 的 publisher 从 ROS 图中消失。

当前修正：

- 轨迹离散化使用 `ceil(segment_length / interval)`，短于 interval 的线段也保留终点。
- `PlannerInterface::makePlan()` 在参考路径少于 4 点时直接跳过 Ego B-spline 规划并清空旧结果，不再进入 Eigen 断言。
- 如果短路径发生在接近目标阶段，Ego critic 会因为收到空参考轨迹而自动跳过，MPPI 仍由 Goal/Path/Obstacle critics 工作。

## 对比原仓库删除项

- 删除 planner/matplotlib-cpp
- 删除旧 ROS1 生成头目录 planner/include
- 移除旧 Bspline.h 依赖并简化 PlannerInterface::getTraj()
- 删除未接线的 PointCloud2 手动障碍物入口
- 删除 add_obstacle_at_position() 死代码
- 删除未使用 include 和 theta_
- 删除旧的空 /visual_obstacles 手动障碍物发布链路，保留 /visual_local_obstacles

## 第一版限制

- `Trajectory` 消息暂不发布 acceleration / jerk。
- 参考点匹配按时间步比例推进，不做复杂最近点搜索。
- `yaw` 由重采样轨迹段方向估算，不是 Ego-Planner 优化变量。
- `time_step` 已由 `reference_time_step` 参数透出。
- `Trajectory` 当前只包含 MPPI critic 所需的 position、yaw、velocity。

## 后续升级

如果后续 Ego-Planner 暴露完整 B-spline 轨迹，可把 `TrajectoryPoint` 扩展为包含 `time_from_start`、`acceleration`、`jerk` 或直接新增 B-spline 控制点消息。MPPI critic 层只需要把 `buildTrajectory()` 的输入转换逻辑替换掉，打分函数可以保持不变。
