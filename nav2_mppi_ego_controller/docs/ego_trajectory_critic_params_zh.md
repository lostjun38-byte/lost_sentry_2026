# EgoTrajectoryCritic 参数中文说明

本文档说明 `nav2_mppi_ego_controller::critics::EgoTrajectoryCritic` 当前支持的参数。

`EgoTrajectoryCritic` 的作用不是直接执行 Ego-Planner / B-spline 轨迹，而是在 Nav2 MPPI
采样控制中给候选轨迹增加一项参考轨迹代价，引导候选轨迹靠近 Ego-Planner 发布的平滑轨迹。
障碍物、安全约束、动力学约束等 critic 仍然应该拥有更高优先级，不能让本 critic 压过安全约束。

## 推荐配置示例

```yaml
EgoTrajectoryCritic:
  enabled: true
  cost_power: 1
  cost_weight: 8.0

  ego_trajectory_topic: "/ego_reference_trajectory"

  position_weight: 1.0
  yaw_weight: 0.1
  velocity_weight: 0.2
  velocity_direction_weight: 0.4
  velocity_direction_min_speed: 0.05

  use_curvature_cost: true
  curvature_weight: 0.1
  max_reference_curvature: 5.0
  max_candidate_curvature: 8.0

  use_acceleration_cost: false
  acceleration_weight: 0.05
  max_reference_acceleration: 5.0
  max_candidate_acceleration: 8.0

  velocity_frame: "base"

  max_reference_age: 0.5
  lookahead_time: 1.0
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

## 参数总览

| 参数名 | 类型 | 默认值 | 单位 | 作用 |
|---|---:|---:|---:|---|
| `enabled` | bool | `true` | - | 是否启用该 critic。该参数由 MPPI critic 基类读取。 |
| `ego_trajectory_topic` | string | `"/ego_reference_trajectory"` | - | Ego-Planner 参考轨迹订阅话题。 |
| `max_reference_age` | double | `1.0` | s | 参考轨迹最大允许年龄，超时则跳过本 critic。 |
| `lookahead_time` | double | `1.0` | s | 本 critic 在 MPPI 预测时域内参与打分的前瞻时长。 |
| `max_match_distance` | double | `1.0` | m | 候选轨迹点离参考点超过该距离后，额外增加距离惩罚。 |
| `reference_dt` | double | `0.1` | s | 当消息 `time_step <= 0` 或缺少显式时间时使用的参考轨迹时间步长。 |
| `max_reference_speed` | double | `3.0` | m/s | 对参考速度做上限裁剪，防止异常参考速度放大代价。 |
| `trajectory_point_step` | size_t | `1` | step | MPPI 预测轨迹采样间隔，每隔多少个预测点计算一次本 critic。 |
| `cost_power` | uint | `1` | - | 对最终 critic cost 做幂次放大。 |
| `cost_weight` | float | `8.0` | - | 本 critic 的总权重。 |
| `position_weight` | float | `1.0` | - | 位置误差权重。 |
| `yaw_weight` | float | `0.4` | - | 朝向误差权重。 |
| `velocity_weight` | float | `0.2` | - | 速度大小误差权重。 |
| `velocity_direction_weight` | float | `0.3` | - | 速度方向误差权重。 |
| `velocity_direction_min_speed` | float | `0.05` | m/s | 候选速度低于该值时，不计算速度方向误差。 |
| `distance_penalty_weight` | float | `10.0` | - | 超过 `max_match_distance` 后的额外距离惩罚权重。 |
| `use_curvature_cost` | bool | `false` | - | 是否启用曲率误差代价。 |
| `curvature_weight` | float | `0.0` | - | 曲率误差权重。 |
| `max_reference_curvature` | double | `5.0` | 1/m | 参考曲率限幅。 |
| `max_candidate_curvature` | double | `8.0` | 1/m | MPPI 候选轨迹曲率限幅。 |
| `use_acceleration_cost` | bool | `false` | - | 是否启用加速度误差代价。 |
| `acceleration_weight` | float | `0.0` | - | 加速度误差权重。 |
| `max_reference_acceleration` | double | `5.0` | m/s^2 | 参考加速度限幅。 |
| `max_candidate_acceleration` | double | `8.0` | m/s^2 | MPPI 候选轨迹加速度限幅。 |
| `velocity_frame` | string | `"base"` | - | `data.state.vx/vy` 的坐标系语义，可选 `"base"` 或 `"global"`。 |
| `debug_enabled` | bool | `false` | - | 运行时自检总开关。 |
| `debug_log_period` | double | `1.0` | s | debug 日志节流周期。 |
| `debug_sample_candidates` | size_t | `3` | 条 | `score()` 中最多抽查的 MPPI 候选轨迹数量。 |
| `debug_check_trajectory_on_callback` | bool | `true` | - | 收到 Ego 参考轨迹后是否检查轨迹本身。 |
| `debug_check_score_runtime` | bool | `true` | - | `score()` 中是否检查匹配、candidate 和 cost。 |
| `debug_max_point_gap` | double | `1.0` | m | 连续参考点最大合理距离。 |
| `debug_min_point_gap` | double | `0.001` | m | 连续参考点最小合理距离。 |
| `debug_max_yaw_jump` | double | `1.57` | rad | 连续参考 yaw 最大合理跳变。 |
| `debug_max_speed_jump` | double | `2.0` | m/s | 连续参考速度最大合理跳变。 |
| `debug_max_curvature` | double | `8.0` | 1/m | debug 判定参考曲率异常的阈值。 |
| `debug_max_acceleration` | double | `10.0` | m/s^2 | debug 判定参考加速度异常的阈值。 |
| `debug_max_cost` | double | `1000000.0` | - | debug 判定本 critic cost 爆炸的阈值。 |
| `debug_large_error_distance` | double | `2.0` | m | debug 判定匹配距离或候选误差过大的阈值。 |

## 基础开关与订阅参数

### `enabled`

是否启用 `EgoTrajectoryCritic`。

- `true`：正常订阅并参与 MPPI 候选轨迹打分。
- `false`：`score()` 直接返回，不向 `data.costs` 添加代价。

该参数由 `CriticFunction` 基类读取，不在 `EgoTrajectoryCritic::initialize()` 中直接读取。

### `ego_trajectory_topic`

Ego-Planner 参考轨迹的订阅话题。当前代码订阅消息类型为：

```cpp
ego_planner_msgs::msg::Trajectory
```

默认值是 `"/ego_reference_trajectory"`。

如果话题名配置错误，本 critic 不会收到参考轨迹，`score()` 会因为内部参考轨迹为空而直接返回。

## 时间与轨迹有效性参数

### `max_reference_age`

参考轨迹最大允许年龄。`score()` 中会用当前时间减去最近一次收到参考轨迹的时间：

- 如果参考轨迹年龄超过 `max_reference_age`，本 critic 跳过打分。
- 日志使用 `RCLCPP_WARN_THROTTLE` 节流，避免 MPPI 高频循环刷屏。
- 如果设置为 `0.0`，表示不做超时检查。

建议值：

- 轨迹发布频率较高时可设为 `0.3 ~ 0.5`。
- 通信或规划更新较慢时可放宽到 `1.0`。

### `lookahead_time`

本 critic 在 MPPI 预测时域内参与打分的时间长度。

例如：

- MPPI `model_dt = 0.05`
- `lookahead_time = 1.0`

则本 critic 最多打分前约 20 个预测步。

如果设置为 `0.0`，表示使用 MPPI 候选轨迹的全部预测点。

调参建议：

- 过短：只约束当前附近，未来弯道利用不足。
- 过长：可能让本 critic 过早拉向远处参考，和避障 critic 冲突。

### `reference_dt`

参考轨迹默认时间步长。

当 `ego_planner_msgs::msg::Trajectory.time_step <= 0`，或者点级消息没有显式时间字段时，内部参考点时间按以下方式生成：

```cpp
t = index * reference_dt
```

当前 `TrajectoryPoint.msg` 只有：

```text
float32 x
float32 y
float32 z
float32 yaw
float32 velocity
```

因此现阶段主要依赖 `Trajectory.time_step`，若 `time_step` 无效则回退到 `reference_dt`。

### `trajectory_point_step`

本 critic 对 MPPI 候选轨迹的采样间隔。

- `1`：每个 MPPI 预测步都计算一次参考代价，最精细，计算量最大。
- `2`：每两个预测步计算一次，计算量约减半。
- 更大值：更省算力，但参考约束变稀疏。

代码会强制将该值限制为至少 `1`。

## 位置匹配与距离惩罚参数

### `max_match_distance`

候选轨迹点到参考点的位置误差超过该距离后，会额外增加惩罚：

```cpp
match_violation = max(position_error - max_match_distance, 0)
step_cost += distance_penalty_weight * match_violation
```

作用是防止 MPPI 候选轨迹离 Ego 参考轨迹太远。

注意：

- 设置太小会让本 critic 变得很硬，可能和避障行为冲突。
- 高速或局部代价地图较复杂时，可以适当放宽到 `1.0 ~ 1.5`。

### `distance_penalty_weight`

超过 `max_match_distance` 后的额外惩罚权重。

该参数只影响超限部分，不影响正常范围内的位置误差。

调参建议：

- MPPI 经常完全不理 Ego 轨迹：可适当增大。
- MPPI 为了贴 Ego 轨迹而绕障不自然：应降低该值或降低 `cost_weight` / `position_weight`。

## 总权重与代价形状参数

### `cost_weight`

本 critic 的总权重。最终代价形式为：

```cpp
data.costs += pow(cost_weight * average_step_cost, cost_power)
```

它会统一放大或缩小本 critic 对 MPPI 采样结果的影响。

安全建议：

不要把 `cost_weight` 调得过高，以免 Ego 参考轨迹代价压过障碍物、安全边界、动力学约束等 critic。

### `cost_power`

最终代价幂次。

- `1`：线性代价，调参直观。
- `2` 或更高：大误差会被更强惩罚，小误差影响相对变弱。

通常建议保持 `1`，除非明确需要对大偏差做强惩罚。

## 位置、朝向、速度参数

### `position_weight`

位置误差权重。

位置误差计算方式：

```cpp
position_error = hypot(candidate_x - ref.x, candidate_y - ref.y)
```

该参数体现对 Ego-Planner / B-spline 平滑位置参考的跟随程度。

### `yaw_weight`

候选 yaw 和参考 yaw 的误差权重。

角度误差使用 shortest angular distance，避免 `pi` 到 `-pi` 跳变问题。

全向底盘建议：

- `yaw_weight` 不宜过高。
- 全向底盘可以侧向运动，不一定需要车体朝向严格贴合轨迹切线。
- 常见配置可从 `0.0 ~ 0.1` 开始。

### `velocity_weight`

速度大小误差权重。

候选速度大小计算方式：

```cpp
candidate_speed = sqrt(vx^2 + vy^2)
speed_error = abs(candidate_speed - ref.speed)
```

参考速度来自消息字段 `velocity`，并会被 `max_reference_speed` 限幅。

### `max_reference_speed`

参考速度上限。

作用：

- 防止 Ego 参考消息中的异常速度导致 cost 爆炸。
- 让 critic 的速度参考不超过底盘或 MPPI 配置的合理速度范围。

建议与 Ego-Planner 的 `max_vel`、MPPI 的速度约束保持一致或略低。

## 速度方向参数

### `velocity_direction_weight`

速度方向误差权重。

本参数关注候选轨迹的实际运动方向是否接近参考 yaw，而不是车体 yaw 是否接近参考 yaw。

对全向底盘很有用：

- `yaw_weight` 可以较低。
- `velocity_direction_weight` 可以相对更高。
- 这样允许车体朝向和运动方向解耦，但仍鼓励沿 Ego/B-spline 切线方向运动。

### `velocity_direction_min_speed`

速度方向误差的最低速度门槛。

当候选速度很低时，速度方向 `atan2(vy, vx)` 容易抖动且意义不强。因此：

```cpp
candidate_speed <= velocity_direction_min_speed
```

时，速度方向误差直接记为 `0`。

### `velocity_frame`

指定 `data.state.vx / data.state.vy` 的坐标系语义。

可选值：

- `"base"`：`vx/vy` 是机器人坐标系速度。
- `"global"`：`vx/vy` 已经是全局坐标系速度。

当前工程中 MPPI 优化器积分位置时，会用候选 yaw 将 `state.vx/vy` 旋转到全局坐标：

```cpp
dx = vx * cos(yaw) - vy * sin(yaw)
dy = vx * sin(yaw) + vy * cos(yaw)
```

因此当前默认值选择 `"base"`，保持和现有 MPPI 状态语义一致。

如果设置为 `"base"`，本 critic 会先把速度旋转到 global，再计算速度方向：

```cpp
velocity_x = vx * cos(yaw) - vy * sin(yaw)
velocity_y = vx * sin(yaw) + vy * cos(yaw)
velocity_yaw = atan2(velocity_y, velocity_x)
```

如果设置为 `"global"`，则直接：

```cpp
velocity_yaw = atan2(vy, vx)
```

如果配置成其他字符串，代码会打印 warning 并回退到 `"base"`。

## 曲率参数

### `use_curvature_cost`

是否启用曲率误差代价。

默认 `false`，用于保持向后兼容：不开启时旧行为基本不变。

启用后，本 critic 会鼓励 MPPI 候选轨迹不仅贴近参考点，还贴近 Ego-Planner / B-spline 的转弯连续性。

### `curvature_weight`

曲率误差权重。

曲率误差计算方式：

```cpp
curvature_error = abs(candidate_curvature - ref.curvature)
step_cost += curvature_weight * curvature_error
```

建议从较小值开始，例如 `0.05 ~ 0.1`。

### `max_reference_curvature`

参考曲率限幅。

如果消息点未来增加 `curvature` 字段，代码会优先读取该字段；当前消息没有该字段，所以使用离散轨迹估计：

```cpp
ds = distance(p[i + 1], p[i - 1])
dyaw = shortest_angular_distance(yaw[i - 1], yaw[i + 1])
curvature = dyaw / max(ds, epsilon)
```

随后使用 `max_reference_curvature` 做对称限幅，避免离散噪声导致曲率异常放大。

### `max_candidate_curvature`

MPPI 候选轨迹曲率限幅。

候选曲率估计方式：

```cpp
ds_candidate = distance(candidate[t], candidate[t - 1])
dyaw_candidate = shortest_angular_distance(yaw[t - 1], yaw[t])
candidate_curvature = dyaw_candidate / max(ds_candidate, epsilon)
```

随后使用 `max_candidate_curvature` 做限幅。

如果该值过小，会削弱急转弯候选的表达；如果过大，离散噪声可能导致曲率 cost 过敏。

## 加速度参数

### `use_acceleration_cost`

是否启用加速度误差代价。

默认 `false`。这是有意设计：加速度是可选弱约束，避免一上来就改变旧控制行为。

### `acceleration_weight`

加速度误差权重。

启用后：

```cpp
acceleration_error = abs(candidate_acceleration - ref.acceleration)
step_cost += acceleration_weight * acceleration_error
```

建议保持弱权重，例如 `0.01 ~ 0.05`。

### `max_reference_acceleration`

参考加速度限幅。

如果消息点未来增加 `acceleration` 字段，代码会优先读取该字段；当前消息没有该字段，所以使用速度差分估计：

```cpp
acceleration[i] = (speed[i] - speed[i - 1]) / max(dt, epsilon)
```

然后使用 `max_reference_acceleration` 做对称限幅。

### `max_candidate_acceleration`

MPPI 候选轨迹加速度限幅。

候选加速度估计方式：

```cpp
candidate_speed = sqrt(vx^2 + vy^2)
candidate_acceleration =
  (candidate_speed[t] - candidate_speed[t - 1]) / model_dt
```

只在 `t > 0` 时计算，并使用 `max_candidate_acceleration` 限幅。

## 运行时自检参数

### `debug_enabled`

运行时自检总开关。

- `false`：默认值，不执行额外轨迹诊断、candidate 抽样诊断和 cost 增量检查。
- `true`：启用 debug 逻辑，轨迹回调检查和 `score()` 运行时检查都会按
  `debug_log_period` 先做早退出，避免每个控制周期都扫描。

默认关闭是为了避免在 MPPI 高频控制循环中引入额外候选轨迹抽样和日志开销。
即使开启，也只按 `debug_log_period` 低频运行。

### `debug_log_period`

debug 日志节流周期，单位秒。

所有 debug warning / info 都使用 throttle。值越小，日志越密；值越大，越适合长期运行观察。

### `debug_sample_candidates`

`score()` 中最多抽查多少条 MPPI 候选轨迹。

该参数只影响 debug 检查，不影响控制结果。建议保持较小值，例如 `3`。
运行时会在 batch 内抽样少量候选轨迹，而不是扫描整个 batch。设置为 `0` 时不会打印
candidate 抽样详情。

### `debug_check_trajectory_on_callback`

控制收到 Ego trajectory 后是否检查参考轨迹本身。

检查内容包括：

- 点数量是否小于 2。
- `header.frame_id` 是否为空。
- 输入 frame 和 costmap global frame 是否一致。
- `trajectory_dt` 是否过大、过小或非正。
- 连续点间距是否异常。
- yaw 是否跳变。
- speed 是否跳变或全部接近 0。
- curvature / acceleration 是否过大。
- 点内 `x/y/yaw/speed/curvature/acceleration` 是否非有限。

### `debug_check_score_runtime`

控制 `score()` 中是否执行运行时匹配诊断。

检查内容包括：

- robot 当前 pose 和 `start_idx` 参考点的距离。
- MPPI `model_dt` 和 Ego `trajectory_dt` 的比例。
- 关键预测步的参考采样时间和参考点。
- batch 内最多 `debug_sample_candidates` 条候选轨迹和参考轨迹的抽样误差。
- 本 critic 的抽样新增 cost 是否非有限、超出合理范围或过大。

### `debug_max_point_gap`

连续 Ego 参考点之间允许的最大距离。

超过该值会提示：

```text
Large point gap detected; possible unordered trajectory points or frame jump.
```

常见原因：

- 轨迹点顺序错乱。
- 坐标系 transform 出现跳变。
- Ego-Planner 输出点间隔过稀。

### `debug_min_point_gap`

连续 Ego 参考点之间允许的最小距离。

低于该值会提示可能存在重复点。重复点太多会影响 yaw、curvature、acceleration 的离散估计。

### `debug_max_yaw_jump`

连续参考 yaw 最大允许跳变，单位 rad。

超过该值会提示：

```text
Yaw jump detected; possible angle wrapping issue.
```

常见原因：

- yaw 未按 shortest angular distance 处理。
- 轨迹方向突然反转。
- 上游 yaw 字段未正确填充。

### `debug_max_speed_jump`

连续参考速度最大允许跳变。

超过该值说明 Ego reference 的速度剖面可能不连续，或者上游 velocity 字段有异常。

### `debug_max_curvature`

debug 判定参考曲率异常的阈值。

超过该值不会改变控制，只打印 warning。常见原因是轨迹几何不平滑、yaw 跳变、重复点导致 `ds` 太小。

### `debug_max_acceleration`

debug 判定参考加速度异常的阈值。

超过该值通常说明速度剖面突变，或者 `trajectory_dt` 不合理。

### `debug_max_cost`

debug 判定本 critic 新增 cost 过大的阈值。

`score()` 中如果 debug 开启，代码会在本 critic 计算出新增 cost 后，对少量候选轨迹做抽样检查：

```cpp
added_cost = ego_trajectory_critic_cost
```

这样避免拷贝完整 `data.costs`。如果抽样 `added_cost` 非有限、超出合理范围或超过
`debug_max_cost`，会打印 warning。

### `debug_large_error_distance`

debug 判定“大距离误差”的阈值。

用于两处：

- robot 当前 pose 到最近 Ego reference 点的距离。
- 抽样 candidate 到按时间采样 reference 点的位置误差。

如果该值频繁超限，通常优先检查坐标系、轨迹是否过旧、时间对齐和 `start_idx` 匹配。

## B-spline 优势如何被利用

升级后的 `EgoTrajectoryCritic` 利用 Ego-Planner / B-spline 轨迹的方式包括：

1. 平滑位置参考：通过 `position_error` 引导候选轨迹靠近参考位置。
2. 连续朝向参考：通过 `yaw_error` 和角度插值处理参考 yaw。
3. 速度参考：通过 `velocity_error` 跟随参考速度。
4. 速度方向参考：通过 `velocity_direction_error` 鼓励沿轨迹切线方向运动。
5. 曲率连续性：通过可选 `curvature_error` 鼓励候选轨迹转弯形态接近 B-spline。
6. 加速度连续性：通过可选 `acceleration_error` 弱约束速度变化。

但需要注意：

- MPPI 仍然是最终控制器。
- 本 critic 不会直接输出 `cmd_vel`。
- 本 critic 只是给 MPPI 候选轨迹加一项参考代价。
- 障碍物、安全约束、动力学约束等 critic 应保持更高优先级。

## 调参建议

全向底盘通常可以从以下方向调：

- 降低 `yaw_weight`，例如 `0.0 ~ 0.1`。
- 提高 `velocity_direction_weight`，例如 `0.3 ~ 0.5`。
- 开启 `use_curvature_cost`，从 `curvature_weight: 0.05 ~ 0.1` 试起。
- 暂时关闭 `use_acceleration_cost`，确认曲率效果稳定后再小权重开启。
- 如果贴 Ego 轨迹过强导致避障不自然，优先降低 `cost_weight`、`position_weight` 或 `distance_penalty_weight`。
- 如果 MPPI 明显不跟 Ego 速度，适当提高 `velocity_weight`。
