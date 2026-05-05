# mid360_deskew 测试计划

本包只为 Nav2 `local_costmap` 发布去畸变后的 MID360 `PointCloud2`。它不应该发布 odom、map 或 `map->odom`。

## 1. 字段检查

```bash
ros2 topic echo /mid360/points --once
```

确认点云中存在 `x`、`y`、`z`，并且至少存在一个每点时间字段：

- `offset_time`
- `time`
- `timestamp`
- `t`

对于 `livox_ros_driver2` 输出的 PointCloud2，常见字段是 `timestamp`，通常为 `FLOAT64`，内容是纳秒单位的每点时间戳。对于 CustomMsg，`offset_time` 是相对 `timebase` 的偏移。本节点会用当前 scan 内的最小点时间做归一化，因此只要单位配置正确，相对时间和绝对时间字段都可以使用。

### CustomMsg 输入检查

```bash
ros2 topic info /livox/lidar
```

期望类型：

```text
Type: livox_ros_driver2/msg/CustomMsg
```

查看一帧消息：

```bash
ros2 topic echo /livox/lidar --once
```

重点检查：

- `header.stamp`
- `timebase`
- `point_num`
- `points[0].offset_time`
- 最后一个点的 `offset_time`

如果 `/mid360/points` 不包含 `offset_time`、`time`、`timestamp` 或 `t`，不要强行使用 PointCloud2 做逐点去畸变，应改用：

```yaml
cloud_input_type: "custom_msg"
```

## 2. 频率检查

```bash
ros2 topic hz /odom
ros2 topic hz /mid360/points
ros2 topic hz /mid360/imu
```

odom 相邻样本间隔应小于 `max_allowed_time_gap`。使用 `imu_only` 或 `odom_imu` 时，IMU 相邻样本间隔应小于 `max_allowed_imu_gap`。

## 3. 延迟检查

```bash
ros2 topic delay /odom
ros2 topic delay /mid360/points
ros2 topic delay /mid360/imu
```

同时对照节点日志中的统计：

- `avg_cloud_delay`、`std_cloud_delay`
- `avg_odom_delay`、`std_odom_delay`
- `avg_imu_delay`、`std_imu_delay`
- `Recommended value for odom_time_offset`
- `Recommended value for imu_time_offset`

这些推荐值只是比较各消息 `header.stamp` 相对 ROS `now()` 的延迟差，不等价于硬件时间同步。

## 4. TF 检查

```bash
ros2 run tf2_ros tf2_echo base_link mid360_link
ros2 run tf2_ros tf2_echo mid360_link mid360_imu_link
ros2 run tf2_ros tf2_echo odom base_link
```

期望 TF 树：

```text
odom
  base_link
    odin1_link
    mid360_link
      mid360_imu_link
```

节点使用 `/odom` 中的 `T_odom_base(t)`、TF 中的 `T_base_mid360`，并在积分前把 IMU gyro 转到 `mid360_link`。不要把 Odin1 和 MID360 当成同一个物理位姿源。

期望 odom 消息语义为 `T_fixed_base(t)`：`odom.header.frame_id` 应与 `fixed_frame` 一致，`odom.child_frame_id` 应与 `base_frame` 一致。

## 5. 三种模式测试

### A. imu_only

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_imu_only.yaml
```

让机器人快速原地旋转。旋转拖影应有改善。由于 `imu_only` 不补偿平移，横向移动或平移较明显时点云仍可能不正确。

### B. odom

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_odom.yaml
```

让机器人移动并旋转。如果 `/odom` 频率足够高，且与 MID360 时间对齐，该模式应能补偿平移和旋转。

### C. odom_imu

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_odom_imu.yaml
```

让机器人快速移动并旋转。该模式的旋转补偿应比纯 `odom` 更能适应高频旋转，同时平移补偿应优于 `imu_only`。

### D. CustomMsg odom

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_custom_odom.yaml
```

从节点启动日志确认 `cloud_input_type=custom_msg`。此时节点应订阅 `custom_cloud_topic`，不应订阅 PointCloud2 输入 topic。

### E. CustomMsg odom_imu

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_custom_odom_imu.yaml
```

当 odom 提供平移，且 MID360 IMU 能覆盖完整 scan 时间区间时使用该模式。

### F. CustomMsg imu_only

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_custom_imu_only.yaml
```

该模式只建议用于验证旋转拖影补偿；它不补偿平移。

## 6. IMU 旋转方向 bag 测试

录制或回放一段机器人原地顺时针、逆时针旋转的短 bag。对比：

- 原始 MID360 点云
- `imu_only`
- `odom`
- `odom_imu`

先保持：

```yaml
imu_rotation_sign: 1.0
```

如果原地旋转时 `imu_only` 明显比 raw 点云更差，临时改成：

```yaml
imu_rotation_sign: -1.0
```

然后回放同一段 bag。如果 `-1.0` 明显改善点云，应重点检查 quaternion 方向、gyro 坐标系变换，以及代码中实际使用的是 `q_i_to_ref` 还是 `q_ref_to_i`。调试日志会为 `imu_only` / `odom_imu` 打印 gyro bias、积分时间区间、相对 quaternion 和 yaw delta。

## 7. RViz 对比

同时显示这些 topic：

- `/mid360/points`
- 使用 CustomMsg 输入时的 `/livox/lidar`
- `/mid360/points_deskewed`

局部运动检查时，RViz fixed frame 建议使用 `odom`。去畸变输出点云的 frame 应为 `mid360_link`。

## 8. 如果去畸变后更差

优先检查以下项：

a. `point_time_unit` 配错。

a2. CustomMsg 模式下 `custom_msg_offset_unit` 配错。

b. `cloud_stamp_type` 配错。

c. `odom_time_offset` 配错。

d. `imu_time_offset` 配错。

e. `base_link -> mid360_link` 外参错误。

f. `mid360_imu_link -> mid360_link` 外参错误。

g. 启动时机器人移动，导致 gyro bias 未正确估计。

h. IMU angular velocity 单位或轴向错误。

h2. IMU quaternion / 相对旋转方向反了；对比 `imu_rotation_sign: 1.0` 和 `-1.0`。

i. Odin1 与 MID360 没有稳定共享时间基准。

i2. 启用了 `custom_msg_timebase_type: "custom_timebase"`，但 Livox timebase 没有与 ROS odom/IMU 时间同步。

j. 测试或 costmap 使用了 `map`，而不是 `odom`。

k. odom 频率过低，或 IMU 样本间隔大于 `max_allowed_imu_gap`。
