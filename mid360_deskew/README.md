# mid360_deskew

`mid360_deskew` 是一个面向 ROS 2 Humble 的轻量级 C++ 点云预处理节点，用于对 Livox MID360 补盲雷达点云做逐点去畸变。

本包针对的机器人结构是：

- Odin1 是主雷达，负责主定位和主 `/odom` 链路。
- MID360 只作为近距离补盲、避障、防撞传感器。
- MID360 去畸变后的点云只应该进入 Nav2 `local_costmap`。

本包不是 LIO，不做定位，不建图，不发布 odom，不发布 map，不发布 `map -> odom`，不提取特征，不做因子图优化，也不修改 Odin1 主定位链路。

## 输入输出

订阅：

- `/mid360/points`：`sensor_msgs/msg/PointCloud2`，当 `cloud_input_type: "pointcloud2"` 时使用
- `/livox/lidar`：`livox_ros_driver2/msg/CustomMsg`，当 `cloud_input_type: "custom_msg"` 时使用，话题由 `custom_cloud_topic` 配置
- `/odom`：`nav_msgs/msg/Odometry`
- `/mid360/imu`：`sensor_msgs/msg/Imu`
- TF：`base_link -> mid360_link`
- IMU 模式额外需要 TF：`mid360_link -> mid360_imu_link`，除非 IMU 数据本来就已经在 `mid360_link` 下表达

发布：

- `/mid360/points_deskewed`：`sensor_msgs/msg/PointCloud2`

PointCloud2 输入时，输出点云会尽量保持输入 `PointCloud2` 的原始字段和布局，只修改 `x/y/z`。CustomMsg 输入时，输出会转换为 `PointCloud2`，包含 `x/y/z/intensity/offset_time/tag/line`。距离过滤或无效点会在 `keep_size_nan` 模式下写成 NaN，不改变点数；CustomMsg 输入也支持 `filter_mode: "compact"` 压缩掉无效点。

## 坐标系

推荐 TF 树：

```text
odom
  base_link
    odin1_link
    mid360_link
      mid360_imu_link
```

节点将 Odin1 odom 理解为：

```text
T_odom_base(t)
```

更一般地说，`odom_topic` 的消息语义必须是：

```text
T_fixed_base(t)
```

其中 `odom.header.frame_id` 应等于 `fixed_frame`，`odom.child_frame_id` 应等于 `base_frame`。节点会对这两个 frame 做 throttle warning；frame 不一致时，去畸变时间和坐标语义都可能错。

然后通过静态外参计算：

```text
T_odom_mid360(t) = T_odom_base(t) * T_base_mid360
```

不要把 Odin1 和 MID360 当成同一个物理位置。`base_link -> mid360_link` 外参是必须的。

## 去畸变模式

### odom

只使用 Odin1 `/odom` 插值得到每个点时刻的位姿，补偿旋转和平移：

```text
p_corrected =
  inverse(T_odom_mid360(t_ref))
  * T_odom_mid360(t_i)
  * p_raw
```

优点是能补偿平移和旋转；缺点是依赖 Odin1 odom 频率和 MID360 点云时间对齐。

### imu_only

只使用 MID360 IMU 的 gyro，在 `mid360_link` 下做旋转补偿。

这个模式只补旋转，不补平移，不能说成完整运动补偿。它适合先验证原地快速旋转时拖影是否改善。

### odom_imu

增强模式，也是默认推荐模式。

分工：

- Odin1 odom 提供平移和低频位姿连续性。
- MID360 IMU gyro 提供一帧点云内部的高频相对旋转。
- MID360 点云的 `offset_time` / `timestamp` 提供每个点的采样时刻。

当前工程策略：

```text
T_rel_odom =
  inverse(T_odom_mid360_odom(t_ref))
  * T_odom_mid360_odom(t_i)

T_rel.translation = T_rel_odom.translation
T_rel.rotation    = R_lidar_ref_i_from_imu

p_corrected = T_rel * p_raw
```

这不是紧耦合 LIO，不会估计新的机器人全局位姿。

`rotation_source: "imu"` 是第一版推荐设置。`rotation_source: "odom"` 会保留 odom 相对旋转；`rotation_source: "fused"` 目前只是 odom 相对旋转和 IMU 相对旋转的 50/50 四元数 slerp，适合作为实验对比，不建议一开始就依赖。

`imu_rotation_sign: 1.0` 用于 IMU 旋转方向验证。正常保持 `1.0`；如果 `imu_only` 去畸变比 raw 点云更差，可以临时设为 `-1.0` 做对比，同时检查 quaternion 方向、gyro frame、`q_i_to_ref / q_ref_to_i` 语义。

## 配置文件

默认 launch 加载：

```text
config/mid360_deskew_odom_imu.yaml
```

另外提供两个测试配置：

- `config/mid360_deskew_odom.yaml`
- `config/mid360_deskew_imu_only.yaml`

Livox CustomMsg 输入配置：

- `config/mid360_deskew_custom_odom.yaml`
- `config/mid360_deskew_custom_odom_imu.yaml`
- `config/mid360_deskew_custom_imu_only.yaml`

兼容入口：

- `config/mid360_deskew.yaml`，内容与增强默认配置一致

本包直接依赖 `livox_ros_driver2` 来 include `livox_ros_driver2/msg/custom_msg.hpp`。如果系统没有安装该包，需要先安装或把 `livox_ros_driver2` 源码放入工作区一起编译。

运行：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
ros2 launch mid360_deskew mid360_deskew.launch.py
```

指定配置文件：

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_odom.yaml
```

如果 `/mid360/points` 的 PointCloud2 没有每点时间字段，推荐改用 CustomMsg：

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_custom_odom.yaml
```

建议初始时间偏移先全部设为 0：

```yaml
cloud_time_offset: 0.0
odom_time_offset: 0.0
imu_time_offset: 0.0
```

然后参考节点的 `time_check` 输出辅助调参：

```text
Recommended value for odom_time_offset: xxx seconds
Recommended value for imu_time_offset: xxx seconds
```

这些推荐值不是独立 YAML 参数。时间自检只是在比较各消息 `header.stamp` 相对 ROS `now()` 的延迟差，不能替代硬件时间同步。如果每次启动推荐值变化很大，说明 Odin1 和 MID360 的时间基准可能不稳定，需要检查驱动时间戳或硬件同步。

## 点时间字段

节点会动态检测：

- `x`
- `y`
- `z`
- `offset_time` / `time` / `timestamp` / `t` 之一

支持的点时间单位：

- `second`
- `millisecond`
- `microsecond`
- `nanosecond`

本地 `livox_ros_driver2` 的 PointCloud2 输出通常有 `timestamp` 字段，内容是每个点的纳秒时间戳。节点内部会用本帧最小点时间做归一化，因此绝对 `timestamp` 和相对 `offset_time` 都可以用，只要单位配置正确。

使用 `livox_ros_driver2` PointCloud2 输出时通常可设：

```yaml
point_time_field: "timestamp"
point_time_unit: "nanosecond"
```

如果你的 PointCloud2 里直接有相对时间字段 `offset_time`：

```yaml
point_time_field: "offset_time"
point_time_unit: "nanosecond"
```

如果找不到点时间字段且 `drop_if_no_point_time: true`，节点会丢弃该帧，不会发布错误的“去畸变”点云。

## Livox CustomMsg Input

如果 `/mid360/points` 的 `sensor_msgs/msg/PointCloud2` 没有 `offset_time`、`time`、`timestamp` 或 `t` 字段，就不能可靠做 per-point deskew。此时应优先使用 `livox_ros_driver2/msg/CustomMsg`，因为它的每个点包含 `offset_time`。

常用配置：

```yaml
cloud_input_type: "custom_msg"
custom_cloud_topic: "/livox/lidar"
custom_msg_timebase_type: "ros_header"
custom_msg_offset_unit: "nanosecond"
cloud_stamp_type: "scan_start"
```

CustomMsg 路径会直接读取 `points[i].offset_time`，用本帧最小 offset 做归一化，然后计算：

```text
t_i = scan_start + (offset_time_i - min_offset_time)
```

不要把 `offset_time` 当作绝对 ROS 时间。`custom_msg_timebase_type: "ros_header"` 是默认推荐值，会使用 `msg.header.stamp`，再按 `cloud_stamp_type` 判断它代表 scan_start 还是 scan_end。

`custom_msg_timebase_type: "custom_timebase"` 第一版不会直接拿 `msg.timebase` 查询 Odin1 odom，而是打印 warning 并 fallback 到 `msg.header.stamp`。只有在 Livox timebase 已经与 ROS time、Odin1 `/odom`、MID360 IMU 时间同步后，才应该考虑真正启用 `timebase` 查询。

输出仍然是：

```text
/mid360/points_deskewed  sensor_msgs/msg/PointCloud2
```

CustomMsg 输出 PointCloud2 的 `offset_time` 字段保留 raw Livox `offset_time`。内部去畸变使用的是 `normalized_offset = raw_offset - min_raw_offset`；输出的 `offset_time` 仅用于调试和追溯，不建议下游再拿它做二次 deskew。

如果去畸变后更差，优先检查：

- `custom_msg_offset_unit`
- `cloud_stamp_type`
- `cloud_time_offset`
- `odom_time_offset`
- `custom_msg_timebase_type`
- `base_link -> mid360_link` 外参

## IMU 处理

IMU buffer 会缓存最近几秒的 IMU 数据，并把 gyro 转到 `mid360_link` 下再积分。

关键点：

- 如果 IMU 消息 `header.frame_id` 非空，优先使用消息里的 frame。
- 否则使用参数 `imu_frame`。
- 若 IMU frame 与 `lidar_frame` 不同，需要 TF 能查到 `lidar_frame <- imu_frame`。
- 启动初期可以用静止样本估计 gyro bias。
- `linear_acceleration` 会被缓存，但第一版不用于平移积分。
- `use_acc_for_translation` 当前应保持 `false`。

如果 IMU TF 查不到：

- `imu_only` 会失败，并按失败策略丢帧或发布 raw。
- `odom_imu` 在 `fallback_method: "odom"` 时会降级到 odom-only。

## 失败策略

增强默认配置：

```yaml
fallback_method: "odom"
drop_if_no_point_time: true
drop_if_no_pose: true
drop_if_no_imu: false
publish_raw_when_failed: false
```

缺数据、插值失败、IMU 覆盖不足都会明确打印 warning。节点不会静默把失败点云当成 corrected cloud 发布。

## Nav2 使用方式

MID360 去畸变点云只应该进入 `local_costmap`：

- `global_frame: odom`
- topic: `/mid360/points_deskewed`

不要把 MID360 放进 `global_costmap`，也不要让 MID360 影响 `/map`。

完整示例见：

```text
docs/nav2_local_costmap_example.yaml
```

## 推荐调试顺序

1. 先跑 `imu_only`，看原地快速旋转拖影是否改善。
2. 再跑 `odom`，看平移加旋转是否改善。
3. 最后跑 `odom_imu`，验证快速旋转加横移时是否优于前两者。

如果 corrected 点云更差，优先检查：

- `point_time_unit` 是否错误。
- `cloud_stamp_type` 是否错误。
- `odom_time_offset` 符号或数值是否错误。
- `imu_time_offset` 符号或数值是否错误。
- `base_link -> mid360_link` 外参是否错误。
- `mid360_imu_link -> mid360_link` 外参是否错误。
- 启动时机器人是否在动，导致 gyro bias 没估准。
- IMU 角速度单位或坐标轴是否错误。
- Odin1 与 MID360 时间基准是否一致。
- RViz 或 costmap 是否用了 `map` 而不是 `odom`。
- odom 频率是否过低，或 IMU 样本间隔是否超过 `max_allowed_imu_gap`。

## References and Design Mapping

- [FAST_LIO / FAST_LIO2](https://github.com/hku-mars/FAST_LIO)：借鉴 IMU buffer、IMU 覆盖检查、gyro bias 处理思想、LiDAR-IMU 外参意识和点云去畸变时间流程。明确没有借鉴 iEKF、ikd-tree map、scan-to-map matching、odom/map 发布或基于加速度的完整状态传播。

- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)：借鉴 `deskewPoint` 的逐点相对时间补偿思想、IMU/odom 队列覆盖判断，以及点时间字段必须可靠的设计。明确没有借鉴 range image、ring 组织、edge/surf 特征、因子图或 mapping。

- [Livox_cloud_undistortion_ROS2](https://github.com/Tim-HW/Livox_cloud_undistortion_ROS2)：借鉴 Livox/MID360 每点时间和 ego-motion deskew 思路，以及 scan start/end 补偿目标。明确没有借鉴 intensity 小数时间编码或完整 PCL/Sophus 流水线。

- [ori-drs/lidar_undistortion](https://github.com/ori-drs/lidar_undistortion)：借鉴外部 pose 驱动的 motion compensation、PoseBuffer 插值、平移线性插值和四元数 slerp。这里适配为 Odin1 `/odom` 和 MID360 `PointCloud2`。

- [MOLA lidar odometry](https://github.com/MOLAorg/mola_lidar_odometry)：借鉴 motion compensation method 和 fallback method 的模式化设计。明确没有借鉴完整 LO/LIO pipeline、ICP、map 更新或 odometry 输出。

- [LIO-Livox](https://github.com/Livox-SDK/LIO-Livox)：借鉴 Livox `offset_time`、IMU 时间区间处理和静止初始化 / bias 处理思路。明确没有借鉴 Ceres 优化、特征提取、滑窗 LIO 初始化或完整 IMU 预积分。

- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)：用于确认 MID360 `CustomPoint.offset_time`、`CustomMsg.timebase`、PointCloud2 `timestamp` 和 `sensor_msgs/msg/Imu` 发布行为。

本包不是上述任何项目的完整复现，而是为“主雷达 Odin1 + 补盲雷达 MID360 + Nav2 local_costmap”场景定制的轻量 corrected `PointCloud2` 前处理节点。

更细的源码对比见：

```text
docs/reference_comparison.md
```
