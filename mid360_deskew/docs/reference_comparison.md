# 参考项目对比与适配决策

本文记录重新实现 `mid360_deskew` 时做过的源码级对比和取舍。

## 项目约束

- Odin1 仍然是唯一主定位 / 主 odom 来源。
- MID360 是补盲障碍物传感器，只输入 Nav2 `local_costmap`。
- 本包只发布 `/mid360/points_deskewed`。
- 本包不发布 odom、map、`map->odom`、TF odom、特征点、关键帧或地图。
- 本包不依赖 `ring` 字段。
- 本包必须使用 `base_link -> mid360_link`；在 IMU 模式下还必须使用 `mid360_link -> mid360_imu_link`。

## 参考项目对比

| 参考项目 | 有价值的代码 / 设计 | 未采用内容 | 本包中的决策 |
|---|---|---|---|
| `ori-drs/lidar_undistortion` | 外部 pose 驱动的运动补偿；pose buffer；平移线性插值；四元数 slerp；`T_ref^-1 * T_i * p` | 针对 Mechanical/Ouster/Hesai 的 range image 时间模型、reprocess buffer、PCL 点类型模板 | 采用 odom 驱动逐点变换和 `PoseBuffer` 行为，并适配到 `nav_msgs/msg/Odometry` |
| `Livox_cloud_undistortion_ROS2` | Livox 风格自运动去畸变；gyro 积分；scan start/end 参考时刻 | 用 intensity 小数部分编码时间、PCL/Sophus 管线、把 rotation-only demo 当成完整管线 | 采用 scan target 概念和 gyro 积分思路；保留通用 `PointCloud2` 动态字段处理 |
| `LIO-SAM` | `deskewInfo()`、`imuDeskewInfo()`、`odomDeskewInfo()`、`deskewPoint()`；显式检查 IMU/odom 覆盖范围；要求每点相对时间 | range image、ring 组织、边缘/平面特征、因子图、地图优化、LIO odom | 只采用每点时间检查和覆盖失败处理思路 |
| `MOLA lidar odometry` | 运动补偿方法分层和 fallback 策略 | 完整 LO/LIO 管线、ICP、MRPT/MOLA 运行时、odom 输出 | 采用 `deskew_method` 和 `fallback_method` 参数设计 |
| `FAST_LIO / FAST_LIO2` | IMU buffer、LiDAR-IMU 时间覆盖检查、gyro bias 思路、外参使用、反向补偿到 scan end | iEKF、ikd-tree 地图、scan-to-map 匹配、在线外参估计、odom/map 发布、基于加速度的传播 | 采用 IMU buffer 覆盖检查、静止 gyro bias 初始化，以及通过 TF 显式处理 LiDAR-IMU 坐标旋转 |
| `LIO-Livox` | Livox `offset_time` 纳秒时间、IMU 消息区间获取、完整预积分前的 gyro-only delta rotation | Ceres 优化、特征提取、滑窗 LIO 初始化、完整 IMU 预积分状态 | 采用纳秒每点时间处理和区间覆盖思路；不使用优化或加速度预积分 |
| `livox_ros_driver2` | MID360 ROS2 topic 和字段：`CustomPoint.offset_time`、`CustomMsg.timebase`、PointCloud2 `timestamp`、IMU topic 和 frame 行为 | Livox LIO 或 odom 发布 | 同时支持 `sensor_msgs/msg/PointCloud2` 和直接 `livox_ros_driver2/msg/CustomMsg` 输入；CustomMsg 直接使用每点 `offset_time` |

## 最终实现选择

对当前机器人来说，最合适的方案不是移植完整 LIO 管线，而是一个小型点云前处理节点：

- `odom` 模式：Odin1 odom 提供 `T_odom_base(t)`。每个点按下面方式补偿：

```text
p_corrected =
  inverse(T_odom_mid360(t_ref))
  * T_odom_mid360(t_i)
  * p_raw
```

- `imu_only` 模式：MID360 IMU gyro 先旋转到 `mid360_link`，再积分得到 rotation-only 补偿。该模式不补偿平移。

- `odom_imu` 模式：采用策略 A。

```text
T_rel_odom =
  inverse(T_odom_mid360_odom(t_ref))
  * T_odom_mid360_odom(t_i)

T_rel.translation = T_rel_odom.translation
T_rel.rotation    = R_lidar_ref_i_from_imu

p_corrected = T_rel * p_raw
```

该策略把 Odin1 odom 的平移与 MID360 gyro 的高频旋转结合起来，但不估计新的全局位姿。

## 明确不做的内容

- 不做 iEKF、图优化、scan-to-map 或特征提取。
- 第一版不使用 IMU 加速度积分平移。
- 不做在线外参标定。
- 不发布 odom、map 或 `map->odom`。
- 不把 MID360 数据送入 `global_costmap`。

## Livox 字段说明

根据本地 `livox_ros_driver2` 源码：

- `CustomPoint.msg` 包含 `uint32 offset_time`，该值相对 `CustomMsg.timebase`。
- `CustomMsg.msg` 包含 `uint64 timebase`，表示第一个点的时间。
- PointCloud2 路径定义字段 `x`、`y`、`z`、`intensity`、`tag`、`line` 和 `timestamp`。
- 驱动会用纳秒单位的每点时间戳填充 `timestamp`。
- IMU 消息发布为 `sensor_msgs/msg/Imu`。

因此本包同时支持 PointCloud2 中的相对 `offset_time` 和绝对 `timestamp`，并保持 `point_time_unit` 可配置。deskew 前会先减去当前 scan 内的最小点时间，把点时间统一成帧内相对时间。

对于本地 `livox_ros_driver2` 的 PointCloud2 输出，建议使用：

```yaml
point_time_field: "timestamp"
point_time_unit: "nanosecond"
```

如果你的驱动导出了 PointCloud2 字段 `offset_time`，则使用：

```yaml
point_time_field: "offset_time"
point_time_unit: "nanosecond"
```

如果 PointCloud2 不包含任何每点时间字段，应改用直接 CustomMsg 输入：

```yaml
cloud_input_type: "custom_msg"
custom_cloud_topic: "/livox/lidar"
custom_msg_offset_unit: "nanosecond"
custom_msg_timebase_type: "ros_header"
```
