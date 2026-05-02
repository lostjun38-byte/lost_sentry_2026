# mid360_deskew 严格代码自检报告

审查对象：当前 `src/mid360_deskew` 仓库源码。  
审查口径：按 `README.md` 的设计约束检查是否只是 MID360 corrected `PointCloud2` 前处理节点，不检查或新增新功能。

## 一、全局禁止项检查

| 检查项 | 是否发现 | 文件/函数 | 判断 |
|---|---:|---|---|
| `nav_msgs/msg/Odometry` publisher / `create_publisher<nav_msgs::msg::Odometry>` | 否 | 仅 `deskew_node.cpp:223` 是 odom subscription | 符合，不发布 odom |
| topic 名含 odom 的 publisher | 否 | 唯一 publisher 是 `deskew_node.cpp:214` `PointCloud2` | 符合 |
| `nav_msgs/msg/OccupancyGrid` publisher / map publisher | 否 | 全局搜索无源码命中 | 符合 |
| `sensor_msgs/msg/PointCloud2` 作为 map 发布 | 否 | 唯一输出 topic 来自 `output_cloud_topic_` | 符合 |
| `tf2_ros::TransformBroadcaster` / `sendTransform` | 否 | 只有 `tf2_ros::TransformListener` | 符合，不发布 TF |
| `child_frame_id == "odom"` / `header.frame_id == "map"` 发送 TF | 否 | 全局搜索无源码命中 | 符合 |
| scan-to-map / ICP / Ceres / GTSAM / iEKF / ikd-tree / mapping / feature extraction / factor graph | 否 | 源码目录无命中，README/docs 仅引用说明 | 符合 |

结论：当前源码确实只是 corrected `PointCloud2` 前处理节点；没有发现变相 LIO、定位、建图、map 发布、odom 发布或 `map->odom` TF 发布逻辑。

## 二、ROS 接口检查

| Topic/接口 | README 要求 | 当前代码 | 是否符合 | 风险 |
|---|---|---|---|---|
| 输入点云 | `PointCloud2` `/mid360/points` 可参数化 | `create_subscription<PointCloud2>` 使用 `input_cloud_topic_`，`deskew_node.cpp:218` | 是 | 低 |
| 输入 odom | `nav_msgs/msg/Odometry` `/odom` | `create_subscription<Odometry>`，`deskew_node.cpp:223` | 是 | 中：frame 只告警不拒绝 |
| 输入 IMU | `sensor_msgs/msg/Imu` `/mid360/imu` | `create_subscription<Imu>`，`deskew_node.cpp:228` | 是 | 低 |
| TF buffer | 查询 `base_link -> mid360_link` 和 IMU 外参 | `tf_buffer_`/`TransformListener`，`deskew_node.cpp:211` | 是 | 低 |
| 输出点云 | 唯一主要输出 `/mid360/points_deskewed` `PointCloud2` | 唯一 publisher 为 `cloud_pub_`，`deskew_node.cpp:214` | 是 | 低 |
| output frame | `header.frame_id = lidar_frame` | `deskew_node.cpp:701` | 是 | 低 |
| output stamp | `header.stamp = t_ref` | `deskew_node.cpp:702` | 是；失败 raw 例外 | 低 |
| 保留字段布局 | 复制输入，只改 `x/y/z` | `output = cloud`，`deskew_node.cpp:700`；`writeXYZ` 只写 xyz | 是 | 低 |
| keep_size_nan | 不改变点数，写 NaN | 循环原 `width/height`，不改 data size；`is_dense=false` | 是 | 低 |
| `publish_raw_when_failed=false` | 失败不发 raw | `handleDeskewFailure()` 只有 true 才 publish raw | 是 | 低 |

## 三、参数读取检查

| 参数 | 是否声明 | 是否读取 | 是否参与实际逻辑 | 默认值是否合理 | 问题 |
|---|---:|---:|---:|---:|---|
| `use_sim_time` | ROS2 TimeSource | ROS2 TimeSource | 是 | 是 | 源码未显式 declare，属 ROS2 内建参数 |
| `input_cloud_topic` | 是 | 是 | 是 | 是 | 无 |
| `output_cloud_topic` | 是 | 是 | 是 | 是 | 无 |
| `odom_topic` | 是 | 是 | 是 | 是 | 无 |
| `imu_topic` | 是 | 是 | 是 | 是 | odom 模式也订阅 IMU，但不影响输出 |
| `fixed_frame` | 是 | 是 | 仅日志/语义 | 是 | 中：未检查 `odom.header.frame_id` 是否等于它 |
| `base_frame` | 是 | 是 | 是 | 是 | `child_frame_id` 不匹配只告警 |
| `lidar_frame` | 是 | 是 | 是 | 是 | 无 |
| `imu_frame` | 是 | 是 | 是 | 是 | 无 |
| `deskew_method` | 是 | 是 | 是 | 是 | 非法值回退 odom |
| `fallback_method` | 是 | 是 | 是 | 是 | 支持 `linear` fallback |
| `deskew_target_time` | 是 | 是 | 是 | 是 | 非法值回退 scan_end |
| `rotation_source` | 是 | 是 | 是 | 是 | `fused` 是实验性 50/50 slerp |
| `translation_source` | 是 | 是 | 是 | 是 | 只实现 odom，其他值告警后当 odom |
| `point_time_field` | 是 | 是 | 是 | 是 | 无 |
| `point_time_unit` | 是 | 是 | 是 | 是 | 非法值导致丢帧/失败 |
| `cloud_stamp_type` | 是 | 是 | 是 | 是 | 非法值回退 scan_start |
| `cloud_time_offset` | 是 | 是 | 是 | 是 | 用于 cloud stamp 修正 |
| `odom_time_offset` | 是 | 是 | 是 | 是 | 用于 odom 查询，runtime 公式见下 |
| `imu_time_offset` | 是 | 是 | 是 | 是 | 用于 IMU 查询，runtime 公式见下 |
| `pose_buffer_duration` | 是 | 是 | 是 | 是 | 无 |
| `imu_buffer_duration` | 是 | 是 | 是 | 是 | 无 |
| `max_allowed_time_gap` | 是 | 是 | 是 | 是 | 无 |
| `max_allowed_imu_gap` | 是 | 是 | 是 | 是 | 无 |
| `allow_extrapolation` | 是 | 是 | 是 | 是 | 默认 false 合理 |
| `estimate_gyro_bias` | 是 | 是 | 是 | 是 | 无 |
| `gyro_bias_estimation_duration` | 是 | 是 | 是 | 是 | 无 |
| `gyro_bias_static_threshold` | 是 | 是 | 是 | 是 | 无 |
| `use_acc_for_translation` | 是 | 是 | 是 | 是 | true 会告警并强制 false |
| `min_range` | 是 | 是 | 是 | 是 | 无 |
| `max_range` | 是 | 是 | 是 | 是 | 无 |
| `remove_nan` | 是 | 是 | 是 | 是 | false 时保留输入 NaN/Inf |
| `filter_mode` | 是 | 是 | 是 | 是 | 只实现 keep_size_nan |
| `drop_if_no_point_time` | 是 | 是 | 是 | 是 | 无 |
| `drop_if_no_pose` | 是 | 是 | 是 | 是 | 对 NoPose 生效 |
| `drop_if_no_imu` | 是 | 是 | 是 | 是 | 对 NoImu 生效 |
| `publish_raw_when_failed` | 是 | 是 | 是 | 是 | 默认 false 安全 |
| `enable_time_check` | 是 | 是 | 是 | 是 | 无 |
| `time_check_window` | 是 | 是 | 是 | 是 | 无 |
| `time_check_min_samples` | 是 | 是 | 是 | 是 | 无 |
| `apply_auto_time_offset` | 是 | 是 | 是 | 是 | false 时不覆盖 runtime |
| `auto_time_offset_max_abs` | 是 | 是 | 是 | 是 | 无 |
| `delay_jitter_warning_threshold` | 是 | 是 | 是 | 是 | 无 |
| `print_time_check_result` | 是 | 是 | 是 | 是 | 无 |
| `enable_debug_log` | 是 | 是 | 是 | 是 | 无 |

偏移确认：

- `cloud_time_offset_` 用于 `adjusted_cloud_stamp = cloud.header.stamp + offset`，`deskew_node.cpp:449`。
- `odom_time_offset_runtime = odom_time_offset_config_ + odom_time_offset_auto_`，`deskew_node.cpp:1183`，查询处 `deskew_node.cpp:595`。
- `imu_time_offset_runtime = imu_time_offset_config_ + imu_time_offset_auto_`，`deskew_node.cpp:1188`，查询处 `deskew_node.cpp:616`。
- `apply_auto_time_offset=false` 时只打印建议，不改 `*_auto_`，符合 README。

## 四、PointCloud2 字段处理检查

| 检查点 | 文件/函数 | 当前实现 | 是否安全 | 风险等级 | 修改建议 |
|---|---|---|---|---|---|
| x/y/z 存在性 | `PointCloudFieldHelper::configure/hasXYZ` | 动态查找，缺失则失败 | 是 | 低 | 无 |
| x/y/z datatype | `readScalarAsDouble/writeScalarFromDouble` | 读取支持多类型，写只支持 FLOAT32/FLOAT64 | 基本安全 | 低 | 可在字段检测日志中强调 xyz 必须可写 |
| point_step 越界 | `findField/readScalarAsDouble/writeScalarFromDouble` | 检查 offset + size <= point_step | 是 | 低 | 无 |
| data buffer 越界 | `DeskewNode::cloudCallback` | 检查 `data.size()` 与 row/point step | 是 | 低 | 无 |
| is_bigendian | `readValue/writeValue` | 按主机端序 byte swap | 是 | 低 | 无 |
| 时间字段 fallback | `configure()` | 指定字段优先，fallback `offset_time/time/timestamp/t` | 是 | 低 | 无 |
| 时间 datatype | `readScalarAsDouble` | 支持 UINT32/INT32/FLOAT32/FLOAT64/非标准 UINT64 9 等 | 是 | 低 | 无 |
| 时间单位 | `unitScale()` | second/ms/us/ns | 是 | 低 | 无 |
| 找不到点时间 | `cloudCallback()` | true 丢帧，false 退化零 offset | 是 | 低 | 无 |
| timestamp 归一化 | `cloudCallback()` | 全部减本帧最小点时间 | 部分安全 | 中 | 增加 `point_time_is_absolute` 或按字段名启发式告警 |
| absolute/relative 判断 | 无显式判断 | README 说明统一减 min，但代码不区分 | 不完整 | 中 | 对 `timestamp` 与 `offset_time` 分开记录日志，异常时 warning |
| scan_duration | `cloudCallback()` | `max-min`，不硬编码 | 是 | 低 | 增加 <=0、>0.2s 等异常 warning |
| keep_size_nan | `deskewWithMethod()` | 写 NaN、不改尺寸、`is_dense=false` | 是 | 低 | 无 |

## 五、cloud_stamp_type 和点时间计算

期望伪代码：

```text
cloud_stamp_corrected = cloud.header.stamp + cloud_time_offset
scan_duration = max_point_offset - min_point_offset
if cloud_stamp_type == scan_start:
  scan_start = cloud_stamp_corrected
  scan_end = scan_start + scan_duration
else:
  scan_end = cloud_stamp_corrected
  scan_start = scan_end - scan_duration
point_offset = raw_point_time - min_point_time
t_i = scan_start + point_offset
t_ref = scan_start / scan_mid / scan_end
t_odom_query = t_i + odom_time_offset_runtime
t_imu_query = t_i + imu_time_offset_runtime
```

当前实现：

```text
raw offsets read at deskew_node.cpp:422-438
scan_duration = max(0, max-min) at deskew_node.cpp:448
cloud_time_offset applied at deskew_node.cpp:449
scan_start/scan_end selected at deskew_node.cpp:455-460
t_ref selected at deskew_node.cpp:463-468
offset -= min_point_offset at deskew_node.cpp:480-482
t_i = actual_scan_start + offset at deskew_node.cpp:756
odom query adds runtime offset at deskew_node.cpp:595
IMU query adds runtime offset at deskew_node.cpp:616-617
output.header.stamp = t_ref at deskew_node.cpp:702
```

结论：scan_start/scan_end 和 t_ref 主逻辑符合 README。主要缺口是没有对“绝对 timestamp 与相对 offset_time”做显式分类和异常提示。

## 六、PoseBuffer 检查

| 项目 | 当前实现 | 是否符合 | 风险 | 修改建议 |
|---|---|---|---|---|
| push odom stamp | `addOdometry()` 使用 `header.stamp` | 是 | 低 | 无 |
| 保存 position/orientation | 保存 pose.position 和 quaternion | 是 | 低 | 无 |
| quaternion 归一化 | `addPose()` normalize | 是 | 低 | 无 |
| NaN 处理 | 未显式拒绝 NaN/Inf pose | 否 | 中 | addPose 前检查 position/quaternion finite |
| 时间排序 | `lower_bound` 插入/同 stamp 替换 | 是 | 低 | 无 |
| 清理旧数据 | 按 newest - front > duration 清理 | 是 | 低 | 无 |
| 插值 bracket | `lower_bound` 找前后 | 是 | 低 | 无 |
| 平移插值 | linear interpolation | 是 | 低 | 无 |
| 旋转插值 | quaternion slerp | 是 | 低 | 无 |
| 等于某帧 | 直接返回该 pose | 是 | 低 | 无 |
| 空/单样本 | 明确失败，除非允许短外推 | 是 | 低 | 无 |
| gap 检查 | `gap > max_allowed_time_gap` 失败 | 是 | 低 | 无 |
| 禁止外推 | 默认 false 严格禁止 | 是 | 低 | 无 |
| 失败原因 | reason 字符串明确 | 是 | 低 | 无 |
| odom frame 语义 | 代码假定 msg pose 是 `T_fixed_base` | 部分符合 | 高 | `header.frame_id` 和 `child_frame_id` 不匹配应拒绝或变换 |

## 七、TF 外参方向检查

base 到 lidar：

- 当前调用：`lookupTransform(base_frame_, lidar_frame_, TimePointZero)`，`deskew_node.cpp:860-864`。
- tf2 语义：target=`base_frame_`，source=`lidar_frame_`，返回 source 到 target，即 `T_base_lidar`。
- 当前乘法：`poseToIsometry(ref_pose) * t_base_lidar`，`deskew_node.cpp:645/696/802`。
- 结论：得到的是 `T_odom_base * T_base_mid360`，方向正确。

IMU 到 lidar：

- 当前调用：`lookupTransform(lidar_frame_, source_imu_frame, TimePointZero)`，`deskew_node.cpp:293-297`。
- 返回 source 到 target，即 `T_lidar_imu`。
- 当前旋转：`gyro_lidar = R_lidar_imu * gyro_imu`，`deskew_node.cpp:299`。
- 结论：IMU 外参方向正确。

高风险补充：odom 输入本身必须是 `odom -> base_link`。如果输入的是 `odom -> odin1_link`，当前代码只 warning 但仍存入 PoseBuffer，这是必须修改的风险。

## 八、odom 模式检查

| odom 模式检查项 | 是否符合 | 文件/函数 | 风险 |
|---|---:|---|---|
| 不依赖 IMU | 是 | `deskewWithMethod()` `needs_imu=false` | 低 |
| 需要点时间 + odom + 外参 | 是 | `cloudCallback()` + `deskewWithMethod()` | 低 |
| 每点查询 odom | 是 | `deskew_node.cpp:760-767` | 低 |
| 查询 t_ref pose | 是 | `deskew_node.cpp:639-645` | 低 |
| `T_ref^-1 * T_i` | 是 | `deskew_node.cpp:699,826` | 低 |
| 没有整帧单 TF 冒充 deskew | 是 | 每点循环内查询/计算 | 低 |
| odom 模式下不因缺 IMU 丢帧 | 是 | `needs_imu=false` | 低 |
| `drop_if_no_pose` 生效 | 是 | `handleDeskewFailure()` | 低 |
| raw 发布默认关闭 | 是 | `publish_raw_when_failed=false` | 低 |

## 九、imu_only 模式检查

| 检查项 | 当前实现 | 是否符合 | 风险 |
|---|---|---:|---|
| 只做旋转 | `rotationOnlyPose(q_ref_point)`，无平移 | 是 | 低 |
| 不查询 odom | `needs_odom=false` | 是 | 低 |
| 需要 IMU 覆盖整帧 | full scan integrate check | 是 | 低 |
| gyro 转 lidar frame | IMU callback 中转换后入 buffer | 是 | 低 |
| 扣除 gyro bias | `angular_velocity_lidar - gyro_bias_` | 是 | 低 |
| 不用 acc 平移 | `use_acc_for_translation` 强制 false | 是 | 低 |
| max_allowed_imu_gap | 积分和插值均检查 | 是 | 低 |
| IMU 缺失失败策略 | NoImu 进入 `drop_if_no_imu`/raw 逻辑 | 是 | 低 |
| bias 失败 | warning，零 bias 继续 | 是 | 中：运动启动会保留零 bias |

旋转方向小例子：机器人绕 z 正方向从点时刻转到 scan_end。早期点补到 scan_end，需要把点按 z 负方向旋转到末帧坐标。当前代码调用 `integrate_imu(t_ref, point_time)`；当 `point_time < t_ref` 时内部先积分 `point_time -> t_ref` 再取 inverse，因此得到负方向旋转，方向符合补偿到 scan_end 的需求。

## 十、odom_imu 模式检查

| 检查项 | 当前实现 | 是否符合 | 风险 |
|---|---|---:|---|
| `T_rel_odom = T_ref^-1 * T_i` | `deskew_node.cpp:802-804` | 是 | 低 |
| 平移来自 odom | `translation() = t_rel_odom.translation()` | 是 | 低 |
| `rotation_source=imu` | 使用 `q_ref_point` | 是 | 低 |
| `rotation_source=odom` | 使用 `t_rel_odom.linear()` | 是 | 低 |
| `rotation_source=fused` | 50/50 slerp，README 说明实验性 | 是 | 中 |
| `translation_source` | 非 odom 告警并强制 odom | 是 | 低 |
| 不用 IMU acc 平移 | 强制 false | 是 | 低 |
| IMU 不可用 fallback odom | 默认 YAML `fallback_method=odom`，代码会尝试 | 是 | 低 |
| odom 不可用 fallback imu_only | 只有显式设置才可能 | 是 | 中 |
| frame 一致性 | odom 相对平移在 lidar_ref，IMU 旋转也在 lidar_ref | 基本符合 | 中：依赖 IMU TF 与 odom 外参准确 |

风险评价：主算法链路正确，但对 odom 消息 frame 的运行时保护不够强。如果输入的是 Odin1 雷达坐标系 odometry 而不是车身 `base_link` odom，`odom_imu` 会把错误平移与 IMU 旋转拼接，属于高风险配置错误。

## 十一、IMU Buffer 和积分检查

| 项目 | 当前实现 | 是否符合 | 风险 |
|---|---|---:|---|
| 缓存 stamp/angular/linear | 是，角速度和加速度已转 lidar | 是 | 低 |
| 缓存 frame_id | 不缓存 | 部分 | 低：转换后不需要，调试信息少 |
| 时间排序 | `lower_bound` 插入 | 是 | 低 |
| 清理旧数据 | 按 buffer duration 清理 | 是 | 低 |
| 查询区间 | `integrateRotation(samples, from, to)` | 是 | 低 |
| max_allowed_imu_gap | 积分 knot 和角速度插值均检查 | 是 | 低 |
| `t1 < t0` | 正向积分再 inverse | 是 | 低 |
| 缺样本 | 少于 2 或覆盖不足失败 | 是 | 低 |
| `imu_time_offset` | 查询时加 runtime offset | 是 | 低 |
| 积分公式 | 梯形平均 omega，`q = q * Exp(omega*dt)` | 是 | 低 |
| quaternion normalize | 每段 normalize | 是 | 低 |
| gyro bias | 入 buffer 前扣除 | 是 | 低 |
| gyro 单位 | 默认 rad/s，README/YAML 说明 | 是 | 低 |
| IMU frame 为空 | 使用参数 `imu_frame` | 是 | 低 |
| 静止样本不足 | warning，bias=0，清空早期 buffer | 是 | 中 |

## 十二、时间自检检查

| 检查项 | 当前代码 | 是否符合 |
|---|---|---:|
| `cloud_delay = now - cloud.header.stamp` | `recordCloudDelay()` | 是 |
| `odom_delay = now - odom.header.stamp` | `recordOdomDelay()` | 是 |
| `imu_delay = now - imu.header.stamp` | `recordImuDelay()` | 是 |
| mean/stddev/min/max/count | `DelayStatistics::calculate()` | 是 |
| time_check_window | `addSample/prune` 使用窗口 | 是 |
| min samples | cloud/odom 必需；IMU 不足时仅跳过 IMU 建议 | 是 |
| 打印推荐 odom/imu offset | `maybePrintTimeCheck()` | 是 |
| 不作为 YAML 参数 | 无 `suggested_*` 参数 | 是 |
| auto false 不应用 | 只打印 | 是 |
| auto true 只改 runtime | 改 `odom_time_offset_auto_` / `imu_time_offset_auto_` | 是 |
| max_abs | 超限 warning 且不应用 auto | 是 |
| jitter warning | stddev 超阈值 warning | 是 |
| 负延迟 warning | 三类 delay 均 warning | 是 |

变量与公式：

```text
recommended_odom_time_offset = avg_odom_delay - avg_cloud_delay
recommended_imu_time_offset  = avg_imu_delay  - avg_cloud_delay
odom_time_offset_runtime = odom_time_offset_config_ + odom_time_offset_auto_
imu_time_offset_runtime  = imu_time_offset_config_  + imu_time_offset_auto_
```

当 `apply_auto_time_offset=true`，当前实现会让 runtime offset 等于推荐总值；这与“Recommended value for xxx_time_offset”作为总配置建议一致。

## 十三、失败策略检查

| 失败场景 | 当前处理 | 是否安全 | 是否符合 README | 建议 |
|---|---|---:|---:|---|
| 缺点时间字段 | true 丢帧，false 零 offset | 是 | 是 | 无 |
| x/y/z 缺失 | failure，默认丢帧 | 是 | 是 | 无 |
| `point_time_unit` 非法 | configure 失败 | 是 | 是 | 无 |
| `cloud_stamp_type` 非法 | 启动时告警回退 scan_start | 是 | 是 | 无 |
| `deskew_method` 非法 | 启动时告警回退 odom | 是 | 是 | 无 |
| `fallback_method` 非法 | 启动时告警回退 none | 是 | 是 | 无 |
| odom buffer 不覆盖 | NoPose 失败 | 是 | 是 | 无 |
| odom gap 过大 | lookup 失败 | 是 | 是 | 无 |
| base->lidar TF 查不到 | failure，默认丢帧 | 是 | 是 | 无 |
| IMU buffer 不覆盖 | NoImu 失败，可 fallback | 是 | 是 | 无 |
| IMU gap 过大 | integrate 失败 | 是 | 是 | 无 |
| IMU TF 查不到 | IMU sample 丢弃，后续 NoImu | 是 | 是 | 可增加“当前无有效 IMU 样本”日志 |
| gyro bias 未估计 | warning，零 bias 继续 | 部分 | 是 | 实车前确保静止启动或关闭估计 |
| output xyz 写入失败 | failure，默认丢帧 | 是 | 是 | 无 |
| odom child_frame 不匹配 | 只 warning，仍入 buffer | 否 | 否 | 必改：拒绝或转换到 base_frame |
| odom header frame 不匹配 | 未检查 | 否 | 否 | 必改：检查 `header.frame_id == fixed_frame` |

## 十四、编译和静态检查

已运行：

```bash
colcon build --packages-select mid360_deskew --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test --packages-select mid360_deskew
ctest --test-dir build/mid360_deskew --output-on-failure
clang-tidy -p build/mid360_deskew src/mid360_deskew/src/*.cpp --quiet
```

结果：

- 单包编译通过。
- `colcon test` 通过。
- `ctest` 显示当前包没有测试用例。
- `clang-tidy` 退出码 0，没有输出本包源码问题。
- `CMakeLists.txt` 已启用 `-Wall -Wextra -Wpedantic`。
- `CMakeLists.txt` 正确 find `ament_cmake/rclcpp/sensor_msgs/nav_msgs/geometry_msgs/tf2/tf2_ros/tf2_eigen/Eigen3`，并安装 executable/config/launch/docs/README。
- `package.xml` 依赖齐全，当前环境构建已验证。

全工作空间命令建议在排除无关包干扰时运行：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## 十五、运行级自检命令

`docs/test_plan.md` 已包含以下命令：

```bash
ros2 topic echo /mid360/points --once
ros2 topic hz /mid360/points
ros2 topic hz /odom
ros2 topic hz /mid360/imu
ros2 topic delay /mid360/points
ros2 topic delay /odom
ros2 topic delay /mid360/imu
ros2 run tf2_ros tf2_echo base_link mid360_link
ros2 run tf2_ros tf2_echo mid360_link mid360_imu_link
ros2 run tf2_ros tf2_echo odom base_link
```

也包含 RViz 对比和三模式测试：`imu_only` 原地旋转，`odom` 边走边转，`odom_imu` 快速旋转加横移。

## 总体结论

- 符合“不是 LIO，只做 corrected PointCloud2”的主设计。
- 不会主动影响 Odin1 主定位链路，因为没有 odom/map/TF 发布。
- 不会发布 odom、map、`map->odom`。
- 编译、基础静态检查通过。
- 进入实车测试前建议先修正 odom frame 检查；否则容易把 Odin1 自身 odometry 误当车身 odom。

当前最危险的 3 个问题：

| 风险等级 | 问题 | 文件/函数 | 为什么危险 | 修改建议 |
|---|---|---|---|---|
| 高 | `child_frame_id != base_frame` 只 warning 但仍缓存 | `deskew_node.cpp:250` `PoseBuffer::addOdometry` | 误接 Odin1 雷达自身 odom 时，会把 Odin1 位姿当 base_link 位姿，违反 README 外参约束 | 不匹配时丢弃 odom，或用 TF 把 odom pose 转到 base_frame |
| 高 | 未检查 `odom.header.frame_id == fixed_frame` | `DeskewNode::odomCallback` | 可能把 map/world frame 下位姿当 odom frame 使用，影响 local_costmap 一致性 | 增加 frame 检查，不匹配则 warning+丢弃 |
| 中 | 点时间 absolute/relative 没有显式判断 | `DeskewNode::cloudCallback` | driver 字段语义配置错时，scan_start 与每点时间可能错位 | 增加字段语义参数或启发式 warning，记录 detected time field |

## 必改项

1. `odomCallback()` 中对 `msg->child_frame_id != base_frame_` 的情况不能只告警后继续使用。推荐默认丢弃该 odom，或者增加明确参数允许并执行 TF 转换。
2. `odomCallback()` 应检查 `msg->header.frame_id` 是否等于 `fixed_frame_`，不匹配默认拒绝。
3. `PoseBuffer::addPose()` 应拒绝 NaN/Inf position 和 quaternion。

## 建议项

1. 对 `scan_duration <= 0`、`scan_duration > 0.2s` 增加 warning。
2. 增加点时间字段语义日志：`timestamp` 可能是绝对时间，`offset_time` 通常是相对时间；无法判断时提示用户。
3. `publish_raw_when_failed=true` 时说明 raw 输出 stamp 仍是原始 cloud stamp，不是 t_ref。
4. 为 PoseBuffer、ImuBuffer、PointCloudFieldHelper 增加 gtest 单元测试。

## 可实车测试前检查清单

| 项目 | 必须确认 |
|---|---|
| `point_time_field` | MID360 PointCloud2 中真实存在的字段，如 `offset_time` 或 `timestamp` |
| `point_time_unit` | 与驱动字段一致，Livox 常见为 `nanosecond` |
| `cloud_stamp_type` | cloud header 表示 scan_start 还是 scan_end |
| `cloud_time_offset` | 初始 0，按 time_check 和实测调整 |
| `odom_time_offset` | 初始 0，按 time_check 和墙面旋转测试调整 |
| `imu_time_offset` | 初始 0，按 time_check 和原地旋转测试调整 |
| `base_link -> mid360_link` | TF 方向和数值准确 |
| `mid360_link -> mid360_imu_link` | TF 方向和数值准确；若 IMU 已在 lidar frame，设为 identity |
| odom frequency | 相邻间隔小于 `max_allowed_time_gap` |
| IMU frequency | 相邻间隔小于 `max_allowed_imu_gap` |
| local_costmap frame | `global_frame: odom`，MID360 不进入 global_costmap |
