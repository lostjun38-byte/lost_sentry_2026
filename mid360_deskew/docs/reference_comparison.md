# Reference Comparison and Adaptation Decision

This document records the source-level comparison used to regenerate `mid360_deskew`.

## Project Constraints

- Odin1 remains the only main localization / odometry source.
- MID360 is a blind-spot obstacle sensor and only feeds Nav2 `local_costmap`.
- This package publishes only `/mid360/points_deskewed`.
- It does not publish odom, map, map->odom, TF odometry, features, keyframes, or maps.
- It does not depend on `ring`.
- It must use `base_link -> mid360_link` and, for IMU modes, `mid360_link -> mid360_imu_link`.

## Source Comparison

| Reference | Useful code/design | Not adapted | Decision in this package |
|---|---|---|---|
| `ori-drs/lidar_undistortion` | External pose driven motion compensation; pose buffer; translation lerp; quaternion slerp; `T_ref^-1 * T_i * p` | Mechanical/Ouster/Hesai-specific range image timing, reprocess buffer, PCL point templates | Adopted odom-driven per-point transform and `PoseBuffer` behavior, adapted to `nav_msgs/msg/Odometry` |
| `Livox_cloud_undistortion_ROS2` | Livox-style ego-motion undistortion; gyro integration; scan start/end target | Intensity fractional time encoding, PCL/Sophus pipeline, rotation-only demo as full pipeline | Adopted scan target concept and gyro integration idea; kept generic `PointCloud2` dynamic fields |
| `LIO-SAM` | `deskewInfo()`, `imuDeskewInfo()`, `odomDeskewInfo()`, `deskewPoint()`; explicit IMU/odom coverage checks; per-point relative time requirement | Range image, ring organization, edge/surface features, factor graph, map optimization, LIO odom | Adopted per-point time checks and coverage failure behavior only |
| `MOLA lidar odometry` | Motion-compensation method layering and fallback strategy | Full LO/LIO pipeline, ICP, MRPT/MOLA runtime, odometry outputs | Adopted `deskew_method` and `fallback_method` parameter design |
| `FAST_LIO / FAST_LIO2` | IMU buffer, LiDAR-IMU time coverage, gyro bias concept, extrinsic use, backward point compensation to scan end | iEKF, ikd-tree map, scan-to-map matching, online extrinsic estimation, odom/map publication, acceleration-based propagation | Adopted IMU buffer coverage, static gyro bias initialization, and explicit LiDAR-IMU frame rotation via TF |
| `LIO-Livox` | Livox `offset_time` in nanoseconds, IMU message interval fetch, gyro-only delta rotation before full preintegration | Ceres optimization, feature extraction, sliding-window LIO initialization, full IMU preintegration state | Adopted nanosecond point-time handling and interval coverage thinking; did not use optimization or acceleration preintegration |
| `livox_ros_driver2` | MID360 ROS2 topics and fields: `CustomPoint.offset_time`, `CustomMsg.timebase`, PointCloud2 `timestamp`, IMU topic and frame behavior | CustomMsg-specific subscription in this package | Kept input as `sensor_msgs/msg/PointCloud2`; supports `timestamp` and `offset_time`, with configurable units |

## Final Implementation Choice

The best fit for this robot is not a transplanted LIO pipeline. The best fit is a small preprocessor:

- `odom` mode: Odin1 odom supplies `T_odom_base(t)`. Points are transformed with:

```text
p_corrected =
  inverse(T_odom_mid360(t_ref))
  * T_odom_mid360(t_i)
  * p_raw
```

- `imu_only` mode: MID360 IMU gyro is rotated into `mid360_link` and integrated for rotation-only compensation. It does not compensate translation.

- `odom_imu` mode: Strategy A.

```text
T_rel_odom =
  inverse(T_odom_mid360_odom(t_ref))
  * T_odom_mid360_odom(t_i)

T_rel.translation = T_rel_odom.translation
T_rel.rotation    = R_lidar_ref_i_from_imu

p_corrected = T_rel * p_raw
```

This combines Odin1 odom translation with high-frequency MID360 gyro rotation without estimating a new global pose.

## Important Deliberate Omissions

- No iEKF, graph optimization, scan-to-map, or feature extraction.
- No IMU acceleration integration for translation in this first version.
- No online extrinsic calibration.
- No odom/map/map->odom publishing.
- No MID360 data in `global_costmap`.

## Livox Field Notes

From the local `livox_ros_driver2` source:

- `CustomPoint.msg` has `uint32 offset_time`, relative to `CustomMsg.timebase`.
- `CustomMsg.msg` has `uint64 timebase`, the time of the first point.
- The PointCloud2 path defines fields `x`, `y`, `z`, `intensity`, `tag`, `line`, and `timestamp`.
- The driver fills `timestamp` with each point timestamp in nanoseconds.
- The IMU message is published as `sensor_msgs/msg/Imu`.

Therefore this package supports both relative `offset_time` and absolute `timestamp`, keeps `point_time_unit` configurable, and normalizes point times by subtracting the minimum point time in each scan before deskew. For local `livox_ros_driver2` PointCloud2 output, use:

```yaml
point_time_field: "timestamp"
point_time_unit: "nanosecond"
```

or, if your driver exports an `offset_time` PointCloud2 field:

```yaml
point_time_field: "offset_time"
point_time_unit: "nanosecond"
```
