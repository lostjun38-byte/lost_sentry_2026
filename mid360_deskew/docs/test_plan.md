# mid360_deskew Test Plan

This package only publishes corrected MID360 `PointCloud2` for Nav2 `local_costmap`. It must not publish odom, map, or map->odom.

## 1. Field Check

```bash
ros2 topic echo /mid360/points --once
```

Confirm that `x`, `y`, `z` exist and that one per-point time field exists:

- `offset_time`
- `time`
- `timestamp`
- `t`

For `livox_ros_driver2` PointCloud2 output, the common field is `timestamp` as a `FLOAT64` carrying point timestamp in nanoseconds. For `CustomMsg`, `offset_time` is relative to `timebase`. This node normalizes point times by subtracting the minimum point time inside each scan, so either form can be used if the unit is configured correctly.

### CustomMsg Input Check

```bash
ros2 topic info /livox/lidar
```

Expected:

```text
Type: livox_ros_driver2/msg/CustomMsg
```

Inspect one message:

```bash
ros2 topic echo /livox/lidar --once
```

Check:

- `header.stamp`
- `timebase`
- `point_num`
- `points[0].offset_time`
- the last point `offset_time`

If `/mid360/points` does not include `offset_time`, `time`, `timestamp`, or `t`, use `cloud_input_type: "custom_msg"` instead of forcing PointCloud2 deskew.

## 2. Frequency Check

```bash
ros2 topic hz /odom
ros2 topic hz /mid360/points
ros2 topic hz /mid360/imu
```

Odom intervals must stay below `max_allowed_time_gap`. IMU intervals must stay below `max_allowed_imu_gap` when using `imu_only` or `odom_imu`.

## 3. Delay Check

```bash
ros2 topic delay /odom
ros2 topic delay /mid360/points
ros2 topic delay /mid360/imu
```

Compare with the node logs:

- `avg_cloud_delay`, `std_cloud_delay`
- `avg_odom_delay`, `std_odom_delay`
- `avg_imu_delay`, `std_imu_delay`
- `Recommended value for odom_time_offset`
- `Recommended value for imu_time_offset`

The recommendation only compares message `header.stamp` delays against ROS `now()`. It is not hardware time synchronization.

## 4. TF Check

```bash
ros2 run tf2_ros tf2_echo base_link mid360_link
ros2 run tf2_ros tf2_echo mid360_link mid360_imu_link
ros2 run tf2_ros tf2_echo odom base_link
```

Required tree:

```text
odom
  base_link
    odin1_link
    mid360_link
      mid360_imu_link
```

The node uses `T_odom_base(t)` from `/odom`, `T_base_mid360` from TF, and rotates IMU gyro into `mid360_link` before integration. Do not treat Odin1 and MID360 as the same physical pose.

The expected odom message semantics are `T_fixed_base(t)`: `odom.header.frame_id` should match `fixed_frame`, and `odom.child_frame_id` should match `base_frame`.

## 5. Three Mode Tests

### A. imu_only

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_imu_only.yaml
```

Rotate the robot in place quickly. Rotation smear should improve. Side translation may still be wrong because `imu_only` does not compensate translation.

### B. odom

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_odom.yaml
```

Move and rotate the robot. This mode should compensate translation and rotation if `/odom` is frequent enough and time-aligned with MID360.

### C. odom_imu

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_odom_imu.yaml
```

Move and rotate quickly. This mode should be more robust to high-frequency rotation than `odom`, and better for translation than `imu_only`.

### D. CustomMsg odom

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_custom_odom.yaml
```

Confirm from node startup log that `cloud_input_type=custom_msg`. It should subscribe to `custom_cloud_topic` and not subscribe to the PointCloud2 input topic.

### E. CustomMsg odom_imu

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_custom_odom_imu.yaml
```

Use this when odom gives translation and MID360 IMU covers the full scan interval.

### F. CustomMsg imu_only

```bash
ros2 launch mid360_deskew mid360_deskew.launch.py \
  config_file:=/home/lost/nav_1_ws/src/mid360_deskew/config/mid360_deskew_custom_imu_only.yaml
```

Use this only for rotation-smear validation; it does not compensate translation.

## 6. IMU Rotation Direction Bag Test

Record or replay a short bag with the robot rotating in place clockwise and counter-clockwise. Compare:

- raw MID360 cloud
- `imu_only`
- `odom`
- `odom_imu`

Keep `imu_rotation_sign: 1.0` first. If `imu_only` is visibly worse than raw during in-place rotation, temporarily set:

```yaml
imu_rotation_sign: -1.0
```

Then rerun the same bag. If `-1.0` improves the cloud, inspect quaternion direction, gyro frame transform, and whether the code is applying `q_i_to_ref` or `q_ref_to_i` in the expected direction. The debug log prints gyro bias, integration interval, relative quaternion, and yaw delta for `imu_only` / `odom_imu`.

## 7. RViz Comparison

Display both topics:

- `/mid360/points`
- `/livox/lidar` when using CustomMsg input
- `/mid360/points_deskewed`

Use `odom` as RViz fixed frame for local motion inspection. The deskewed output frame should be `mid360_link`.

## 8. If Corrected Looks Worse

Check these first:

a. `point_time_unit` is wrong.

a2. For CustomMsg, `custom_msg_offset_unit` is wrong.

b. `cloud_stamp_type` is wrong.

c. `odom_time_offset` is wrong.

d. `imu_time_offset` is wrong.

e. `base_link -> mid360_link` extrinsic is wrong.

f. `mid360_imu_link -> mid360_link` extrinsic is wrong.

g. Gyro bias was not estimated because the robot moved during startup.

h. IMU angular velocity units or axes are wrong.

h2. IMU quaternion direction is reversed; compare `imu_rotation_sign: 1.0` and `-1.0`.

i. Odin1 and MID360 do not share a stable time base.

i2. `custom_msg_timebase_type: "custom_timebase"` is enabled but Livox timebase is not synchronized with ROS odom/IMU time.

j. A test or costmap uses `map` instead of `odom`.

k. Odom frequency is too low, or IMU samples have gaps larger than `max_allowed_imu_gap`.
