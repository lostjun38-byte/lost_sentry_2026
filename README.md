# 机器人导航说明

## 框架说明

* pd2025_nav_bringup 启动包
* livox_ros_driver2 激光雷达驱动包
* autonomous_exploration_development_environment 地形分析
* small_point_lio 对Odometry、雷达点云初步处理
* loam_interface 对点云的坐标系从lidar_odom_到odom的转换,发布从odom到lidar的odometry：lidar_odometry
* sensor_scan_generation 点云的坐标系从odom到lidar的转换，发布从odom到robot_base(gimbal)的odometry:odometry
* rm_interfaces & ego_planner_interface消息定义
* pd_omni_pid_pursuit_controller 控制器插件
* cpp_lidar_filter & cpp_lidar_filter_pcd2 去除车身点云包
* small_gicp_relocalization 重定位模块
* fake_vel_transform cmd_vel速度处理，辅助路径规划
* velocity_smoother_ext 速度平滑器
* pointcloud_to_laserscan 将terrain_map_ext转换为laserScan类型以表示障碍物（仅 SLAM 模式启动）
* control_panel 控制面板
* nav2_mppi_ego_controller MPPI控制器插件
* ego_planner Ego B-spline 规划器插件
* rm_sentry_pp_nocrc_serial 上下位机通信，视觉与导航通信包

> ## 待处理问题
>
>* ego_planner状态机并不完美
>* ego_planner,mppi的参数还没有调好，没有到达理想状态
>* 地图膨胀处理存在问题
>* 在高速旋转时里程计不稳定

## 编译

```bash
CC=gcc-13 CXX=g++-13 \
CMAKE_BUILD_PARALLEL_LEVEL=4 \
MAKEFLAGS="-j4" \
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  --parallel-workers 4
```

## 导航启动

0.启动基础条件（北极熊仿真和rosbag）（这样一步只是提供导航所需的lidar、imu和tf数据，导航模块的功能和性能不受影响，
后续可以替换为其他仿真环境或实车数据）

```bash
ros2 bag play /home/lost/Downloads/rosbag2_2026_02_25-15_48_07 \
    --clock \
    --topics /livox/lidar /livox/imu \
    --remap /livox/lidar:=/red_standard_robot1/livox/lidar \
            /livox/imu:=/red_standard_robot1/livox/imu

ros2 launch pb2025_robot_description test.launch.py
```

1.建图模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=True
```

2.导航模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=rmuc_2025 \
slam:=False
```

3.打开控制面板：

```bash
ros2 run control_panel control_panel
```

4.启动决策模块：

```bash
ros2 launch rm_decision_cpp run.launch.py
```
