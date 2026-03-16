# 机器人导航说明

## 框架说明

* rm_bringup/pd2025_nav_bringup 启动包
* livox_laser_simulation_RO2 Livox激光雷达仿真包（ROS2 版本）
* livox_ros_driver2 激光雷达驱动包
* autonomous_exploration_development_environment 地形分析
* small_point_lio 对Odometry、雷达点云初步处理
* loam_interface 对点云的坐标系从lidar_odom_到odom的转换,发布从odom到lidar的odometry：lidar_odometry
* sensor_scan_generation 点云的坐标系从odom到lidar的转换，发布从odom到base_footprint的odometry:odometry
* rm_decision_cpp 决策模块
* auto_aim_interfaces & rm_interfaces 消息定义
* pd_omni_pid_pursuit_controller 控制器插件
* ign_sim_pointcloud_tool 仿真点云处理工具
* small_gicp_relocalization 重定位模块
* fake_vel_transform cmd_vel速度处理，辅助路径规划
* velocity_smoother_ext 速度平滑器
* pointcloud_to_laserscan 将terrain_map_ext转换为laserScan类型以表示障碍物（仅 SLAM 模式启动）
* control_panel 控制面板

> ## 待处理问题
>
> * colcon build太麻烦   //暂时解决方法：colcon build --cmake-args -DROS_EDITION=ros2 -DHUMBLE_ROS=humble
> * 机器人自己移动，在场地交互模块中移动易发生漂移 //解决方法：发现地图倾斜，将地图模块修正
> * 盲目拼接导致的接口、话题名字对不上，参数不准
> * 潜在的功能重复问题
> * 功能没有创新，只是对北极熊，火锅代码的复制拼接，没有寻找其他资源
> * 无仿真环境还没有搞清楚
> * 遇到坡上不去，有时候卡壳  //解决方法：速度平滑器的参数一不小心填错了，改正即可

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
