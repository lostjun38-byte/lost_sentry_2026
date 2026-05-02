# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    ego_planner_dir = get_package_share_directory("ego_planner")
    mid360_deskew_dir = get_package_share_directory("mid360_deskew")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_ego_planner = LaunchConfiguration("use_ego_planner")
    ego_frame_id = LaunchConfiguration("ego_frame_id")
    ego_map_topic = LaunchConfiguration("ego_map_topic")
    ego_max_vel = LaunchConfiguration("ego_max_vel")
    ego_max_acc = LaunchConfiguration("ego_max_acc")
    ego_max_jerk = LaunchConfiguration("ego_max_jerk")
    ego_path_sample_interval = LaunchConfiguration("ego_path_sample_interval")
    ego_reference_speed = LaunchConfiguration("ego_reference_speed")
    ego_reference_time_step = LaunchConfiguration("ego_reference_time_step")
    ego_terminal_slowdown_distance = LaunchConfiguration("ego_terminal_slowdown_distance")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmuc_2025",
        description="Select world: 'rmul_2024' or 'rmuc_2024' or 'rmul_2025' or 'rmuc_2025' (map file share the same name as the this parameter)",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "map", "simulation", "")),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to map file to load",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "pcd", "simulation", "")),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="Full path to prior pcd file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if True",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "simulation", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_use_ego_planner_cmd = DeclareLaunchArgument(
        "use_ego_planner",
        default_value="true",
        description="Whether to start ego_planner reference trajectory publisher",
    )

    declare_ego_frame_id_cmd = DeclareLaunchArgument(
        "ego_frame_id",
        default_value="map",
        description="Frame used by ego_planner published paths and markers",
    )

    declare_ego_map_topic_cmd = DeclareLaunchArgument(
        "ego_map_topic",
        default_value="global_costmap/costmap",
        description="OccupancyGrid topic consumed by ego_planner",
    )

    declare_ego_max_vel_cmd = DeclareLaunchArgument(
        "ego_max_vel",
        default_value="2.5",
        description="Maximum reference speed used by ego_planner",
    )

    declare_ego_max_acc_cmd = DeclareLaunchArgument(
        "ego_max_acc",
        default_value="3.0",
        description="Maximum reference acceleration used by ego_planner",
    )

    declare_ego_max_jerk_cmd = DeclareLaunchArgument(
        "ego_max_jerk",
        default_value="4.0",
        description="Maximum reference jerk used by ego_planner",
    )

    declare_ego_path_sample_interval_cmd = DeclareLaunchArgument(
        "ego_path_sample_interval",
        default_value="0.4",
        description="Path resampling interval used before ego_planner B-spline generation",
    )

    declare_ego_reference_speed_cmd = DeclareLaunchArgument(
        "ego_reference_speed",
        default_value="2.0",
        description="Time-parameterized reference speed published for MPPI EgoTrajectoryCritic",
    )

    declare_ego_reference_time_step_cmd = DeclareLaunchArgument(
        "ego_reference_time_step",
        default_value="0.1",
        description="Time step used by ego_planner reference trajectory message",
    )

    declare_ego_terminal_slowdown_distance_cmd = DeclareLaunchArgument(
        "ego_terminal_slowdown_distance",
        default_value="0.8",
        description="Distance before local ego reference end where published reference speed tapers to zero",
    )

    start_velodyne_convert_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="ign_sim_pointcloud_tool",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    ego_planner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ego_planner_dir, "launch", "ego_planner.launch.py")
        ),
        condition=IfCondition(use_ego_planner),
        launch_arguments={
            "namespace": namespace,
            "frame_id": ego_frame_id,
            "use_sim_time": use_sim_time,
            "map_topic": ego_map_topic,
            "max_vel": ego_max_vel,
            "max_acc": ego_max_acc,
            "max_jerk": ego_max_jerk,
            "path_sample_interval": ego_path_sample_interval,
            "reference_speed": ego_reference_speed,
            "reference_time_step": ego_reference_time_step,
            "terminal_slowdown_distance": ego_terminal_slowdown_distance,
        }.items(),
    )

    mid360_deskew_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mid360_deskew_dir, "launch", "mid360_deskew.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "config_file": os.path.join(mid360_deskew_dir, "config", "mid360_deskew_odom_imu.yaml"),
        }.items(),
    )

    joy_teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "joy_teleop_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "joy_config_file": params_file,
        }.items(),
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_ego_planner_cmd)
    ld.add_action(declare_ego_frame_id_cmd)
    ld.add_action(declare_ego_map_topic_cmd)
    ld.add_action(declare_ego_max_vel_cmd)
    ld.add_action(declare_ego_max_acc_cmd)
    ld.add_action(declare_ego_max_jerk_cmd)
    ld.add_action(declare_ego_path_sample_interval_cmd)
    ld.add_action(declare_ego_reference_speed_cmd)
    ld.add_action(declare_ego_reference_time_step_cmd)
    ld.add_action(declare_ego_terminal_slowdown_distance_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch all of the navigation nodes
    # ld.add_action(start_velodyne_convert_tool)
    ld.add_action(bringup_cmd)
    ld.add_action(ego_planner_cmd)
    ld.add_action(joy_teleop_cmd)
    ld.add_action(rviz_cmd)
    # ld.add_action(mid360_deskew_cmd)
    return ld
