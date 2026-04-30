from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    frame_id = LaunchConfiguration("frame_id")
    auto_plan = LaunchConfiguration("auto_plan")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_topic = LaunchConfiguration("map_topic")
    max_vel = LaunchConfiguration("max_vel")
    max_acc = LaunchConfiguration("max_acc")
    max_jerk = LaunchConfiguration("max_jerk")
    path_sample_interval = LaunchConfiguration("path_sample_interval")
    reference_speed = LaunchConfiguration("reference_speed")
    reference_time_step = LaunchConfiguration("reference_time_step")
    terminal_slowdown_distance = LaunchConfiguration("terminal_slowdown_distance")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("auto_plan", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_topic", default_value="global_costmap/costmap"),
        DeclareLaunchArgument("max_vel", default_value="2.5"),
        DeclareLaunchArgument("max_acc", default_value="3.0"),
        DeclareLaunchArgument("max_jerk", default_value="4.0"),
        DeclareLaunchArgument("path_sample_interval", default_value="0.4"),
        DeclareLaunchArgument("reference_speed", default_value="2.0"),
        DeclareLaunchArgument("reference_time_step", default_value="0.1"),
        DeclareLaunchArgument("terminal_slowdown_distance", default_value="0.8"),
        Node(
            package="ego_planner",
            executable="motion_plan",
            name="ego_planner",
            namespace=namespace,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "map_topic": map_topic,
                "auto_plan": auto_plan,
                "frame_id": frame_id,
                "max_vel": ParameterValue(max_vel, value_type=float),
                "max_acc": ParameterValue(max_acc, value_type=float),
                "max_jerk": ParameterValue(max_jerk, value_type=float),
                "path_sample_interval": ParameterValue(path_sample_interval, value_type=float),
                "reference_speed": ParameterValue(reference_speed, value_type=float),
                "reference_time_step": ParameterValue(reference_time_step, value_type=float),
                "terminal_slowdown_distance": ParameterValue(terminal_slowdown_distance, value_type=float),
                "local_planning_horizon": 30.0,
                "local_trajectory_topic": "visual_local_trajectory",
                "ego_trajectory_topic": "/ego_reference_trajectory",
                "input_global_path_topic": "ego_planner/input_path",
                "global_path_topic": "visual_global_path",
                "astar_topic": "trajectories",
                "local_obstacles_topic": "visual_local_obstacles",
                "goal_pose_topic": "goal_pose",
                "clicked_point_topic": "clicked_point",
                "initial_pose_topic": "initialpose",
                "trigger_plan_topic": "trigger_plan",
            }],
        ),
    ])
