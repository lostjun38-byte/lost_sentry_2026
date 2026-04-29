from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    frame_id = LaunchConfiguration("frame_id")
    auto_plan = LaunchConfiguration("auto_plan")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_topic = LaunchConfiguration("map_topic")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("auto_plan", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_topic", default_value="global_costmap/costmap"),
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
                "local_planning_horizon": 5.0,
                "local_trajectory_topic": "visual_local_trajectory",
                "ego_trajectory_topic": "/ego_reference_trajectory",
                "input_global_path_topic": "ego_planner/input_path",
                "global_path_topic": "visual_global_path",
                "astar_topic": "trajectories",
                "obstacles_topic": "visual_obstacles",
                "local_obstacles_topic": "visual_local_obstacles",
                "goal_pose_topic": "goal_pose",
                "clicked_point_topic": "clicked_point",
                "initial_pose_topic": "initialpose",
                "trigger_plan_topic": "trigger_plan",
            }],
        ),
    ])
