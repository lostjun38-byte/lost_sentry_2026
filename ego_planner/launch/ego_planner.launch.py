import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("ego_planner"),
                "config",
                "ego_planner.yaml",
            ),
            description="Full path to the ego_planner parameter file",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="ego_planner",
            executable="motion_plan",
            name="ego_planner",
            namespace=namespace,
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
    ])
