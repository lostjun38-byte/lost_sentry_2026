/*
MIT License

Copyright (c) 2025 Chunyu Ju

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
@Function: Ego Planner + RViz Interactive Input (Global Path + Obstacles)
@Create by: juchunyu@qq.com
@Date: 2025-11-01 18:58:01
*/
#ifndef TRAJECTORY_OBSTACLES_PUBLISHER_H
#define TRAJECTORY_OBSTACLES_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "ego_planner_msgs/msg/trajectory.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <mutex>

// Ego Planner相关头文件
#include "planner_interface.h"

using namespace ego_planner;

class TrajectoryAndObstaclesPublisher : public rclcpp::Node
{
public:
    TrajectoryAndObstaclesPublisher();
    ~TrajectoryAndObstaclesPublisher() = default;

private:
    void init_ego_planner_base();
    void publish_and_plan();
    void clearActiveGoalAndTrajectory(const std::string& reason);
    bool inputPathTimedOut(const rclcpp::Time& now) const;
    bool isCurrentTrajectoryBlocked();
    void setStopTrajectory(const PathPoint& stop_pose, const std::string& reason);
    
    // 回调函数
    void rviz_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void trigger_plan_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    // 发布函数
    void publish_global_path();
    void publish_planned_trajectory();
    void publish_a_star_path();
    void publish_local_obstacles();

    // 辅助函数
    void generate_straight_path(const geometry_msgs::msg::PoseStamped& start, 
                               const geometry_msgs::msg::PoseStamped& goal);
    std::string topic_or_default(const std::string& name, const std::string& default_value);

    void discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                           std::vector<PathPoint>& discrete_trajectory,
                           double interval = 0.4);

    double distance(const PathPoint& p1, const PathPoint& p2);
    void requestReplanLocked(const std::string& reason);

    // ROS 2发布者/订阅者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr a_star_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_traj_pub_;
    rclcpp::Publisher<ego_planner_msgs::msg::Trajectory>::SharedPtr ego_trajectory_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_local_pub_;

    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rviz_global_path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_estimate_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_plan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr plan_cb_group_;   // 独占给 publish_and_plan timer
    rclcpp::CallbackGroup::SharedPtr io_cb_group_;     // 输入 callback 共用（与 plan 解耦）

    // Ego Planner核心对象
    std::shared_ptr<PlannerInterface> ego_planner_;

    // 数据存储
    std::vector<PathPoint> global_plan_traj_;
    std::mutex data_mutex_;          // 保护本类成员（轻量元数据）
    std::mutex planner_mutex_;       // 保护 ego_planner_ 内部状态（grid_map_ / 优化器），
                                     // 只在调用 ego_planner_ 的成员方法时持有，
                                     // 与 data_mutex_ 互不嵌套（避免 AB/BA 死锁）
    bool has_valid_global_path_;
    bool should_plan_;
    bool needs_replan_;  // 新增：是否需要重新规划的标志
    bool costmap_updated_ = false;
    bool active_path_from_input_topic_ = false;
    bool has_last_input_path_time_ = false;
    bool has_last_costmap_update_time_ = false;
    bool has_last_replan_time_ = false;
    bool has_last_success_plan_time_ = false;
    bool stop_trajectory_active_ = false;
    bool map_from_costmap_ = false;
    bool has_costmap_bounds_ = false;
    bool flag_ = false;
    std::vector<PathPoint> planned_traj;
    std::vector<PathPoint> current_ego_traj_;
    std::string pending_replan_reason_;
    rclcpp::Time last_input_path_time_;
    rclcpp::Time last_costmap_update_time_;
    rclcpp::Time last_replan_time_;
    rclcpp::Time last_success_plan_time_;

    PathPoint  cur_pose_;

    vector<vector<Eigen::Vector2d>> a_star_pathes_;

    // 参数
    std::string local_trajectory_topic_ = "visual_local_trajectory";
    std::string ego_trajectory_topic_ = "/ego_reference_trajectory";
    std::string input_global_path_topic_ = "ego_planner/input_path";
    double max_vel_ = 2.0;
    double max_acc_ = 3.0;
    double max_jerk_ = 4.0;
    double path_sample_interval_ = 0.4;
    double reference_speed_ = 2.0;
    double reference_time_step_ = 0.1;
    double terminal_slowdown_distance_ = 0.8;
    double input_path_timeout_ = 3.0;
    double replan_cooldown_ = 0.2;
    double trajectory_collision_check_time_ = 1.5;
    double trajectory_collision_check_dt_ = 0.1;
    double trajectory_collision_check_distance_ = 3.0;
    double fallback_trajectory_max_age_ = 0.5;
    double make_plan_warn_time_ms_ = 100.0;
    double map_resolution_ = 0.1;
    double map_x_size_ = 50.0;
    double map_y_size_ = 50.0;
    double map_z_size_ = 10.0;
    Eigen::Vector3d map_origin_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    double map_inflate_value_ = 1.0;
    double local_planning_horizon_ = 5.0;
    int occupied_cost_threshold_ = 50;

    bool auto_plan_ = true;
    std::string frame_id_ = "map";
    std::string map_topic_ = "global_costmap/costmap";
    std::string costmap_frame_id_;
    double costmap_min_x_ = 0.0;
    double costmap_max_x_ = 0.0;
    double costmap_min_y_ = 0.0;
    double costmap_max_y_ = 0.0;
    double costmap_resolution_ = 0.0;
};

#endif // TRAJECTORY_OBSTACLES_PUBLISHER_H
