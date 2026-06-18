/*
 * @Function: Ego Planner + RViz Interactive Input (Global Path + Obstacles)
 * @Create by: juchunyu@qq.com
 * @Date: 2025-11-01 18:58:01
 */
#include "trajectory_obstacles_publisher.h"
#include <algorithm>
#include <tf2/utils.h>

namespace
{

double yawFromTrajectoryPoint(const std::vector<PathPoint> & trajectory, size_t index)
{
    if (trajectory.size() < 2) {
        return 0.0;
    }

    const size_t prev_index = index > 0 ? index - 1 : index;
    const size_t next_index = index + 1 < trajectory.size() ? index + 1 : index;
    const double dx = trajectory[next_index].x - trajectory[prev_index].x;
    const double dy = trajectory[next_index].y - trajectory[prev_index].y;
    return std::hypot(dx, dy) > 1e-4 ? std::atan2(dy, dx) : 0.0;
}

}  // namespace

TrajectoryAndObstaclesPublisher::TrajectoryAndObstaclesPublisher() 
    : Node("ego_planner_interactive_node"),
      has_valid_global_path_(false),
      should_plan_(true),
      needs_replan_(false)
{
    this->declare_parameter("auto_plan", auto_plan_);
    this->declare_parameter("frame_id", frame_id_);
    this->declare_parameter("map_topic", map_topic_);
    this->declare_parameter("local_planning_horizon", local_planning_horizon_);
    this->declare_parameter("max_vel", max_vel_);
    this->declare_parameter("max_acc", max_acc_);
    this->declare_parameter("max_jerk", max_jerk_);
    this->declare_parameter("path_sample_interval", path_sample_interval_);
    this->declare_parameter("reference_speed", reference_speed_);
    this->declare_parameter("reference_time_step", reference_time_step_);
    this->declare_parameter("terminal_slowdown_distance", terminal_slowdown_distance_);
    this->declare_parameter("input_path_timeout", input_path_timeout_);
    this->declare_parameter("replan_cooldown", replan_cooldown_);
    this->declare_parameter("trajectory_collision_check_time", trajectory_collision_check_time_);
    this->declare_parameter("trajectory_collision_check_dt", trajectory_collision_check_dt_);
    this->declare_parameter("trajectory_collision_check_distance", trajectory_collision_check_distance_);
    this->declare_parameter("fallback_trajectory_max_age", fallback_trajectory_max_age_);
    this->declare_parameter("make_plan_warn_time_ms", make_plan_warn_time_ms_);
    this->declare_parameter("occupied_cost_threshold", occupied_cost_threshold_);
    this->get_parameter("auto_plan", auto_plan_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("map_topic", map_topic_);
    this->get_parameter("local_planning_horizon", local_planning_horizon_);
    this->get_parameter("max_vel", max_vel_);
    this->get_parameter("max_acc", max_acc_);
    this->get_parameter("max_jerk", max_jerk_);
    this->get_parameter("path_sample_interval", path_sample_interval_);
    this->get_parameter("reference_speed", reference_speed_);
    this->get_parameter("reference_time_step", reference_time_step_);
    this->get_parameter("terminal_slowdown_distance", terminal_slowdown_distance_);
    this->get_parameter("input_path_timeout", input_path_timeout_);
    this->get_parameter("replan_cooldown", replan_cooldown_);
    this->get_parameter("trajectory_collision_check_time", trajectory_collision_check_time_);
    this->get_parameter("trajectory_collision_check_dt", trajectory_collision_check_dt_);
    this->get_parameter("trajectory_collision_check_distance", trajectory_collision_check_distance_);
    this->get_parameter("fallback_trajectory_max_age", fallback_trajectory_max_age_);
    this->get_parameter("make_plan_warn_time_ms", make_plan_warn_time_ms_);
    this->get_parameter("occupied_cost_threshold", occupied_cost_threshold_);
    local_planning_horizon_ = std::max(local_planning_horizon_, 0.0);
    max_vel_ = std::max(max_vel_, 1e-3);
    max_acc_ = std::max(max_acc_, 1e-3);
    max_jerk_ = std::max(max_jerk_, 1e-3);
    path_sample_interval_ = std::max(path_sample_interval_, 1e-3);
    reference_speed_ = std::clamp(reference_speed_, 1e-3, max_vel_);
    reference_time_step_ = std::max(reference_time_step_, 1e-3);
    terminal_slowdown_distance_ = std::max(terminal_slowdown_distance_, 0.0);
    input_path_timeout_ = std::max(input_path_timeout_, 0.0);
    replan_cooldown_ = std::max(replan_cooldown_, 0.0);
    trajectory_collision_check_time_ = std::max(trajectory_collision_check_time_, 0.0);
    trajectory_collision_check_dt_ = std::max(trajectory_collision_check_dt_, 1e-3);
    trajectory_collision_check_distance_ = std::max(trajectory_collision_check_distance_, 0.0);
    fallback_trajectory_max_age_ = std::max(fallback_trajectory_max_age_, 0.0);
    make_plan_warn_time_ms_ = std::max(make_plan_warn_time_ms_, 0.0);
    should_plan_ = auto_plan_;

    const auto global_path_topic = topic_or_default("global_path_topic", "visual_global_path");
    const auto astar_topic = topic_or_default("astar_topic", "trajectories");
    local_trajectory_topic_ = topic_or_default("local_trajectory_topic", local_trajectory_topic_);
    ego_trajectory_topic_ = topic_or_default("ego_trajectory_topic", ego_trajectory_topic_);
    input_global_path_topic_ = topic_or_default("input_global_path_topic", input_global_path_topic_);
    const auto local_obstacles_topic = topic_or_default("local_obstacles_topic", "visual_local_obstacles");
    const auto goal_pose_topic = topic_or_default("goal_pose_topic", "goal_pose");
    const auto clicked_point_topic = topic_or_default("clicked_point_topic", "clicked_point");
    const auto initial_pose_topic = topic_or_default("initial_pose_topic", "initialpose");
    const auto trigger_plan_topic = topic_or_default("trigger_plan_topic", "trigger_plan");
    const auto map_topic = map_topic_;

    // 1. 创建发布者（可视化用）
    global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(global_path_topic, 10);
    // a_star_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_astar_path", 10);
    a_star_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(astar_topic, 10);
    local_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>(local_trajectory_topic_, 10);
    ego_trajectory_pub_ = this->create_publisher<ego_planner_msgs::msg::Trajectory>(ego_trajectory_topic_, 10);
    obs_local_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(local_obstacles_topic, 10);

    // 1.5 创建 callback group：把"重活定时器"和"输入 callback"分到不同组，
    //     这样即使 publish_and_plan 卡在 makePlan 里，输入 callback 仍可被调度
    //     （MultiThreadedExecutor 默认每个 callback 进 MutuallyExclusive group，会串行）。
    plan_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);   // 定时器自身不重入
    io_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);           // 输入 callback 彼此可并行

    rclcpp::SubscriptionOptions io_opts;
    io_opts.callback_group = io_cb_group_;

    // 2. 创建订阅者（接收RViz下发的数据）
    rviz_global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        input_global_path_topic_,
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::rviz_global_path_callback, this, std::placeholders::_1),
        io_opts
    );

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic,
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::goal_pose_callback, this, std::placeholders::_1),
        io_opts
    );

    // 路径添加：使用Publish Point工具逐个添加
    rviz_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        clicked_point_topic,
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::rviz_point_callback, this, std::placeholders::_1),
        io_opts
    );

    // 使用 2D Pose Estimate 更新当前位姿
    pose_estimate_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic,
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::pose_estimate_callback, this, std::placeholders::_1),
        io_opts
    );

    // 3. 触发规划的话题
    trigger_plan_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        trigger_plan_topic,
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::trigger_plan_callback, this, std::placeholders::_1),
        io_opts
    );

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic,
        rclcpp::QoS(10),
        std::bind(&TrajectoryAndObstaclesPublisher::costmap_callback, this, std::placeholders::_1),
        io_opts
    );

    // 4. 初始化Ego Planner基础配置
    init_ego_planner_base();

    // 5. 定时器：5Hz触发规划与发布
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&TrajectoryAndObstaclesPublisher::publish_and_plan, this),
        plan_cb_group_
    );

    RCLCPP_INFO(this->get_logger(), "Interactive Ego Planner Node Ready!");
    RCLCPP_INFO(this->get_logger(), "=== 使用方法 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加全局路径：使用2D Nav Goal工具设置终点");
    RCLCPP_INFO(this->get_logger(), "2. 触发规划：ros2 topic pub /trigger_plan std_msgs/Bool \"{data: true}\"");
    RCLCPP_INFO(this->get_logger(), "3. 停止规划：ros2 topic pub /trigger_plan std_msgs/Bool \"{data: false}\"");
    RCLCPP_INFO(this->get_logger(), "4. 支持多次规划：可以反复发送true/false控制规划");
    RCLCPP_INFO(this->get_logger(), "5. 地图输入话题：%s", map_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "6. Nav2全局路径输入话题：%s", input_global_path_topic_.c_str());
    RCLCPP_INFO(
        this->get_logger(),
        "7. Ego速度参数: max_vel=%.3f, max_acc=%.3f, max_jerk=%.3f, "
        "path_sample_interval=%.3f, reference_speed=%.3f, reference_time_step=%.3f",
        max_vel_, max_acc_, max_jerk_, path_sample_interval_, reference_speed_, reference_time_step_);
    RCLCPP_INFO(
        this->get_logger(),
        "8. Ego动态重规划参数: local_horizon=%.3f, input_path_timeout=%.3f, "
        "replan_cooldown=%.3f, collision_check_time=%.3f, collision_check_dt=%.3f, "
        "collision_check_distance=%.3f, fallback_trajectory_max_age=%.3f, occupied_cost_threshold=%d",
        local_planning_horizon_,
        input_path_timeout_,
        replan_cooldown_,
        trajectory_collision_check_time_,
        trajectory_collision_check_dt_,
        trajectory_collision_check_distance_,
        fallback_trajectory_max_age_,
        occupied_cost_threshold_);
    RCLCPP_INFO(this->get_logger(), "=== RViz2设置步骤 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加Grid显示");
    RCLCPP_INFO(this->get_logger(), "2. 添加2D Nav Goal工具（话题：/goal_pose）");
    RCLCPP_INFO(this->get_logger(), "3. 添加2D Pose Estimate工具（使用默认话题：/initialpose）");
    RCLCPP_INFO(this->get_logger(), "4. 添加Publish Point工具（使用默认话题：/clicked_point）");
    RCLCPP_INFO(this->get_logger(), "5. 添加PointCloud2显示（话题：/visual_local_obstacles）");
    RCLCPP_INFO(this->get_logger(), "6. 添加Path显示（话题：/visual_global_path和/visual_local_trajectory）");
    RCLCPP_INFO(this->get_logger(), "7. Ego参考轨迹输出话题：%s", ego_trajectory_topic_.c_str());
}

std::string TrajectoryAndObstaclesPublisher::topic_or_default(
    const std::string& name, const std::string& default_value)
{
    this->declare_parameter(name, default_value);
    std::string value;
    this->get_parameter(name, value);
    return value;
}

// 初始化Ego Planner基础参数
void TrajectoryAndObstaclesPublisher::init_ego_planner_base()
{
    ego_planner_ = std::make_shared<PlannerInterface>();
    ego_planner_->initParam(max_vel_, max_acc_, max_jerk_);
    ego_planner_->initEsdfMap(
        map_x_size_, map_y_size_, map_z_size_,
        map_resolution_, map_origin_, map_inflate_value_
    );
}

void TrajectoryAndObstaclesPublisher::requestReplanLocked(const std::string& reason)
{
    needs_replan_ = true;
    pending_replan_reason_ = reason;
}

bool TrajectoryAndObstaclesPublisher::inputPathTimedOut(const rclcpp::Time& now) const
{
    return active_path_from_input_topic_ &&
        has_last_input_path_time_ &&
        input_path_timeout_ > 0.0 &&
        (now - last_input_path_time_).seconds() > input_path_timeout_;
}

void TrajectoryAndObstaclesPublisher::clearActiveGoalAndTrajectory(const std::string& reason)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        has_valid_global_path_ = false;
        should_plan_ = false;
        needs_replan_ = false;
        costmap_updated_ = false;
        active_path_from_input_topic_ = false;
        has_last_input_path_time_ = false;
        has_last_replan_time_ = false;
        has_last_success_plan_time_ = false;
        stop_trajectory_active_ = false;
        pending_replan_reason_.clear();
        global_plan_traj_.clear();
        current_ego_traj_.clear();
        planned_traj.clear();
        a_star_pathes_.clear();
    }

    {
        std::lock_guard<std::mutex> plock(planner_mutex_);
        ego_planner_->clearPlanTrajResults();
    }

    RCLCPP_INFO(this->get_logger(), "清理 Ego active goal 和轨迹缓存: reason=%s", reason.c_str());
}

void TrajectoryAndObstaclesPublisher::setStopTrajectory(const PathPoint& stop_pose, const std::string& reason)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    PathPoint stop_point = stop_pose;
    stop_point.v = 0.0F;
    current_ego_traj_.clear();
    current_ego_traj_.push_back(stop_point);
    current_ego_traj_.push_back(stop_point);
    stop_trajectory_active_ = true;
    RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "发布停车轨迹: reason=%s, pos=(%.3f, %.3f)",
        reason.c_str(),
        stop_point.x,
        stop_point.y);
}

bool TrajectoryAndObstaclesPublisher::isCurrentTrajectoryBlocked()
{
    std::vector<PathPoint> trajectory;
    bool has_bounds = false;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        trajectory = current_ego_traj_;
        has_bounds = has_costmap_bounds_;
        min_x = costmap_min_x_;
        max_x = costmap_max_x_;
        min_y = costmap_min_y_;
        max_y = costmap_max_y_;
    }

    if (!has_bounds || trajectory.size() < 2) {
        return false;
    }

    const double max_check_time = trajectory_collision_check_time_;
    const double max_check_distance = trajectory_collision_check_distance_;
    const double point_dt = std::max(reference_time_step_, trajectory_collision_check_dt_);
    double accumulated_distance = 0.0;
    PathPoint prev = trajectory.front();

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& point = trajectory[i];
        const double t = static_cast<double>(i) * point_dt;
        if (max_check_time > 0.0 && t > max_check_time) {
            break;
        }

        if (i > 0) {
            accumulated_distance += distance(prev, point);
            prev = point;
            if (max_check_distance > 0.0 && accumulated_distance > max_check_distance) {
                break;
            }
        }

        if (point.x < min_x || point.x >= max_x || point.y < min_y || point.y >= max_y) {
            // Rolling costmap 外的点不能在 blocked-check 阶段直接触发重规划。
            continue;
        }

        bool occupied = false;
        {
            std::lock_guard<std::mutex> plock(planner_mutex_);
            occupied = ego_planner_->isWorldPointOccupied(point.x, point.y);
        }

        if (occupied) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "当前 Ego 轨迹短窗口被阻断: t=%.3f, dist=%.3f, pos=(%.3f, %.3f), reason=inflated_occupancy",
                t,
                accumulated_distance,
                point.x,
                point.y);
            return true;
        }
    }

    return false;
}

// 2D Pose Estimate回调函数
void TrajectoryAndObstaclesPublisher::pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
   // 1. 更新机器人初始位姿（原有逻辑）
    cur_pose_.x = msg->pose.pose.position.x;
    cur_pose_.y = msg->pose.pose.position.y;
    cur_pose_.z = 0;
    // 计算偏航角（使用tf2或手动计算，确保正确）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    cur_pose_.z = tf2::getYaw(q);
    std::cout << "pose_estimate_callback " << std::endl;
    // 2. 触发规划逻辑（新增）
    if (has_valid_global_path_) {  // 确保已有全局路径
        requestReplanLocked("pose_update");
        if (should_plan_) {
            RCLCPP_INFO(this->get_logger(), "初始位置更新，触发重新规划！");
        } else {
            RCLCPP_WARN(this->get_logger(), "初始位置已更新，但规划未启动！请先发送 /trigger_plan true 启动规划");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "初始位置已更新，但无有效全局路径，无法规划！");
    }
}

// 触发规划话题回调
void TrajectoryAndObstaclesPublisher::trigger_plan_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) {
        clearActiveGoalAndTrajectory("trigger_plan_false");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        should_plan_ = true;
        if (has_valid_global_path_) {
            requestReplanLocked("manual_trigger");
            RCLCPP_INFO(this->get_logger(), "规划已触发! 路径点: %zu", global_plan_traj_.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "无法触发规划: 没有有效的全局路径!");
        }
        flag_ = true;
    }
}

void TrajectoryAndObstaclesPublisher::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // —— 1. 仅在 data_mutex_ 下写元数据；不持有该锁调入 ego_planner_，
    //       否则费时的 setNav2InflatedOccupancyGridMap 会和 makePlan 的锁串成死锁链。
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        map_from_costmap_ = true;
        has_costmap_bounds_ = msg->info.width > 0 && msg->info.height > 0 && msg->info.resolution > 0.0;
        costmap_frame_id_ = msg->header.frame_id;
        costmap_resolution_ = msg->info.resolution;
        costmap_min_x_ = msg->info.origin.position.x;
        costmap_min_y_ = msg->info.origin.position.y;
        costmap_max_x_ = costmap_min_x_ + static_cast<double>(msg->info.width) * msg->info.resolution;
        costmap_max_y_ = costmap_min_y_ + static_cast<double>(msg->info.height) * msg->info.resolution;
        costmap_updated_ = true;
        last_costmap_update_time_ = this->now();
        has_last_costmap_update_time_ = true;

        if (!flag_) {
            flag_ = true;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "收到 OccupancyGrid 地图: %u x %u, resolution=%.3f, origin=(%.3f, %.3f), "
            "bounds_x=[%.3f, %.3f), bounds_y=[%.3f, %.3f), frame=%s, topic=%s",
            msg->info.width,
            msg->info.height,
            msg->info.resolution,
            msg->info.origin.position.x,
            msg->info.origin.position.y,
            costmap_min_x_,
            costmap_max_x_,
            costmap_min_y_,
            costmap_max_y_,
            msg->header.frame_id.c_str(),
            map_topic_.c_str());
    }

    // —— 2. 把 OccupancyGrid 写入 ego_planner_->grid_map_：与 makePlan / getObstacles 互斥，
    //       但不会和 data_mutex_ 互锁。
    {
        std::lock_guard<std::mutex> plock(planner_mutex_);
        ego_planner_->setNav2InflatedOccupancyGridMap(*msg, occupied_cost_threshold_);
    }
}

// 处理2D Nav Goal - 生成从起点到目标的直线路
void TrajectoryAndObstaclesPublisher::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    geometry_msgs::msg::PoseStamped start;
    start.pose.position.x = cur_pose_.x;
    start.pose.position.y = cur_pose_.y;
    start.pose.position.z = 0.0;
    generate_straight_path(start, *msg);
    has_valid_global_path_ = true;
    active_path_from_input_topic_ = false;
    if (should_plan_) {
        requestReplanLocked("new_goal_pose");
    }
    RCLCPP_INFO(
        this->get_logger(), "收到目标点，生成 Ego 全局参考路径: %zu 点", global_plan_traj_.size());
}

// 生成直线路径
void TrajectoryAndObstaclesPublisher::generate_straight_path(const geometry_msgs::msg::PoseStamped& start, 
                                                           const geometry_msgs::msg::PoseStamped& goal)
{
    global_plan_traj_.clear();

    // 计算路径点数量（每0.1米一个点）
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    int num_points = std::max(2, static_cast<int>(distance / 0.1));

    // 生成直线路径点
    for (int i = 0; i < num_points; ++i) {
        double ratio = static_cast<double>(i) / (num_points - 1);
        PathPoint point;
        point.x = start.pose.position.x + ratio * dx;
        point.y = start.pose.position.y + ratio * dy;
        point.z = 0.0;
        global_plan_traj_.push_back(point);
    }
}

// 处理 Publish Point 点击 - 添加全局路径点
void TrajectoryAndObstaclesPublisher::rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    // global_plan_traj_.clear();
    PathPoint point;
    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = 0; // 使用z作为theta角度
    global_plan_traj_.push_back(point);
    std::cout << "从RViz接收到全局路径点: (" << point.x << ", " << point.y << ")" << std::endl;
    has_valid_global_path_ = true;
    active_path_from_input_topic_ = false;
    
    // 如果有路径更新且正在规划中，则标记需要重新规划
    if (should_plan_) 
    {
        requestReplanLocked("manual_path_point");
        RCLCPP_INFO(this->get_logger(), "路径更新，已标记需要重新规划");
    }
}

// 原有的全局路径回调
void TrajectoryAndObstaclesPublisher::rviz_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "从RViz接收到空的全局路径!");
        clearActiveGoalAndTrajectory("empty_input_path");
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    global_plan_traj_.clear();
    for (const auto& pose_stamped : msg->poses)
    {
        PathPoint path_point;
        path_point.x = pose_stamped.pose.position.x;
        path_point.y = pose_stamped.pose.position.y;
        path_point.z = pose_stamped.pose.position.z;
        global_plan_traj_.push_back(path_point);
    }

    if (has_costmap_bounds_) {
        size_t out_of_map_count = 0;
        PathPoint first_out_of_map;
        bool has_first_out_of_map = false;
        for (const auto& path_point : global_plan_traj_) {
            const bool out_of_bounds =
                path_point.x < costmap_min_x_ || path_point.x >= costmap_max_x_ ||
                path_point.y < costmap_min_y_ || path_point.y >= costmap_max_y_;
            if (out_of_bounds) {
                ++out_of_map_count;
                if (!has_first_out_of_map) {
                    first_out_of_map = path_point;
                    has_first_out_of_map = true;
                }
            }
        }

        if (!costmap_frame_id_.empty() && !msg->header.frame_id.empty() &&
            msg->header.frame_id != costmap_frame_id_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Ego 全局参考路径 frame 与 OccupancyGrid frame 不一致: path_frame=%s, map_frame=%s。"
                "当前代码不会做 TF 转换，请确认两者处在同一坐标系。",
                msg->header.frame_id.c_str(),
                costmap_frame_id_.c_str());
        }

        if (out_of_map_count > 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Ego 全局参考路径有 %zu/%zu 个点超出当前 OccupancyGrid 边界，首个越界点=(%.3f, %.3f)，"
                "map_x=[%.3f, %.3f), map_y=[%.3f, %.3f), map_frame=%s, path_frame=%s。"
                "若使用 rolling costmap，长全局路径末端超出当前滚动地图范围是常见原因。",
                out_of_map_count,
                global_plan_traj_.size(),
                first_out_of_map.x,
                first_out_of_map.y,
                costmap_min_x_,
                costmap_max_x_,
                costmap_min_y_,
                costmap_max_y_,
                costmap_frame_id_.c_str(),
                msg->header.frame_id.c_str());
        }
    } else {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "收到 Ego 全局参考路径，但尚未收到 OccupancyGrid，无法检查路径是否在地图范围内: topic=%s",
            input_global_path_topic_.c_str());
    }

    cur_pose_.x = msg->poses.front().pose.position.x;
    cur_pose_.y = msg->poses.front().pose.position.y;
    tf2::Quaternion q(
        msg->poses.front().pose.orientation.x,
        msg->poses.front().pose.orientation.y,
        msg->poses.front().pose.orientation.z,
        msg->poses.front().pose.orientation.w);
    cur_pose_.z = tf2::getYaw(q);

    has_valid_global_path_ = true;
    should_plan_ = true;
    active_path_from_input_topic_ = true;
    last_input_path_time_ = this->now();
    has_last_input_path_time_ = true;
    
    requestReplanLocked("new_input_path");
    
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "收到 Ego 全局参考路径: poses=%zu, topic=%s, start=(%.3f, %.3f), goal=(%.3f, %.3f), reason=new_input_path",
        global_plan_traj_.size(),
        input_global_path_topic_.c_str(),
        global_plan_traj_.front().x,
        global_plan_traj_.front().y,
        global_plan_traj_.back().x,
        global_plan_traj_.back().y);
}

// 核心逻辑：检查数据更新→触发规划→发布结果
//
// 死锁修复（方案一：把 makePlan 移出 data_mutex_）
//   原版：整个函数持 data_mutex_，直到 makePlan() 返回；如果 makePlan 卡死
//        在 A* / LBFGS 里，所有 callback (costmap_callback, rviz_global_path_callback,
//        ...) 全部排队等锁 → 节点僵死。
//   现版：拆成 4 段
//     1) 在 data_mutex_ 下做"快照"，把规划所需输入拷出来；
//     2) 释放 data_mutex_，在 planner_mutex_ 下跑 makePlan + 取结果；
//     3) 再加 data_mutex_ 把 a_star_pathes_ 写回；
//     4) 调 publish_*（每个内部各自取需要的锁）。
//   data_mutex_ 与 planner_mutex_ 任意时刻最多持有一把，绝对不嵌套。
void TrajectoryAndObstaclesPublisher::publish_and_plan()
{
    const auto ros_now = this->now();
    bool should_clear_timeout = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        should_clear_timeout = has_valid_global_path_ && inputPathTimedOut(ros_now);
    }
    if (should_clear_timeout) {
        clearActiveGoalAndTrajectory("input_path_timeout");
        publish_global_path();
        publish_planned_trajectory();
        publish_a_star_path();
        publish_local_obstacles();
        return;
    }

    bool should_check_blocked = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        should_check_blocked =
            costmap_updated_ &&
            has_valid_global_path_ &&
            should_plan_ &&
            !needs_replan_;
        costmap_updated_ = false;
    }

    if (should_check_blocked && isCurrentTrajectoryBlocked()) {
        bool can_replan = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            can_replan =
                !has_last_replan_time_ ||
                replan_cooldown_ <= 0.0 ||
                (ros_now - last_replan_time_).seconds() >= replan_cooldown_;
            if (can_replan && has_valid_global_path_ && should_plan_) {
                requestReplanLocked("trajectory_blocked");
            }
        }

        if (!can_replan) {
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "当前轨迹被阻断，但 replan_cooldown 尚未结束，跳过本次动态重规划触发。");
        }
    }

    // ========== 1. 快照阶段（持 data_mutex_） ==========
    std::vector<PathPoint> global_traj_snap;
    PathPoint cur_pose_snap;
    bool need_plan        = false;
    bool need_init_grid   = false;
    bool map_received     = false;
    std::string map_topic_copy;
    std::string replan_reason = "unknown";
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (needs_replan_ && has_valid_global_path_ && should_plan_ && !global_plan_traj_.empty()) {
            global_traj_snap = global_plan_traj_;       // 拷一份，避免在解锁后被其他 callback 改
            cur_pose_snap    = cur_pose_;
            need_plan        = true;
            replan_reason    = pending_replan_reason_.empty() ? "unknown" : pending_replan_reason_;
            // 在这里就把 needs_replan_ 置 false：即使 makePlan 长耗时，期间
            // 来的新路径会再次置 true，不会被覆盖
            needs_replan_    = false;
            pending_replan_reason_.clear();
            last_replan_time_ = ros_now;
            has_last_replan_time_ = true;
            need_init_grid   = (!flag_ && !map_from_costmap_);
            if (need_init_grid) flag_ = true;
            map_received     = map_from_costmap_;
            map_topic_copy   = map_topic_;
        }
    }

    // ========== 2. 重活阶段（持 planner_mutex_，不持 data_mutex_） ==========
    std::vector<std::vector<Eigen::Vector2d>> a_star_pathes_snap;
    std::vector<PathPoint> debug_planned_traj;
    if (need_plan) {
        // —— 2a. 路径预处理：纯本地操作，不需要任何锁 ——
        std::vector<PathPoint> global_plan_traj_temp;
        discretize_trajectory(global_traj_snap, global_plan_traj_temp, path_sample_interval_);

        float mindist = 100000000;
        int minddex = 0;
        for (int i = 0; i < static_cast<int>(global_plan_traj_temp.size()); i++) {
            double dist = distance(global_plan_traj_temp[i], cur_pose_snap);
            if (dist < mindist) {
                mindist = dist;
                minddex = i;
            }
        }

        std::vector<PathPoint> global_plan_traj_after;
        global_plan_traj_after.push_back(cur_pose_snap);

        size_t start_index = static_cast<size_t>(minddex);
        if (start_index + 1 < global_plan_traj_temp.size()) {
            const auto & closest = global_plan_traj_temp[start_index];
            const auto & next = global_plan_traj_temp[start_index + 1];
            const double tangent_x = next.x - closest.x;
            const double tangent_y = next.y - closest.y;
            const double closest_from_robot_x = closest.x - cur_pose_snap.x;
            const double closest_from_robot_y = closest.y - cur_pose_snap.y;
            if (tangent_x * closest_from_robot_x + tangent_y * closest_from_robot_y < 0.0) {
                ++start_index;
            }
        }

        PathPoint last_point = cur_pose_snap;
        double accumulated_distance = 0.0;
        const bool limit_horizon = local_planning_horizon_ > 0.0;
        for (size_t i = start_index; i < global_plan_traj_temp.size(); i++) {
            const auto & next_point = global_plan_traj_temp[i];
            const double segment_length = distance(last_point, next_point);
            if (segment_length < 1e-6) {
                continue;
            }

            if (limit_horizon &&
                accumulated_distance + segment_length > local_planning_horizon_)
            {
                const double remaining = local_planning_horizon_ - accumulated_distance;
                if (remaining > 1e-6) {
                    const double ratio = remaining / segment_length;
                    PathPoint horizon_point;
                    horizon_point.x = last_point.x + ratio * (next_point.x - last_point.x);
                    horizon_point.y = last_point.y + ratio * (next_point.y - last_point.y);
                    horizon_point.z = last_point.z + ratio * (next_point.z - last_point.z);
                    horizon_point.v = last_point.v + ratio * (next_point.v - last_point.v);
                    global_plan_traj_after.push_back(horizon_point);
                }
                break;
            }

            global_plan_traj_after.push_back(next_point);
            accumulated_distance += segment_length;
            last_point = next_point;
        }
        discretize_trajectory(global_plan_traj_after, global_plan_traj_temp, path_sample_interval_);

        // —— 2b. 真正调 ego_planner_：持 planner_mutex_ ——
        {
            std::lock_guard<std::mutex> plock(planner_mutex_);
            ego_planner_->setPathPoint(global_plan_traj_temp);
            if (need_init_grid) {
                // only call once
                ego_planner_->setGridMap(cur_pose_snap);
            }
            ego_planner_->setCurrentVehiclePos(cur_pose_snap);

            // 触发 Ego Planner 规划
            const auto make_plan_start = std::chrono::steady_clock::now();
            ego_planner_->makePlan();
            const auto make_plan_elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - make_plan_start).count();
            if (make_plan_warn_time_ms_ > 0.0 && make_plan_elapsed_ms > make_plan_warn_time_ms_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Ego makePlan 耗时偏高: elapsed_ms=%.3f, reason=%s",
                    make_plan_elapsed_ms,
                    replan_reason.c_str());
            } else {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "Ego makePlan 完成: elapsed_ms=%.3f, reason=%s",
                    make_plan_elapsed_ms,
                    replan_reason.c_str());
            }

            ego_planner_->getAStarPath(a_star_pathes_snap);
            ego_planner_->getLocalPlanTrajResults(debug_planned_traj);
        }

        if (!map_received) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "尚未收到 OccupancyGrid 地图，Ego-Planner 正在使用初始化地图；请检查 map_topic=%s",
                map_topic_copy.c_str());
        }

        if (debug_planned_traj.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "Ego-Planner 本次规划未生成 trajectory: trajectory_generated=false, "
                "global_path=%zu, "
                "a_star_paths=%zu, map_received=%s, cur=(%.3f, %.3f)",
                global_traj_snap.size(),
                a_star_pathes_snap.size(),
                map_received ? "true" : "false",
                cur_pose_snap.x,
                cur_pose_snap.y);
        } else {
            const auto & first = debug_planned_traj.front();
            const auto & last = debug_planned_traj.back();
            RCLCPP_INFO(
                this->get_logger(),
                "Ego-Planner 本次规划已生成 trajectory: trajectory_generated=true, "
                "points=%zu, first=(%.3f, %.3f, v=%.3f), "
                "last=(%.3f, %.3f, v=%.3f), a_star_paths=%zu",
                debug_planned_traj.size(),
                first.x,
                first.y,
                first.v,
                last.x,
                last.y,
                last.v,
                a_star_pathes_snap.size());
        }

        if (!debug_planned_traj.empty()) {
            const auto success_time = this->now();
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_ego_traj_ = debug_planned_traj;
            stop_trajectory_active_ = false;
            last_success_plan_time_ = success_time;
            has_last_success_plan_time_ = true;
            a_star_pathes_ = std::move(a_star_pathes_snap);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Ego makePlan 成功并替换当前轨迹: reason=%s, points=%zu",
                replan_reason.c_str(),
                current_ego_traj_.size());
        } else {
            const auto fail_time = this->now();
            bool old_traj_recent = false;
            bool has_old_traj = false;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                has_old_traj = current_ego_traj_.size() >= 2;
                old_traj_recent =
                    has_old_traj &&
                    has_last_success_plan_time_ &&
                    fallback_trajectory_max_age_ > 0.0 &&
                    (fail_time - last_success_plan_time_).seconds() <= fallback_trajectory_max_age_;
                a_star_pathes_ = std::move(a_star_pathes_snap);
            }

            const bool old_traj_safe = old_traj_recent && !isCurrentTrajectoryBlocked();
            if (old_traj_safe) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Ego makePlan 失败，但旧轨迹仍在短时有效期内且短窗口安全，继续沿用: reason=%s",
                    replan_reason.c_str());
            } else {
                const std::string fail_kind =
                    replan_reason == "trajectory_blocked" ?
                    "dynamic_replan_failed_stop" : "initial_or_manual_plan_failed_stop";
                setStopTrajectory(cur_pose_snap, fail_kind);
            }
        }

        RCLCPP_INFO_THROTTLE(
                   this->get_logger(),
                   *this->get_clock(),
                   2000,
                   "规划完成! 路径点: %zu",
                   global_traj_snap.size());
    }

    // ========== 4. 发布阶段（每个 publish_* 自己取需要的锁） ==========
    publish_global_path();
    publish_planned_trajectory();
    publish_a_star_path();
    publish_local_obstacles();
}

// 发布可视化全局路径
void TrajectoryAndObstaclesPublisher::publish_global_path()
{
    // 在锁内拷一份本地副本，再释放锁后做 ROS 发布，避免持锁期间被其他 callback 等待
    std::vector<PathPoint> traj_copy;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        traj_copy = global_plan_traj_;
    }

    nav_msgs::msg::Path visual_path;
    visual_path.header.stamp = this->now();
    visual_path.header.frame_id = frame_id_;

    for (const auto& path_point : traj_copy)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_path.header;
        pose.pose.position.x = path_point.x;
        pose.pose.position.y = path_point.y;
        pose.pose.position.z = path_point.z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_path.poses.push_back(pose);
    }

    global_path_pub_->publish(visual_path);
}

void TrajectoryAndObstaclesPublisher::publish_a_star_path()
{
    // 锁内拷一份再发，避免持锁做 ROS publish
    std::vector<std::vector<Eigen::Vector2d>> paths_copy;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        paths_copy = a_star_pathes_;
    }

    // 如果没有路径数据，直接返回
    if (paths_copy.empty())
    {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "a_star_pathes_ is empty, no A* visualization trajectories to publish.");
      return;
    }

    // 创建一个MarkerArray消息
    visualization_msgs::msg::MarkerArray marker_array_msg;

    // 遍历 a_star_pathes_ 中的每一条路径
    for (size_t i = 0; i < paths_copy.size(); ++i)
    {
      const std::vector<Eigen::Vector2d>& current_path = paths_copy[i];

      // 1. 创建一个Marker对象
      visualization_msgs::msg::Marker marker;

      // 2. 设置Marker的基本信息
      marker.header.frame_id = frame_id_; // 非常重要！指定轨迹所在的坐标系
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "a_star_paths"; // 命名空间，用于分组管理Marker
      marker.id = i;              // 每条路径必须有唯一的ID
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 类型为线串
      marker.action = visualization_msgs::msg::Marker::ADD;      // 动作是添加

      // 3. 设置线的视觉属性
      marker.scale.x = 0.1; // 线宽为0.1米

      // 设置颜色（RGBA格式），每条路径可以设置不同的颜色
      marker.color.r = 1.0 - (i * 0.3); // 红色分量
      marker.color.g = 0.2;             // 绿色分量
      marker.color.b = 0.2 + (i * 0.3); // 蓝色分量
      marker.color.a = 1.0;             // 透明度（1.0为完全不透明）

      // 4. 填充路径点
      for (const auto& eigen_point : current_path)
      {
        geometry_msgs::msg::Point ros_point;
        ros_point.x = eigen_point.x();
        ros_point.y = eigen_point.y();
        ros_point.z = 0.0; // 2D轨迹，z轴设为0

        marker.points.push_back(ros_point);
      }

      // 5. 将当前路径的Marker添加到MarkerArray中
      marker_array_msg.markers.push_back(marker);
    }

    // 6. 发布MarkerArray消息
    a_star_path_pub_->publish(marker_array_msg);
    // RCLCPP_INFO(this->get_logger(), "a_star_path_pub_ Published %zu trajectories.", a_star_pathes_.size());

}

// 发布Ego Planner规划后的局部轨迹
void TrajectoryAndObstaclesPublisher::publish_planned_trajectory()
{
    std::vector<PathPoint> planned_traj;
    bool has_global_path = false;
    bool should_plan = false;
    bool needs_replan = false;
    bool map_received = false;
    bool stop_trajectory_active = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        planned_traj = current_ego_traj_;
        has_global_path = has_valid_global_path_;
        should_plan = should_plan_;
        needs_replan = needs_replan_;
        map_received = map_from_costmap_;
        stop_trajectory_active = stop_trajectory_active_;
    }

    nav_msgs::msg::Path visual_traj;
    visual_traj.header.stamp = this->now();
    visual_traj.header.frame_id = frame_id_;

    ego_planner_msgs::msg::Trajectory ego_traj;
    ego_traj.header = visual_traj.header;
    ego_traj.time_step = static_cast<float>(reference_time_step_);

    for (size_t i = 0; i < planned_traj.size(); ++i)
    {
        // std::cout << "[publish_planned_trajectory] x = " << planned_traj[i].x << " y =" << planned_traj[i].y << std::endl;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_traj.header;
        pose.pose.position.x = planned_traj[i].x;
        pose.pose.position.y = planned_traj[i].y;
        pose.pose.position.z = planned_traj[i].z;
        const double yaw = yawFromTrajectoryPoint(planned_traj, i);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_traj.poses.push_back(pose);
    }

    if (planned_traj.size() == 1) {
        ego_planner_msgs::msg::TrajectoryPoint point;
        point.x = planned_traj.front().x;
        point.y = planned_traj.front().y;
        point.z = planned_traj.front().z;
        point.yaw = 0.0f;
        point.velocity = 0.0f;
        ego_traj.points.push_back(point);
    } else if (planned_traj.size() > 1) {
        std::vector<double> cumulative_distance(planned_traj.size(), 0.0);
        for (size_t i = 1; i < planned_traj.size(); ++i) {
            cumulative_distance[i] =
                cumulative_distance[i - 1] + distance(planned_traj[i - 1], planned_traj[i]);
        }

        const double total_distance = cumulative_distance.back();
        if (total_distance > 1e-6) {
            size_t segment_idx = 1;
            const double spatial_step = std::max(reference_speed_ * reference_time_step_, 1e-3);

            auto append_reference_point = [&](double sample_distance, bool force_stop) {
                sample_distance = std::clamp(sample_distance, 0.0, total_distance);
                while (segment_idx + 1 < cumulative_distance.size() &&
                    cumulative_distance[segment_idx] <= sample_distance)
                {
                    ++segment_idx;
                }

                const size_t prev_idx = segment_idx > 0 ? segment_idx - 1 : 0;
                const auto & start = planned_traj[prev_idx];
                const auto & end = planned_traj[segment_idx];
                const double segment_length = cumulative_distance[segment_idx] -
                    cumulative_distance[prev_idx];
                const double ratio = segment_length > 1e-6 ?
                    (sample_distance - cumulative_distance[prev_idx]) / segment_length : 0.0;

                ego_planner_msgs::msg::TrajectoryPoint point;
                point.x = start.x + ratio * (end.x - start.x);
                point.y = start.y + ratio * (end.y - start.y);
                point.z = start.z + ratio * (end.z - start.z);
                const double dx = end.x - start.x;
                const double dy = end.y - start.y;
                point.yaw = static_cast<float>(
                    std::hypot(dx, dy) > 1e-4 ? std::atan2(dy, dx) : yawFromTrajectoryPoint(planned_traj, prev_idx));

                double point_speed = force_stop ? 0.0 : reference_speed_;
                if (!force_stop && terminal_slowdown_distance_ > 1e-6) {
                    const double remaining_distance = total_distance - sample_distance;
                    point_speed *= std::clamp(remaining_distance / terminal_slowdown_distance_, 0.0, 1.0);
                }
                point.velocity = static_cast<float>(std::clamp(point_speed, 0.0, max_vel_));
                ego_traj.points.push_back(point);
            };

            for (double sample_distance = 0.0; sample_distance < total_distance; sample_distance += spatial_step) {
                append_reference_point(sample_distance, false);
            }
            append_reference_point(total_distance, true);
        } else {
            for (size_t i = 0; i < planned_traj.size(); ++i) {
                ego_planner_msgs::msg::TrajectoryPoint point;
                point.x = planned_traj[i].x;
                point.y = planned_traj[i].y;
                point.z = planned_traj[i].z;
                point.yaw = static_cast<float>(yawFromTrajectoryPoint(planned_traj, i));
                point.velocity = 0.0f;
                ego_traj.points.push_back(point);
            }
        }
    }

    local_traj_pub_->publish(visual_traj);
    ego_trajectory_pub_->publish(ego_traj);

    if (planned_traj.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "发布空 Ego 参考轨迹: topic=%s, has_global_path=%s, should_plan=%s, "
            "needs_replan=%s, map_received=%s",
            ego_trajectory_topic_.c_str(),
            has_global_path ? "true" : "false",
            should_plan ? "true" : "false",
            needs_replan ? "true" : "false",
            map_received ? "true" : "false");
    } else if (ego_traj.points.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "发布空 Ego 时间参考轨迹: topic=%s, bspline_points=%zu",
            ego_trajectory_topic_.c_str(),
            planned_traj.size());
    } else if (stop_trajectory_active) {
        const auto & first = ego_traj.points.front();
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "已发布 Ego 停车参考轨迹: topic=%s, reference_points=%zu, pos=(%.3f, %.3f)",
            ego_trajectory_topic_.c_str(),
            ego_traj.points.size(),
            first.x,
            first.y);
    } else {
        const auto & first = ego_traj.points.front();
        const auto & last = ego_traj.points.back();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "已发布 Ego 参考轨迹: topic=%s, bspline_points=%zu, reference_points=%zu, "
            "reference_speed=%.3f, reference_time_step=%.3f, "
            "first=(%.3f, %.3f, v=%.3f), last=(%.3f, %.3f, v=%.3f)",
            ego_trajectory_topic_.c_str(),
            planned_traj.size(),
            ego_traj.points.size(),
            reference_speed_,
            reference_time_step_,
            first.x,
            first.y,
            first.velocity,
            last.x,
            last.y,
            last.velocity);
    }
}

void TrajectoryAndObstaclesPublisher::publish_local_obstacles()
{
    // grid_map_ 的读取必须在 planner_mutex_ 下进行，
    // 否则与 costmap_callback 的写入构成数据竞争
    std::vector<ObstacleInfo> obstacles;
    {
        std::lock_guard<std::mutex> plock(planner_mutex_);
        ego_planner_->getObstacles(obstacles);
    }

    if (obstacles.empty()) return;

    sensor_msgs::msg::PointCloud2 visual_obs;
    visual_obs.header.stamp = this->now();
    visual_obs.header.frame_id = frame_id_;

    // 正确设置点云大小
    visual_obs.height = 1;
    visual_obs.width = obstacles.size();
    visual_obs.is_dense = true;

    // 为点云分配字段
    sensor_msgs::PointCloud2Modifier modifier(visual_obs);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(obstacles.size());   // 关键！分配空间

    // 创建迭代器
    sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

    for (const auto& obs : obstacles)
    {
        *iter_x = static_cast<float>(obs.x);
        *iter_y = static_cast<float>(obs.y);
        *iter_z = 0.0f;

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    obs_local_pub_->publish(visual_obs);
}

// 计算两点之间的欧氏距离（单位：米）
double TrajectoryAndObstaclesPublisher::distance(const PathPoint& p1, const PathPoint& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx*dx + dy*dy);
}

/**
 * 将轨迹离散为均匀间隔的点
 * @param original_trajectory 原始轨迹（由多个顶点组成的折线）
 * @param discrete_trajectory 输出的离散轨迹
 * @param interval 间隔距离（单位：米）
 */
void TrajectoryAndObstaclesPublisher::discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                                                            std::vector<PathPoint>& discrete_trajectory,
                                                            double interval) {
    discrete_trajectory.clear();
    if (original_trajectory.empty()) {
        std::cerr << "原始轨迹为空，无法离散化！" << std::endl;
        return;
    }

    if (original_trajectory.size() < 2) {
        discrete_trajectory.push_back(original_trajectory.front());
        std::cerr << "原始轨迹至少需要2个点！" << std::endl;
        return;
    }

    interval = std::max(interval, 1e-3);
    // 添加轨迹起点
    // discrete_trajectory.push_back(cur_pose_);
    discrete_trajectory.push_back(original_trajectory[0]);

    // 遍历原始轨迹的每一段线段
    for (size_t i = 0; i < original_trajectory.size() - 1; ++i) {
        const PathPoint& start = original_trajectory[i];
        const PathPoint& end = original_trajectory[i+1];
        double seg_length = distance(start, end);  // 线段总长度

        if (seg_length < 1e-6) {  // 跳过长度接近0的线段（避免除零）
            continue;
        }

        // 计算当前线段需要插入的点数（不含起点，含终点）。
        // 用 ceil 保证短于 interval 的线段也会保留终点，避免整条短路径被压成 1 个点。
        int num_points = std::max(1, static_cast<int>(std::ceil(seg_length / interval)));

        // 生成线段上的离散点
        for (int j = 1; j <= num_points; ++j) {
            double ratio = static_cast<double>(j) / static_cast<double>(num_points);

            // 线性插值计算点坐标
            PathPoint p;
            p.x = start.x + ratio * (end.x - start.x);
            p.y = start.y + ratio * (end.y - start.y);
            p.z = start.z + ratio * (end.z - start.z);
            p.v = start.v + ratio * (end.v - start.v);
            discrete_trajectory.push_back(p);
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryAndObstaclesPublisher>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
