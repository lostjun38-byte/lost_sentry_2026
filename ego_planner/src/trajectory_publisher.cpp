/*
 * @Function: Ego Planner + RViz Interactive Input (Global Path + Obstacles)
 * @Create by: juchunyu@qq.com
 * @Date: 2025-11-01 18:58:01
 */
#include "trajectory_obstacles_publisher.h"
#include <algorithm>
#include <tf2/utils.h>
#include <thread>
#include "geometry_msgs/msg/quaternion.hpp"  // 确保包含四元数消息类型

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
      has_obstacles_(false),
      should_plan_(true),
      needs_replan_(false)
{
    this->declare_parameter("auto_plan", auto_plan_);
    this->declare_parameter("frame_id", frame_id_);
    this->declare_parameter("map_topic", map_topic_);
    this->declare_parameter("local_planning_horizon", local_planning_horizon_);
    this->get_parameter("auto_plan", auto_plan_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("map_topic", map_topic_);
    this->get_parameter("local_planning_horizon", local_planning_horizon_);
    local_planning_horizon_ = std::max(local_planning_horizon_, 0.0);
    should_plan_ = auto_plan_;

    const auto global_path_topic = topic_or_default("global_path_topic", "visual_global_path");
    const auto astar_topic = topic_or_default("astar_topic", "trajectories");
    local_trajectory_topic_ = topic_or_default("local_trajectory_topic", local_trajectory_topic_);
    ego_trajectory_topic_ = topic_or_default("ego_trajectory_topic", ego_trajectory_topic_);
    input_global_path_topic_ = topic_or_default("input_global_path_topic", input_global_path_topic_);
    const auto obstacles_topic = topic_or_default("obstacles_topic", "visual_obstacles");
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
    obs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(obstacles_topic, 10);
    obs_local_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(local_obstacles_topic, 10);

    // 2. 创建订阅者（接收RViz下发的数据）
    rviz_global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        input_global_path_topic_,
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::rviz_global_path_callback, this, std::placeholders::_1)
    );

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic,
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::goal_pose_callback, this, std::placeholders::_1)
    );

    // // 障碍物输入方式1：发布PointCloud2消息
    // rviz_obstacles_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "/rviz_input_obstacles",
    //     10,
    //     std::bind(&TrajectoryAndObstaclesPublisher::rviz_obstacles_callback, this, std::placeholders::_1)
    // );

    // 路径添加：使用Publish Point工具逐个添加
    rviz_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        clicked_point_topic,
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::rviz_point_callback, this, std::placeholders::_1)
    );

    // 障碍物输入方式3：使用2D Pose Estimate工具添加
    pose_estimate_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic,
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::pose_estimate_callback, this, std::placeholders::_1)
    );

    // 3. 触发规划的话题
    trigger_plan_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        trigger_plan_topic,
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::trigger_plan_callback, this, std::placeholders::_1)
    );

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic,
        rclcpp::QoS(10),
        std::bind(&TrajectoryAndObstaclesPublisher::costmap_callback, this, std::placeholders::_1)
    );

    // 4. 初始化Ego Planner基础配置
    init_ego_planner_base();

    // 5. 定时器：5Hz触发规划与发布
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&TrajectoryAndObstaclesPublisher::publish_and_plan, this)
    );

    RCLCPP_INFO(this->get_logger(), "Interactive Ego Planner Node Ready!");
    RCLCPP_INFO(this->get_logger(), "=== 使用方法 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加全局路径：使用2D Nav Goal工具设置终点");
    RCLCPP_INFO(this->get_logger(), "2. 添加障碍物（三种方法任选）：");
    RCLCPP_INFO(this->get_logger(), "   - 方法A：使用Publish Point工具点击地图");
    RCLCPP_INFO(this->get_logger(), "   - 方法B：使用2D Pose Estimate工具点击Grid任意位置（推荐）");
    RCLCPP_INFO(this->get_logger(), "   - 方法C：发布PointCloud2到 /rviz_input_obstacles");
    RCLCPP_INFO(this->get_logger(), "3. 触发规划：ros2 topic pub /trigger_plan std_msgs/Bool \"{data: true}\"");
    RCLCPP_INFO(this->get_logger(), "4. 停止规划：ros2 topic pub /trigger_plan std_msgs/Bool \"{data: false}\"");
    RCLCPP_INFO(this->get_logger(), "5. 支持多次规划：可以反复发送true/false控制规划");
    RCLCPP_INFO(this->get_logger(), "6. 地图输入话题：%s", map_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "7. Nav2全局路径输入话题：%s", input_global_path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "=== RViz2设置步骤 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加Grid显示");
    RCLCPP_INFO(this->get_logger(), "2. 添加2D Nav Goal工具（话题：/goal_pose）");
    RCLCPP_INFO(this->get_logger(), "3. 添加2D Pose Estimate工具（使用默认话题：/initialpose）");
    RCLCPP_INFO(this->get_logger(), "4. 添加Publish Point工具（使用默认话题：/clicked_point）");
    RCLCPP_INFO(this->get_logger(), "5. 添加PointCloud2显示（话题：/visual_obstacles）");
    RCLCPP_INFO(this->get_logger(), "6. 添加Path显示（话题：/visual_global_path和/visual_local_trajectory）");
    RCLCPP_INFO(this->get_logger(), "8. Ego参考轨迹输出话题：%s", ego_trajectory_topic_.c_str());
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

// 统一添加障碍物函数
void TrajectoryAndObstaclesPublisher::add_obstacle_at_position(double x, double y)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    ObstacleInfo obs;
    obs.x = x;
    obs.y = y;
    obs.z = 0.0;
    
    obstacles_.push_back(obs);
    has_obstacles_ = true;
    
    // 如果有障碍物更新且正在规划中，则标记需要重新规划
    if (should_plan_) {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "障碍物更新，已标记需要重新规划");
    }
    
    RCLCPP_INFO(this->get_logger(), "添加障碍物: (%.2f, %.2f), 总障碍物数量: %zu", 
                x, y, obstacles_.size());
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
        needs_replan_ = true;  // 标记需要重新规划
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
    std::lock_guard<std::mutex> lock(data_mutex_);
    should_plan_ = msg->data;
    if (should_plan_) {
        if (has_valid_global_path_) {
            needs_replan_ = true;
            RCLCPP_INFO(this->get_logger(), "规划已触发! 路径点: %zu, 障碍物: %zu",
                       global_plan_traj_.size(), obstacles_.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "无法触发规划: 没有有效的全局路径!");
        }
    } else {
        needs_replan_ = false;
        planned_traj.clear();
        RCLCPP_INFO(this->get_logger(), "规划已停止.");
    }
    flag_ = msg->data;
}

void TrajectoryAndObstaclesPublisher::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    ego_planner_->setNav2InflatedOccupancyGridMap(*msg);
    map_from_costmap_ = true;
    has_costmap_bounds_ = msg->info.width > 0 && msg->info.height > 0 && msg->info.resolution > 0.0;
    costmap_frame_id_ = msg->header.frame_id;
    costmap_resolution_ = msg->info.resolution;
    costmap_min_x_ = msg->info.origin.position.x;
    costmap_min_y_ = msg->info.origin.position.y;
    costmap_max_x_ = costmap_min_x_ + static_cast<double>(msg->info.width) * msg->info.resolution;
    costmap_max_y_ = costmap_min_y_ + static_cast<double>(msg->info.height) * msg->info.resolution;
    if (should_plan_ && has_valid_global_path_) {
        needs_replan_ = true;
    }

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
    if (should_plan_) {
        needs_replan_ = true;
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

// // 处理Publish Point点击 - 添加障碍物
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
    
    // 如果有路径更新且正在规划中，则标记需要重新规划
    if (should_plan_) 
    {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "路径更新，已标记需要重新规划");
    }
}

// 原有的全局路径回调
void TrajectoryAndObstaclesPublisher::rviz_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (msg->poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "从RViz接收到空的全局路径!");
        has_valid_global_path_ = false;
        return;
    }

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
    
    // 如果有路径更新且正在规划中，则标记需要重新规划
    if (should_plan_) {
        needs_replan_ = true;
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "路径更新，已标记需要重新规划");
    }
    
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "收到 Ego 全局参考路径: poses=%zu, topic=%s, start=(%.3f, %.3f), goal=(%.3f, %.3f)",
        global_plan_traj_.size(),
        input_global_path_topic_.c_str(),
        global_plan_traj_.front().x,
        global_plan_traj_.front().y,
        global_plan_traj_.back().x,
        global_plan_traj_.back().y);
}

// 核心逻辑：检查数据更新→触发规划→发布结果
void TrajectoryAndObstaclesPublisher::publish_and_plan()
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 如果有全局路径且应该规划，并且需要重新规划，则进行规划
    if (needs_replan_ && has_valid_global_path_ && !global_plan_traj_.empty())
    {
        a_star_pathes_.clear();
        // 设置全局路径
        if (!global_plan_traj_.empty())
        {
            std::vector<PathPoint> global_plan_traj_temp;


            discretize_trajectory(global_plan_traj_, global_plan_traj_temp, 0.1);
             
            float mindist = 100000000;
            int minddex = 0;
            for(int i = 0; i < global_plan_traj_temp.size();i++)
            {
                double dist =  distance(global_plan_traj_temp[i], cur_pose_); 
                if(dist < mindist)
                {
                    mindist = dist;
                    minddex = i;
                } 
  
            }

            std::vector<PathPoint> global_plan_traj_after;
            global_plan_traj_after.push_back(cur_pose_);

            size_t start_index = static_cast<size_t>(minddex);
            if (start_index + 1 < global_plan_traj_temp.size()) {
                const auto & closest = global_plan_traj_temp[start_index];
                const auto & next = global_plan_traj_temp[start_index + 1];
                const double tangent_x = next.x - closest.x;
                const double tangent_y = next.y - closest.y;
                const double closest_from_robot_x = closest.x - cur_pose_.x;
                const double closest_from_robot_y = closest.y - cur_pose_.y;
                if (tangent_x * closest_from_robot_x + tangent_y * closest_from_robot_y < 0.0) {
                    ++start_index;
                }
            }

            PathPoint last_point = cur_pose_;
            double accumulated_distance = 0.0;
            const bool limit_horizon = local_planning_horizon_ > 0.0;
            for(size_t i = start_index; i < global_plan_traj_temp.size(); i++)
            {
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
            discretize_trajectory(global_plan_traj_after, global_plan_traj_temp, 0.1);

            ego_planner_->setPathPoint(global_plan_traj_temp);
        }
        // std::cout << "cur_pose_x =" << cur_pose_.x << "cur_pose_y =" << cur_pose_.y << std::endl;
        if(!flag_ && !map_from_costmap_)
        {
            // only call once
            ego_planner_->setGridMap(cur_pose_);
            flag_ = true;
        }
        if (!map_from_costmap_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "尚未收到 OccupancyGrid 地图，Ego-Planner 正在使用初始化地图；请检查 map_topic=%s",
                map_topic_.c_str());
        }
        ego_planner_->setCurrentVehiclePos(cur_pose_);
        // 设置障碍物
        ego_planner_->setObstacles(obstacles_);

        // 触发Ego Planner规划
        ego_planner_->makePlan();

        ego_planner_->getAStarPath(a_star_pathes_);

        std::vector<PathPoint> debug_planned_traj;
        ego_planner_->getLocalPlanTrajResults(debug_planned_traj);
        if (debug_planned_traj.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "Ego-Planner 本次规划未生成 trajectory: trajectory_generated=false, "
                "global_path=%zu, obstacles=%zu, "
                "a_star_paths=%zu, map_received=%s, cur=(%.3f, %.3f)",
                global_plan_traj_.size(),
                obstacles_.size(),
                a_star_pathes_.size(),
                map_from_costmap_ ? "true" : "false",
                cur_pose_.x,
                cur_pose_.y);
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
                a_star_pathes_.size());
        }

        if (has_obstacles_) {
            RCLCPP_INFO_THROTTLE(
                       this->get_logger(),
                       *this->get_clock(),
                       2000,
                       "规划完成! 包含障碍物避让. 路径点: %zu, 障碍物: %zu", 
                       global_plan_traj_.size(), obstacles_.size());
        } else {
            RCLCPP_INFO_THROTTLE(
                       this->get_logger(),
                       *this->get_clock(),
                       2000,
                       "规划完成! 无障碍物. 路径点: %zu", 
                       global_plan_traj_.size());
        }
        
        // 规划完成后，重置重新规划标志，但保持 should_plan_ 为 true
        // 这样下次有数据更新时可以自动重新规划
        // needs_replan_ = false;
        needs_replan_ = false;

    }

    // 发布所有可视化数据（无论是否更新，保持实时显示）
    publish_global_path();
    publish_planned_trajectory();
    publish_obstacles();
    publish_a_star_path();
    publish_local_obstacles();
}

// 发布可视化全局路径
void TrajectoryAndObstaclesPublisher::publish_global_path()
{
    // if (global_plan_traj_.empty()) return;

    nav_msgs::msg::Path visual_path;
    visual_path.header.stamp = this->now();
    visual_path.header.frame_id = frame_id_;

    for (const auto& path_point : global_plan_traj_)
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
    // 如果没有路径数据，直接返回
    if (a_star_pathes_.empty())
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
    for (size_t i = 0; i < a_star_pathes_.size(); ++i)
    {
      const std::vector<Eigen::Vector2d>& current_path = a_star_pathes_[i];

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
    planned_traj.clear();
    ego_planner_->getLocalPlanTrajResults(planned_traj);

    nav_msgs::msg::Path visual_traj;
    visual_traj.header.stamp = this->now();
    visual_traj.header.frame_id = frame_id_;

    ego_planner_msgs::msg::Trajectory ego_traj;
    ego_traj.header = visual_traj.header;
    ego_traj.time_step = 0.1f;

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

        ego_planner_msgs::msg::TrajectoryPoint point;
        point.x = planned_traj[i].x;
        point.y = planned_traj[i].y;
        point.z = planned_traj[i].z;
        point.yaw = static_cast<float>(yaw);
        point.velocity = std::max(planned_traj[i].v, 0.0f);
        ego_traj.points.push_back(point);
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
            has_valid_global_path_ ? "true" : "false",
            should_plan_ ? "true" : "false",
            needs_replan_ ? "true" : "false",
            map_from_costmap_ ? "true" : "false");
    } else {
        const auto & first = planned_traj.front();
        const auto & last = planned_traj.back();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "已发布 Ego 参考轨迹: topic=%s, points=%zu, first=(%.3f, %.3f, v=%.3f), "
            "last=(%.3f, %.3f, v=%.3f)",
            ego_trajectory_topic_.c_str(),
            planned_traj.size(),
            first.x,
            first.y,
            first.v,
            last.x,
            last.y,
            last.v);
    }
}

// 发布可视化障碍物
void TrajectoryAndObstaclesPublisher::publish_obstacles()
{
    // if (obstacles_.empty()) return;

    sensor_msgs::msg::PointCloud2 visual_obs;
    visual_obs.header.stamp = this->now();
    visual_obs.header.frame_id = frame_id_;
    visual_obs.width = obstacles_.size();
    visual_obs.height = 1;
    visual_obs.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(visual_obs);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

    for (const auto& obs : obstacles_)
    {
        *iter_x = static_cast<float>(obs.x);
        *iter_y = static_cast<float>(obs.y);
        *iter_z = 0.0f;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    obs_pub_->publish(visual_obs);
}

void TrajectoryAndObstaclesPublisher::publish_local_obstacles()
{
    std::vector<ObstacleInfo> obstacles;
    ego_planner_->getObstacles(obstacles);

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
 * 将轨迹离散为均匀间隔的点（间隔10cm）
 * @param original_trajectory 原始轨迹（由多个顶点组成的折线）
 * @param discrete_trajectory 输出的离散轨迹
 * @param interval 间隔距离（单位：米，默认0.1米即10cm）
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
