#include "rm_decision_cpp/behaviors/is_in_position.hpp"
#include <cmath>

namespace rm_decision
{

IsInPosition::IsInPosition(const std::string& name, const NodeConfig& config,
                               std::shared_ptr<rclcpp::Node> node,
                               std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                               std::shared_ptr<tf2_ros::TransformListener> tf_listener)
    : DecoratorNode(name, config), node_(node), tf_buffer_(tf_buffer), tf_listener_(tf_listener)
{
    node_->get_parameter_or<std::string>("global_frame", global_frame_, "map");
    node_->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
}

NodeStatus IsInPosition::tick()
{
    auto p1 = getInput<geometry_msgs::msg::PoseStamped>("p1");
    auto p2 = getInput<geometry_msgs::msg::PoseStamped>("p2");
    auto p3 = getInput<geometry_msgs::msg::PoseStamped>("p3");
    auto p4 = getInput<geometry_msgs::msg::PoseStamped>("p4");

    if (!p1 || !p2 || !p3 || !p4)
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing parallelogram points");
        return NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    double transform_tolerance_ = 0.1;
    if (!nav2_util::getCurrentPose(
            robot_pose, *tf_buffer_, global_frame_, base_frame_,
            transform_tolerance_))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get robot pose");
        return NodeStatus::FAILURE;
    }

    if (isPointInParallelogram(robot_pose, p1.value(), p2.value(), p3.value(), p4.value()))
    {
        // RCLCPP_INFO(node_->get_logger(), "Robot is inside parallelogram");

        auto child_status = child_node_->executeTick();
        
        if (child_status == NodeStatus::RUNNING)
        {
            return NodeStatus::RUNNING;
        }

        return child_status;
    }
    else
    {
        // RCLCPP_INFO(node_->get_logger(), "Robot is outside parallelogram");
        return NodeStatus::FAILURE;
    }
}

bool IsInPosition::isPointInParallelogram(const geometry_msgs::msg::PoseStamped& point,
                                           const geometry_msgs::msg::PoseStamped& p1,
                                           const geometry_msgs::msg::PoseStamped& p2,
                                           const geometry_msgs::msg::PoseStamped& p3,
                                           const geometry_msgs::msg::PoseStamped& p4)
{
    auto crossProduct = [](const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2, 
                          const geometry_msgs::msg::PoseStamped& p) -> double {
        return (p2.pose.position.x - p1.pose.position.x) * (p.pose.position.y - p1.pose.position.y) - 
               (p2.pose.position.y - p1.pose.position.y) * (p.pose.position.x - p1.pose.position.x);
    };

    double c1 = crossProduct(p1, p2, point);
    double c2 = crossProduct(p2, p3, point);
    double c3 = crossProduct(p3, p4, point);
    double c4 = crossProduct(p4, p1, point);

    return (c1 >= 0 && c2 >= 0 && c3 >= 0 && c4 >= 0) || 
           (c1 <= 0 && c2 <= 0 && c3 <= 0 && c4 <= 0);
}

PortsList IsInPosition::providedPorts()
{
    return {
        InputPort<geometry_msgs::msg::PoseStamped>("p1", "First point of parallelogram"),
        InputPort<geometry_msgs::msg::PoseStamped>("p2", "Second point of parallelogram"),
        InputPort<geometry_msgs::msg::PoseStamped>("p3", "Third point of parallelogram"),
        InputPort<geometry_msgs::msg::PoseStamped>("p4", "Fourth point of parallelogram")
    };
}

} // end namespace rm_decision