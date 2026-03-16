#ifndef RM_DECISION_IS_IN_POSITION_HPP_
#define RM_DECISION_IS_IN_POSITION_HPP_

#include "behaviortree_cpp/decorator_node.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav2_util/robot_utils.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace BT;

namespace rm_decision
{
class IsInPosition : public DecoratorNode
{
public:
  IsInPosition(const std::string& name, const NodeConfig& config, 
                 std::shared_ptr<rclcpp::Node> node,
                 std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                 std::shared_ptr<tf2_ros::TransformListener> tf_listener);

  static PortsList providedPorts();

protected:
  NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string global_frame_;
  std::string base_frame_;

  bool isPointInParallelogram(const geometry_msgs::msg::PoseStamped& point,
                             const geometry_msgs::msg::PoseStamped& p1,
                             const geometry_msgs::msg::PoseStamped& p2,
                             const geometry_msgs::msg::PoseStamped& p3,
                             const geometry_msgs::msg::PoseStamped& p4);
};

} // end namespace rm_decision

#endif