#include "rm_decision_cpp/behaviors/chassis_type.hpp"
using namespace BT;
namespace rm_decision
{

ChassisTypePublisher::ChassisTypePublisher(const std::string& name,
                                          const NodeConfig& config,
                                          std::shared_ptr<rclcpp::Node> node)
  : SyncActionNode(name, config), node_(node)
{
  chassis_type_pub_ = node_->create_publisher<std_msgs::msg::Int8>(
    "/chassis_type", 
    rclcpp::QoS(1).transient_local());
}

NodeStatus ChassisTypePublisher::tick()
{
  auto cmd = getInput<int>("chassis_cmd");
  if (!cmd) {
    RCLCPP_WARN(node_->get_logger(), "No input chassis_type");
    return NodeStatus::FAILURE;
  }

  int value = cmd.value();
  RCLCPP_INFO(node_->get_logger(), "[ChassisTypePublisher] Publishing chassis_cmd: %d", value);

  std_msgs::msg::Int8 msg;
  msg.data = value;
  chassis_type_pub_->publish(msg);

  return NodeStatus::SUCCESS;
}

PortsList ChassisTypePublisher::providedPorts()
{
  return { 
    InputPort<int8_t>("chassis_cmd", "Chassis type value (0-127)")
  };
}

} // namespace rm_decision