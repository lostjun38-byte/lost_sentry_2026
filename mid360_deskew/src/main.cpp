#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mid360_deskew/deskew_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mid360_deskew::DeskewNode>());
  rclcpp::shutdown();
  return 0;
}
