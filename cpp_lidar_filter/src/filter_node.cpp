#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <atomic> // 引入原子操作，保证多线程读写参数时的线程安全

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
// 引入 Livox 自定义消息类型
#include "livox_ros_driver2/msg/custom_msg.hpp" 

class LidarFilterNode : public rclcpp::Node
{
public:
  LidarFilterNode() : Node("lidar_filter_node")
  {
    // 1. 声明并获取初始参数，直接存入成员原子变量中
    this->declare_parameter("input_topic", "/livox/lidar");
    this->declare_parameter("output_topic", "/livox/lidar_filtered");
    
    min_x_ = this->declare_parameter("min_x", -0.4);
    max_x_ = this->declare_parameter("max_x", 0.4);
    min_y_ = this->declare_parameter("min_y", -0.3);
    max_y_ = this->declare_parameter("max_y", 0.3);
    min_z_ = this->declare_parameter("min_z", -0.1);
    max_z_ = this->declare_parameter("max_z", 0.6);
    negative_ = this->declare_parameter("negative", true);

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Listening on CustomMsg: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing CustomMsg to: %s", output_topic.c_str());

    // 2. 注册动态参数回调句柄，实现免重启在线调参
    param_subscriber_ = this->add_on_set_parameters_callback(
      std::bind(&LidarFilterNode::parameters_callback, this, std::placeholders::_1));

    // 3. 初始化订阅者、发布者和定时器
    sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic, rclcpp::SensorDataQoS(), // 使用SensorDataQoS，专为高带宽传感器数据优化
      std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(output_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("crop_box_marker", 10);

    timer_ = this->create_wall_timer(
     std::chrono::milliseconds(500), std::bind(&LidarFilterNode::publish_marker, this)); // 加快可视化刷新率至2Hz
  }

private:
  // --- 成员原子变量缓存区 ---
  // 使用原子变量保证在参数更新回调(写)和点云回调(读)并发时的内存安全，且无需加锁
  std::atomic<double> min_x_{0.0}, max_x_{0.0}, min_y_{0.0}, max_y_{0.0}, min_z_{0.0}, max_z_{0.0};
  std::atomic<bool> negative_{true};

  // 动态参数回调函数：当通过命令行或 rqt 修改参数时触发
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters) {
      if (param.get_name() == "min_x") min_x_ = param.as_double();
      else if (param.get_name() == "max_x") max_x_ = param.as_double();
      else if (param.get_name() == "min_y") min_y_ = param.as_double();
      else if (param.get_name() == "max_y") max_y_ = param.as_double();
      else if (param.get_name() == "min_z") min_z_ = param.as_double();
      else if (param.get_name() == "max_z") max_z_ = param.as_double();
      else if (param.get_name() == "negative") negative_ = param.as_bool();
    }
    return result;
  }

  void publish_marker() {
    // 从原子变量中读取，极速且安全
    double current_min_x = min_x_.load();
    double current_max_x = max_x_.load();
    double current_min_y = min_y_.load();
    double current_max_y = max_y_.load();
    double current_min_z = min_z_.load();
    double current_max_z = max_z_.load();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "front_mid360"; // 根据实际 TF 树修改，例如 livox_frame
    marker.header.stamp = this->now();
    marker.ns = "vehicle_body_cropbox";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = (current_max_x + current_min_x) / 2.0;
    marker.pose.position.y = (current_max_y + current_min_y) / 2.0;
    marker.pose.position.z = (current_max_z + current_min_z) / 2.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = current_max_x - current_min_x;
    marker.scale.y = current_max_y - current_min_y;
    marker.scale.z = current_max_z - current_min_z;

    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 0.7; // 70% 透明度的橙色框

    marker_pub_->publish(marker);
  }

  void cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    livox_ros_driver2::msg::CustomMsg output_msg;
    
    // 复制原消息头与元数据
    output_msg.header = msg->header;
    output_msg.timebase = msg->timebase;
    output_msg.lidar_id = msg->lidar_id;
    output_msg.rsvd = msg->rsvd;

    // 从原子变量中一次性读取当前帧的判断标准
    const double b_min_x = min_x_.load();
    const double b_max_x = max_x_.load();
    const double b_min_y = min_y_.load();
    const double b_max_y = max_y_.load();
    const double b_min_z = min_z_.load();
    const double b_max_z = max_z_.load();
    const bool   b_neg   = negative_.load();

    // 核心优化点：预分配内存，杜绝动态扩容造成的性能抖动
    output_msg.points.reserve(msg->point_num);

    // 内存连续遍历，完美利用 CPU 缓存
    for (uint32_t i = 0; i < msg->point_num; ++i) {
      const auto& pt = msg->points[i];
      
      bool in_box = (pt.x >= b_min_x && pt.x <= b_max_x &&
                     pt.y >= b_min_y && pt.y <= b_max_y &&
                     pt.z >= b_min_z && pt.z <= b_max_z);

      if (b_neg != in_box) {
        output_msg.points.push_back(pt);
      }
    }

    output_msg.point_num = output_msg.points.size();
    pub_->publish(output_msg);
  }

  // 句柄与指针声明
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 使用 MultiThreadedExecutor 可选增强并发能力，但在当前单回调结构下单线程 spin 已足够高效
  rclcpp::spin(std::make_shared<LidarFilterNode>());
  rclcpp::shutdown();
  return 0;
}