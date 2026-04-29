#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

class Pc2ToCustomMsgNode : public rclcpp::Node
{
public:
    Pc2ToCustomMsgNode() : Node("pc2_to_custom_node")
    {
        // 订阅原始的 PointCloud2 话题
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/red_standard_robot1/livox/lidar", 
            rclcpp::SensorDataQoS(), 
            std::bind(&Pc2ToCustomMsgNode::pointcloudCallback, this, std::placeholders::_1)
        );

        // 发布转换后的 CustomMsg 话题
        pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
            "/red_standard_robot1/livox/custom_msg", 
            10
        );

        RCLCPP_INFO(this->get_logger(), "PointCloud2 to Livox CustomMsg converter initialized.");
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        livox_ros_driver2::msg::CustomMsg custom_msg;
        
        // 1. 复制头部信息
        custom_msg.header = msg->header;
        // 将 ROS 时间戳转换为纳秒级 uint64_t
        custom_msg.timebase = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        custom_msg.lidar_id = 0;
        
        // 点云数量，由 width * height 决定（你提供的数据中是 19992 * 1）
        uint32_t point_num = msg->width * msg->height;
        custom_msg.point_num = point_num;

        // 2. 计算点级时间偏移 (假设雷达频率为 10Hz, 周期 100ms)
        // 100ms = 100,000,000 ns
        double time_interval = 100000000.0 / static_cast<double>(point_num);

        // 3. 使用迭代器安全地读取 PointCloud2 的二进制数据
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");

        // 4. 遍历提取数据并构建 CustomPoint
        for (uint32_t i = 0; i < point_num; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
        {
            livox_ros_driver2::msg::CustomPoint pt;
            pt.x = *iter_x;
            pt.y = *iter_y;
            pt.z = *iter_z;
            
            // intensity 通常是 float，需要转换为 CustomMsg 要求的 uint8_t
            pt.reflectivity = static_cast<uint8_t>(*iter_intensity); 
            
            pt.tag = 0;   // 仿真中暂不区分多重回波
            pt.line = 0;  // 统一分配为单线束逻辑
            pt.offset_time = static_cast<uint32_t>(i * time_interval); // 注入插值生成的时间戳

            custom_msg.points.push_back(pt);
        }

        // 5. 发布转换后的消息
        pub_->publish(custom_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pc2ToCustomMsgNode>());
    rclcpp::shutdown();
    return 0;
}