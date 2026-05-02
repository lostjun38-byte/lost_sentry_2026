#pragma once

#include <memory>
#include <cstdint>
#include <string>

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mid360_deskew/delay_statistics.hpp"
#include "mid360_deskew/imu_buffer.hpp"
#include "mid360_deskew/pointcloud_field_helper.hpp"
#include "mid360_deskew/pose_buffer.hpp"

namespace mid360_deskew
{

class DeskewNode : public rclcpp::Node
{
public:
  explicit DeskewNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class FailureKind
  {
    Generic,
    NoPose,
    NoImu
  };

  struct DeskewTiming
  {
    rclcpp::Time actual_scan_start{0, 0, RCL_ROS_TIME};
    rclcpp::Time actual_scan_end{0, 0, RCL_ROS_TIME};
    rclcpp::Time t_ref{0, 0, RCL_ROS_TIME};
    double scan_duration{0.0};
  };

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  bool lookupBaseToLidar(Eigen::Isometry3d & t_base_lidar, std::string & reason);
  void handleDeskewFailure(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & reason,
    FailureKind kind = FailureKind::Generic);

  bool deskewWithMethod(
    const std::string & method,
    const sensor_msgs::msg::PointCloud2 & cloud,
    const PointCloudFieldHelper & field_helper,
    const std::vector<double> & point_offsets,
    const DeskewTiming & timing,
    const Eigen::Isometry3d & t_base_lidar,
    sensor_msgs::msg::PointCloud2 & output,
    std::string & reason,
    FailureKind & failure_kind);

  void recordCloudDelay(const sensor_msgs::msg::PointCloud2 & cloud);
  void recordOdomDelay(const nav_msgs::msg::Odometry & odom);
  void recordImuDelay(const sensor_msgs::msg::Imu & imu);
  void maybePrintTimeCheck();

  void logFieldDetectionIfChanged(const PointCloudFieldHelper & helper);
  void logCloudTiming(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const rclcpp::Time & actual_scan_start,
    const rclcpp::Time & actual_scan_end,
    double scan_duration,
    double min_point_offset,
    double max_point_offset) const;
  void logPoseBufferStatus(const std::string & context) const;
  void logImuBufferStatus(const std::string & context) const;

  static std::string stampToString(const rclcpp::Time & stamp);
  static bool isSupportedScanStampType(const std::string & value);
  static bool isSupportedDeskewTargetTime(const std::string & value);
  static bool isSupportedDeskewMethod(const std::string & value);
  static bool isSupportedFallbackMethod(const std::string & value);
  static bool isSupportedRotationSource(const std::string & value);

  double odomTimeOffsetRuntime() const;
  double imuTimeOffsetRuntime() const;

  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string odom_topic_;
  std::string imu_topic_;

  std::string fixed_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string imu_frame_;
  std::string deskew_method_;
  std::string fallback_method_;
  std::string deskew_target_time_;
  std::string rotation_source_;
  std::string translation_source_;
  std::string point_time_field_;
  std::string point_time_unit_;
  std::string cloud_stamp_type_;
  std::string filter_mode_;

  double cloud_time_offset_{0.0};
  double odom_time_offset_config_{0.0};
  double odom_time_offset_auto_{0.0};
  double imu_time_offset_config_{0.0};
  double imu_time_offset_auto_{0.0};

  double pose_buffer_duration_{5.0};
  double max_allowed_time_gap_{0.05};
  bool allow_extrapolation_{false};
  double imu_buffer_duration_{5.0};
  double max_allowed_imu_gap_{0.02};
  bool estimate_gyro_bias_{true};
  double gyro_bias_estimation_duration_{2.0};
  double gyro_bias_static_threshold_{0.05};
  bool use_acc_for_translation_{false};

  double min_range_{0.05};
  double max_range_{5.0};
  bool remove_nan_{true};

  bool drop_if_no_point_time_{true};
  bool drop_if_no_pose_{true};
  bool drop_if_no_imu_{true};
  bool publish_raw_when_failed_{false};

  bool enable_time_check_{true};
  double time_check_window_{3.0};
  int time_check_min_samples_{20};
  bool apply_auto_time_offset_{false};
  double auto_time_offset_max_abs_{0.1};
  double delay_jitter_warning_threshold_{0.01};
  bool print_time_check_result_{true};

  bool enable_debug_log_{true};

  PoseBuffer pose_buffer_;
  ImuBuffer imu_buffer_;
  std::uint64_t interpolation_failed_count_{0};
  std::uint64_t imu_integration_failed_count_{0};
  std::string last_field_detection_summary_;
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias_sum_{Eigen::Vector3d::Zero()};
  std::uint64_t gyro_bias_sample_count_{0};
  bool gyro_bias_window_started_{false};
  bool gyro_bias_estimation_finished_{false};
  bool gyro_bias_estimated_{false};
  bool gyro_bias_warning_printed_{false};
  rclcpp::Time gyro_bias_window_start_{0, 0, RCL_ROS_TIME};

  DelayStatistics cloud_delay_stats_;
  DelayStatistics odom_delay_stats_;
  DelayStatistics imu_delay_stats_;
  rclcpp::Time last_time_check_print_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace mid360_deskew
