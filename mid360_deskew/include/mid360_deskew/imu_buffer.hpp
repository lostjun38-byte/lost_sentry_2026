#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

namespace mid360_deskew
{

struct ImuSample
{
  rclcpp::Time stamp;
  Eigen::Vector3d angular_velocity_lidar{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration_lidar{Eigen::Vector3d::Zero()};
};

struct ImuBufferStatus
{
  std::size_t sample_count{0};
  rclcpp::Time start{0, 0, RCL_ROS_TIME};
  rclcpp::Time end{0, 0, RCL_ROS_TIME};
};

class ImuBuffer
{
public:
  explicit ImuBuffer(double duration_seconds = 5.0, double max_allowed_time_gap_seconds = 0.02);

  void configure(double duration_seconds, double max_allowed_time_gap_seconds);
  void addSample(const ImuSample & sample);
  void clear();

  std::vector<ImuSample> snapshot() const;
  ImuBufferStatus status() const;

  double maxAllowedTimeGap() const;

  static bool integrateRotation(
    const std::vector<ImuSample> & samples,
    const rclcpp::Time & from,
    const rclcpp::Time & to,
    double max_allowed_time_gap_seconds,
    Eigen::Quaterniond & q_from_to,
    std::string * reason = nullptr);

private:
  void pruneUnlocked();

  static bool interpolateAngularVelocity(
    const std::vector<ImuSample> & samples,
    const rclcpp::Time & stamp,
    double max_allowed_time_gap_seconds,
    Eigen::Vector3d & angular_velocity,
    std::string * reason);

  static Eigen::Quaterniond deltaQuaternion(const Eigen::Vector3d & rotation_vector);

  mutable std::mutex mutex_;
  std::vector<ImuSample> samples_;
  double duration_seconds_{5.0};
  double max_allowed_time_gap_seconds_{0.02};
};

}  // namespace mid360_deskew
