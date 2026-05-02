#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mid360_deskew
{

struct Pose
{
  rclcpp::Time stamp;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct PoseBufferStatus
{
  std::size_t sample_count{0};
  rclcpp::Time start{0, 0, RCL_ROS_TIME};
  rclcpp::Time end{0, 0, RCL_ROS_TIME};
};

class PoseBuffer
{
public:
  PoseBuffer(double duration_seconds, double max_allowed_time_gap_seconds, bool allow_extrapolation);

  void configure(double duration_seconds, double max_allowed_time_gap_seconds, bool allow_extrapolation);

  void addPose(const Pose & pose);
  void addOdometry(const nav_msgs::msg::Odometry & odom);

  bool lookup(const rclcpp::Time & stamp, Pose & pose, std::string * reason = nullptr) const;

  std::vector<Pose> snapshot() const;
  PoseBufferStatus status() const;

  double maxAllowedTimeGap() const;
  bool allowExtrapolation() const;

  static bool lookupInPoses(
    const std::vector<Pose> & poses,
    const rclcpp::Time & stamp,
    double max_allowed_time_gap_seconds,
    bool allow_extrapolation,
    Pose & pose,
    std::string * reason = nullptr);

private:
  void pruneUnlocked();

  mutable std::mutex mutex_;
  std::deque<Pose> poses_;
  double duration_seconds_{5.0};
  double max_allowed_time_gap_seconds_{0.05};
  bool allow_extrapolation_{false};
};

Eigen::Isometry3d poseToIsometry(const Pose & pose);

}  // namespace mid360_deskew
