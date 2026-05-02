#include "mid360_deskew/pose_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>

namespace mid360_deskew
{

PoseBuffer::PoseBuffer(
  double duration_seconds,
  double max_allowed_time_gap_seconds,
  bool allow_extrapolation)
: duration_seconds_(duration_seconds),
  max_allowed_time_gap_seconds_(max_allowed_time_gap_seconds),
  allow_extrapolation_(allow_extrapolation)
{
}

void PoseBuffer::configure(
  double duration_seconds,
  double max_allowed_time_gap_seconds,
  bool allow_extrapolation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  duration_seconds_ = duration_seconds;
  max_allowed_time_gap_seconds_ = max_allowed_time_gap_seconds;
  allow_extrapolation_ = allow_extrapolation;
  pruneUnlocked();
}

void PoseBuffer::addPose(const Pose & pose)
{
  Pose normalized_pose = pose;
  if (normalized_pose.orientation.norm() < std::numeric_limits<double>::epsilon()) {
    normalized_pose.orientation = Eigen::Quaterniond::Identity();
  } else {
    normalized_pose.orientation.normalize();
  }

  std::lock_guard<std::mutex> lock(mutex_);

  const auto new_stamp_ns = normalized_pose.stamp.nanoseconds();
  auto insert_it = std::lower_bound(
    poses_.begin(),
    poses_.end(),
    normalized_pose.stamp,
    [](const Pose & lhs, const rclcpp::Time & stamp) {
      return lhs.stamp < stamp;
    });

  if (insert_it != poses_.end() && insert_it->stamp.nanoseconds() == new_stamp_ns) {
    *insert_it = normalized_pose;
  } else {
    poses_.insert(insert_it, normalized_pose);
  }

  pruneUnlocked();
}

void PoseBuffer::addOdometry(const nav_msgs::msg::Odometry & odom)
{
  Pose pose;
  pose.stamp = rclcpp::Time(odom.header.stamp);
  pose.position = Eigen::Vector3d(
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    odom.pose.pose.position.z);
  pose.orientation = Eigen::Quaterniond(
    odom.pose.pose.orientation.w,
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z);
  addPose(pose);
}

bool PoseBuffer::lookup(const rclcpp::Time & stamp, Pose & pose, std::string * reason) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Pose> poses(poses_.begin(), poses_.end());
  return lookupInPoses(
    poses,
    stamp,
    max_allowed_time_gap_seconds_,
    allow_extrapolation_,
    pose,
    reason);
}

std::vector<Pose> PoseBuffer::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return std::vector<Pose>(poses_.begin(), poses_.end());
}

PoseBufferStatus PoseBuffer::status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  PoseBufferStatus status;
  status.sample_count = poses_.size();
  if (!poses_.empty()) {
    status.start = poses_.front().stamp;
    status.end = poses_.back().stamp;
  }
  return status;
}

double PoseBuffer::maxAllowedTimeGap() const
{
  return max_allowed_time_gap_seconds_;
}

bool PoseBuffer::allowExtrapolation() const
{
  return allow_extrapolation_;
}

bool PoseBuffer::lookupInPoses(
  const std::vector<Pose> & poses,
  const rclcpp::Time & stamp,
  double max_allowed_time_gap_seconds,
  bool allow_extrapolation,
  Pose & pose,
  std::string * reason)
{
  if (poses.empty()) {
    if (reason) {
      *reason = "odom pose buffer is empty";
    }
    return false;
  }

  if (poses.size() == 1) {
    const double dt = std::abs((stamp - poses.front().stamp).seconds());
    if (allow_extrapolation && dt <= max_allowed_time_gap_seconds) {
      pose = poses.front();
      pose.stamp = stamp;
      return true;
    }
    if (reason) {
      *reason = "odom pose buffer has only one sample";
    }
    return false;
  }

  if (stamp < poses.front().stamp) {
    const double dt = (poses.front().stamp - stamp).seconds();
    if (allow_extrapolation && dt <= max_allowed_time_gap_seconds) {
      pose = poses.front();
      pose.stamp = stamp;
      return true;
    }
    if (reason) {
      std::ostringstream oss;
      oss << "query time is earlier than odom buffer start by " << dt << " s";
      *reason = oss.str();
    }
    return false;
  }

  if (stamp > poses.back().stamp) {
    const double dt = (stamp - poses.back().stamp).seconds();
    if (allow_extrapolation && dt <= max_allowed_time_gap_seconds) {
      pose = poses.back();
      pose.stamp = stamp;
      return true;
    }
    if (reason) {
      std::ostringstream oss;
      oss << "query time is later than odom buffer end by " << dt << " s";
      *reason = oss.str();
    }
    return false;
  }

  auto upper = std::lower_bound(
    poses.begin(),
    poses.end(),
    stamp,
    [](const Pose & lhs, const rclcpp::Time & query_stamp) {
      return lhs.stamp < query_stamp;
    });

  if (upper != poses.end() && upper->stamp.nanoseconds() == stamp.nanoseconds()) {
    pose = *upper;
    return true;
  }

  if (upper == poses.begin() || upper == poses.end()) {
    if (reason) {
      *reason = "cannot bracket query time in odom buffer";
    }
    return false;
  }

  const Pose & after = *upper;
  const Pose & before = *(upper - 1);
  const double gap = (after.stamp - before.stamp).seconds();
  if (gap <= 0.0) {
    if (reason) {
      *reason = "odom buffer contains non-increasing timestamps";
    }
    return false;
  }

  if (gap > max_allowed_time_gap_seconds) {
    if (reason) {
      std::ostringstream oss;
      oss << "odom interpolation gap " << gap
          << " s exceeds max_allowed_time_gap " << max_allowed_time_gap_seconds << " s";
      *reason = oss.str();
    }
    return false;
  }

  const double alpha = (stamp - before.stamp).seconds() / gap;
  // 外部 odom 只作为运动来源：平移线性插值，旋转用四元数 slerp，不做外推估计新轨迹。
  pose.stamp = stamp;
  pose.position = before.position + alpha * (after.position - before.position);
  pose.orientation = before.orientation.slerp(alpha, after.orientation);
  pose.orientation.normalize();
  return true;
}

void PoseBuffer::pruneUnlocked()
{
  if (poses_.empty()) {
    return;
  }

  const rclcpp::Time newest = poses_.back().stamp;
  while (!poses_.empty() && (newest - poses_.front().stamp).seconds() > duration_seconds_) {
    poses_.pop_front();
  }
}

Eigen::Isometry3d poseToIsometry(const Pose & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = pose.orientation.toRotationMatrix();
  transform.translation() = pose.position;
  return transform;
}

}  // namespace mid360_deskew
