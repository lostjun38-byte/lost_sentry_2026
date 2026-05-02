#include "mid360_deskew/imu_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>

namespace mid360_deskew
{

ImuBuffer::ImuBuffer(double duration_seconds, double max_allowed_time_gap_seconds)
: duration_seconds_(duration_seconds),
  max_allowed_time_gap_seconds_(max_allowed_time_gap_seconds)
{
}

void ImuBuffer::configure(double duration_seconds, double max_allowed_time_gap_seconds)
{
  std::lock_guard<std::mutex> lock(mutex_);
  duration_seconds_ = duration_seconds;
  max_allowed_time_gap_seconds_ = max_allowed_time_gap_seconds;
  pruneUnlocked();
}

void ImuBuffer::addSample(const ImuSample & sample)
{
  if (!std::isfinite(sample.angular_velocity_lidar.x()) ||
      !std::isfinite(sample.angular_velocity_lidar.y()) ||
      !std::isfinite(sample.angular_velocity_lidar.z()) ||
      !std::isfinite(sample.linear_acceleration_lidar.x()) ||
      !std::isfinite(sample.linear_acceleration_lidar.y()) ||
      !std::isfinite(sample.linear_acceleration_lidar.z()))
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  auto insert_it = std::lower_bound(
    samples_.begin(),
    samples_.end(),
    sample.stamp,
    [](const ImuSample & lhs, const rclcpp::Time & stamp) {
      return lhs.stamp < stamp;
    });

  if (insert_it != samples_.end() && insert_it->stamp.nanoseconds() == sample.stamp.nanoseconds()) {
    *insert_it = sample;
  } else {
    samples_.insert(insert_it, sample);
  }

  pruneUnlocked();
}

void ImuBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
}

std::vector<ImuSample> ImuBuffer::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_;
}

ImuBufferStatus ImuBuffer::status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  ImuBufferStatus status;
  status.sample_count = samples_.size();
  if (!samples_.empty()) {
    status.start = samples_.front().stamp;
    status.end = samples_.back().stamp;
  }
  return status;
}

double ImuBuffer::maxAllowedTimeGap() const
{
  return max_allowed_time_gap_seconds_;
}

bool ImuBuffer::integrateRotation(
  const std::vector<ImuSample> & samples,
  const rclcpp::Time & from,
  const rclcpp::Time & to,
  double max_allowed_time_gap_seconds,
  Eigen::Quaterniond & q_from_to,
  std::string * reason)
{
  q_from_to = Eigen::Quaterniond::Identity();

  if (from.nanoseconds() == to.nanoseconds()) {
    return true;
  }

  if (to < from) {
    // 反向查询时先正向积分再取逆，供“补偿到 scan_end”这类场景使用。
    Eigen::Quaterniond q_to_from = Eigen::Quaterniond::Identity();
    if (!integrateRotation(samples, to, from, max_allowed_time_gap_seconds, q_to_from, reason)) {
      return false;
    }
    q_from_to = q_to_from.inverse();
    q_from_to.normalize();
    return true;
  }

  if (samples.size() < 2) {
    if (reason) {
      *reason = "IMU buffer has fewer than two angular velocity samples";
    }
    return false;
  }

  if (from < samples.front().stamp || to > samples.back().stamp) {
    if (reason) {
      std::ostringstream oss;
      oss << "IMU integration range is outside buffer. query=[" << from.seconds()
          << ", " << to.seconds() << "] buffer=[" << samples.front().stamp.seconds()
          << ", " << samples.back().stamp.seconds() << "]";
      *reason = oss.str();
    }
    return false;
  }

  std::vector<rclcpp::Time> knots;
  knots.push_back(from);
  // 把查询区间内的 IMU 样本时间作为积分节点，两端角速度用线性插值补齐。
  for (const auto & sample : samples) {
    if (sample.stamp > from && sample.stamp < to) {
      knots.push_back(sample.stamp);
    }
  }
  knots.push_back(to);

  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  for (std::size_t i = 1; i < knots.size(); ++i) {
    const rclcpp::Time & a = knots[i - 1];
    const rclcpp::Time & b = knots[i];
    const double dt = (b - a).seconds();
    if (dt < 0.0 || dt > max_allowed_time_gap_seconds) {
      if (reason) {
        std::ostringstream oss;
        oss << "IMU integration gap " << dt
            << " s exceeds max_allowed_imu_gap " << max_allowed_time_gap_seconds << " s";
        *reason = oss.str();
      }
      return false;
    }

    Eigen::Vector3d wa = Eigen::Vector3d::Zero();
    Eigen::Vector3d wb = Eigen::Vector3d::Zero();
    if (!interpolateAngularVelocity(samples, a, max_allowed_time_gap_seconds, wa, reason) ||
        !interpolateAngularVelocity(samples, b, max_allowed_time_gap_seconds, wb, reason))
    {
      return false;
    }

    const Eigen::Vector3d average_omega = 0.5 * (wa + wb);
    // 用梯形平均角速度和指数映射积分小段旋转，避免简单欧拉累加的姿态误差。
    q = q * deltaQuaternion(average_omega * dt);
    q.normalize();
  }

  q_from_to = q;
  return true;
}

void ImuBuffer::pruneUnlocked()
{
  if (samples_.empty()) {
    return;
  }

  const rclcpp::Time newest = samples_.back().stamp;
  auto first_keep = std::find_if(
    samples_.begin(),
    samples_.end(),
    [&](const ImuSample & sample) {
      return (newest - sample.stamp).seconds() <= duration_seconds_;
    });
  samples_.erase(samples_.begin(), first_keep);
}

bool ImuBuffer::interpolateAngularVelocity(
  const std::vector<ImuSample> & samples,
  const rclcpp::Time & stamp,
  double max_allowed_time_gap_seconds,
  Eigen::Vector3d & angular_velocity,
  std::string * reason)
{
  if (samples.empty()) {
    if (reason) {
      *reason = "IMU buffer is empty";
    }
    return false;
  }

  auto upper = std::lower_bound(
    samples.begin(),
    samples.end(),
    stamp,
    [](const ImuSample & lhs, const rclcpp::Time & query_stamp) {
      return lhs.stamp < query_stamp;
    });

  if (upper != samples.end() && upper->stamp.nanoseconds() == stamp.nanoseconds()) {
    angular_velocity = upper->angular_velocity_lidar;
    return true;
  }

  if (upper == samples.begin() || upper == samples.end()) {
    if (reason) {
      *reason = "cannot bracket query time in IMU buffer";
    }
    return false;
  }

  const auto & after = *upper;
  const auto & before = *(upper - 1);
  const double gap = (after.stamp - before.stamp).seconds();
  if (gap <= 0.0 || gap > max_allowed_time_gap_seconds) {
    if (reason) {
      std::ostringstream oss;
      oss << "IMU interpolation gap " << gap
          << " s exceeds max_allowed_imu_gap " << max_allowed_time_gap_seconds << " s";
      *reason = oss.str();
    }
    return false;
  }

  const double alpha = (stamp - before.stamp).seconds() / gap;
  angular_velocity =
    before.angular_velocity_lidar +
    alpha * (after.angular_velocity_lidar - before.angular_velocity_lidar);
  return true;
}

Eigen::Quaterniond ImuBuffer::deltaQuaternion(const Eigen::Vector3d & rotation_vector)
{
  const double angle = rotation_vector.norm();
  if (angle < 1.0e-12) {
    return Eigen::Quaterniond::Identity();
  }

  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

}  // namespace mid360_deskew
