// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_mppi_ego_controller/critics/ego_trajectory_critic.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iterator>
#include <limits>
#include <sstream>
#include <type_traits>
#include <utility>

#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mppi_ego::critics
{

namespace
{

constexpr float kEpsilon = 1e-4f;
constexpr float kMaxReasonableFloat = 1.0e6f;

bool isReasonableFloat(float value)
{
  return std::fabs(value) <= kMaxReasonableFloat;
}

template<typename T, typename = void>
struct has_member_yaw : std::false_type {};

template<typename T>
struct has_member_yaw<T, std::void_t<decltype(std::declval<T>().yaw)>>: std::true_type {};

template<typename T, typename = void>
struct has_member_velocity : std::false_type {};

template<typename T>
struct has_member_velocity<T, std::void_t<decltype(std::declval<T>().velocity)>>
  : std::true_type {};

template<typename T, typename = void>
struct has_member_curvature : std::false_type {};

template<typename T>
struct has_member_curvature<T, std::void_t<decltype(std::declval<T>().curvature)>>
  : std::true_type {};

template<typename T, typename = void>
struct has_member_acceleration : std::false_type {};

template<typename T>
struct has_member_acceleration<T, std::void_t<decltype(std::declval<T>().acceleration)>>
  : std::true_type {};

template<typename T, typename = void>
struct has_member_t : std::false_type {};

template<typename T>
struct has_member_t<T, std::void_t<decltype(std::declval<T>().t)>>: std::true_type {};

template<typename T, typename = void>
struct has_member_time : std::false_type {};

template<typename T>
struct has_member_time<T, std::void_t<decltype(std::declval<T>().time)>>: std::true_type {};

template<typename T, typename = void>
struct has_member_time_from_start : std::false_type {};

template<typename T>
struct has_member_time_from_start<T, std::void_t<decltype(std::declval<T>().time_from_start)>>
  : std::true_type {};

template<typename T, typename = void>
struct has_sec_nanosec : std::false_type {};

template<typename T>
struct has_sec_nanosec<T,
  std::void_t<decltype(std::declval<T>().sec), decltype(std::declval<T>().nanosec)>>
  : std::true_type {};

template<typename T>
float timeFieldToSeconds(const T & value)
{
  if constexpr (std::is_arithmetic_v<std::decay_t<T>>) {
    return static_cast<float>(value);
  } else if constexpr (has_sec_nanosec<T>::value) {
    return static_cast<float>(value.sec) + static_cast<float>(value.nanosec) * 1e-9f;
  } else {
    return 0.0f;
  }
}

template<typename PointT>
float getPointTime(const PointT & point, size_t idx, float dt)
{
  (void)point;
  if constexpr (has_member_time_from_start<PointT>::value) {
    return timeFieldToSeconds(point.time_from_start);
  } else if constexpr (has_member_t<PointT>::value) {
    return timeFieldToSeconds(point.t);
  } else if constexpr (has_member_time<PointT>::value) {
    return timeFieldToSeconds(point.time);
  } else {
    return static_cast<float>(idx) * dt;
  }
}

template<typename PointT>
float getPointYaw(const PointT & point, bool & valid)
{
  (void)point;
  if constexpr (has_member_yaw<PointT>::value) {
    const auto yaw = static_cast<float>(point.yaw);
    valid = isReasonableFloat(yaw);
    return valid ? yaw : 0.0f;
  } else {
    valid = false;
    return 0.0f;
  }
}

template<typename PointT>
float getPointSpeed(const PointT & point, bool & valid)
{
  (void)point;
  if constexpr (has_member_velocity<PointT>::value) {
    const auto speed = static_cast<float>(point.velocity);
    valid = isReasonableFloat(speed);
    return valid ? speed : 0.0f;
  } else {
    valid = false;
    return 0.0f;
  }
}

template<typename PointT>
float getPointCurvature(const PointT & point, bool & valid)
{
  (void)point;
  if constexpr (has_member_curvature<PointT>::value) {
    const auto curvature = static_cast<float>(point.curvature);
    valid = isReasonableFloat(curvature);
    return valid ? curvature : 0.0f;
  } else {
    valid = false;
    return 0.0f;
  }
}

template<typename PointT>
float getPointAcceleration(const PointT & point, bool & valid)
{
  (void)point;
  if constexpr (has_member_acceleration<PointT>::value) {
    const auto acceleration = static_cast<float>(point.acceleration);
    valid = isReasonableFloat(acceleration);
    return valid ? acceleration : 0.0f;
  } else {
    valid = false;
    return 0.0f;
  }
}

}  // namespace

void EgoTrajectoryCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(ego_trajectory_topic_, "ego_trajectory_topic", std::string("/ego_reference_trajectory"));
  getParam(max_reference_age_, "max_reference_age", 1.0);
  getParam(lookahead_time_, "lookahead_time", 1.0);
  getParam(max_match_distance_, "max_match_distance", 1.0);
  getParam(reference_dt_, "reference_dt", 0.1);
  getParam(max_reference_speed_, "max_reference_speed", 3.0);
  getParam(trajectory_point_step_, "trajectory_point_step", 1);
  getParam(power_, "cost_power", 1);
  getParam(cost_weight_, "cost_weight", 8.0f);
  getParam(position_weight_, "position_weight", 1.0f);
  getParam(yaw_weight_, "yaw_weight", 0.4f);
  getParam(velocity_weight_, "velocity_weight", 0.2f);
  getParam(velocity_direction_weight_, "velocity_direction_weight", 0.3f);
  getParam(velocity_direction_min_speed_, "velocity_direction_min_speed", 0.05f);
  getParam(distance_penalty_weight_, "distance_penalty_weight", 10.0f);
  getParam(use_curvature_cost_, "use_curvature_cost", false);
  getParam(curvature_weight_, "curvature_weight", 0.0f);
  getParam(max_reference_curvature_, "max_reference_curvature", 5.0);
  getParam(max_candidate_curvature_, "max_candidate_curvature", 8.0);
  getParam(use_acceleration_cost_, "use_acceleration_cost", false);
  getParam(acceleration_weight_, "acceleration_weight", 0.0f);
  getParam(max_reference_acceleration_, "max_reference_acceleration", 5.0);
  getParam(max_candidate_acceleration_, "max_candidate_acceleration", 8.0);
  getParam(velocity_frame_, "velocity_frame", std::string("base"));
  getParam(debug_enabled_, "debug_enabled", false);
  getParam(debug_log_period_, "debug_log_period", 1.0);
  getParam(debug_sample_candidates_, "debug_sample_candidates", 3);
  getParam(debug_check_trajectory_on_callback_, "debug_check_trajectory_on_callback", true);
  getParam(debug_check_score_runtime_, "debug_check_score_runtime", true);
  getParam(debug_max_point_gap_, "debug_max_point_gap", 1.0);
  getParam(debug_min_point_gap_, "debug_min_point_gap", 0.001);
  getParam(debug_max_yaw_jump_, "debug_max_yaw_jump", 1.57);
  getParam(debug_max_speed_jump_, "debug_max_speed_jump", 2.0);
  getParam(debug_max_curvature_, "debug_max_curvature", 8.0);
  getParam(debug_max_acceleration_, "debug_max_acceleration", 10.0);
  getParam(debug_max_cost_, "debug_max_cost", 1.0e6);
  getParam(debug_large_error_distance_, "debug_large_error_distance", 2.0);

  reference_dt_ = std::max(reference_dt_, 1e-3);
  max_reference_age_ = std::max(max_reference_age_, 0.0);
  lookahead_time_ = std::max(lookahead_time_, 0.0);
  max_match_distance_ = std::max(max_match_distance_, 0.0);
  max_reference_speed_ = std::max(max_reference_speed_, 0.0);
  velocity_direction_min_speed_ = std::max(velocity_direction_min_speed_, 0.0f);
  trajectory_point_step_ = std::max<size_t>(trajectory_point_step_, 1);
  curvature_weight_ = std::max(curvature_weight_, 0.0f);
  max_reference_curvature_ = std::max(max_reference_curvature_, 0.0);
  max_candidate_curvature_ = std::max(max_candidate_curvature_, 0.0);
  acceleration_weight_ = std::max(acceleration_weight_, 0.0f);
  max_reference_acceleration_ = std::max(max_reference_acceleration_, 0.0);
  max_candidate_acceleration_ = std::max(max_candidate_acceleration_, 0.0);
  debug_log_period_ = std::max(debug_log_period_, 0.001);
  debug_sample_candidates_ = std::max<size_t>(debug_sample_candidates_, 0);
  debug_max_point_gap_ = std::max(debug_max_point_gap_, 0.0);
  debug_min_point_gap_ = std::max(debug_min_point_gap_, 0.0);
  debug_max_yaw_jump_ = std::max(debug_max_yaw_jump_, 0.0);
  debug_max_speed_jump_ = std::max(debug_max_speed_jump_, 0.0);
  debug_max_curvature_ = std::max(debug_max_curvature_, 0.0);
  debug_max_acceleration_ = std::max(debug_max_acceleration_, 0.0);
  debug_max_cost_ = std::max(debug_max_cost_, 0.0);
  debug_large_error_distance_ = std::max(debug_large_error_distance_, 0.0);

  std::transform(
    velocity_frame_.begin(), velocity_frame_.end(), velocity_frame_.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  velocity_frame_is_global_ = velocity_frame_ == "global";
  if (velocity_frame_ != "base" && velocity_frame_ != "global") {
    RCLCPP_WARN(
      logger_,
      "Unsupported EgoTrajectoryCritic velocity_frame '%s'; using 'base'.",
      velocity_frame_.c_str());
    velocity_frame_ = "base";
    velocity_frame_is_global_ = false;
  }

  auto node = parent_.lock();
  ego_trajectory_sub_ = node->create_subscription<ego_planner_msgs::msg::Trajectory>(
    ego_trajectory_topic_, rclcpp::QoS(10),
    std::bind(&EgoTrajectoryCritic::trajectoryCallback, this, std::placeholders::_1));

  last_trajectory_update_ = node->now();
  RCLCPP_INFO(
    logger_,
    "EgoTrajectoryCritic subscribed to '%s'", ego_trajectory_topic_.c_str());
}

void EgoTrajectoryCritic::trajectoryCallback(
  const ego_planner_msgs::msg::Trajectory::SharedPtr msg)
{
  auto trajectory = buildTrajectory(*msg);
  auto node = parent_.lock();
  const double trajectory_dt = msg->time_step > 0.0f ? msg->time_step : reference_dt_;

  debugCheckReferenceTrajectory(trajectory, msg->header.frame_id, trajectory_dt);

  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  ego_trajectory_ = std::move(trajectory);
  ego_trajectory_dt_ = std::max(trajectory_dt, 1e-3);
  last_trajectory_update_ = node->now();
}

std::vector<EgoTrajectoryCritic::EgoTrajPoint>
EgoTrajectoryCritic::buildTrajectory(
  const ego_planner_msgs::msg::Trajectory & trajectory_msg)
{
  std::vector<EgoTrajPoint> trajectory;
  if (trajectory_msg.points.size() < 2) {
    return trajectory;
  }

  trajectory.reserve(trajectory_msg.points.size());
  std::vector<bool> yaw_valid;
  std::vector<bool> curvature_valid;
  std::vector<bool> acceleration_valid;
  yaw_valid.reserve(trajectory_msg.points.size());
  curvature_valid.reserve(trajectory_msg.points.size());
  acceleration_valid.reserve(trajectory_msg.points.size());

  const auto trajectory_dt = static_cast<float>(
    trajectory_msg.time_step > 0.0f ? trajectory_msg.time_step : reference_dt_);
  const std::string target_frame = costmap_ros_->getGlobalFrameID();
  const bool should_transform =
    !trajectory_msg.header.frame_id.empty() && trajectory_msg.header.frame_id != target_frame;

  for (size_t i = 0; i != trajectory_msg.points.size(); ++i) {
    const auto & trajectory_point = trajectory_msg.points[i];
    bool has_valid_yaw = false;
    bool has_valid_speed = false;
    bool has_valid_curvature = false;
    bool has_valid_acceleration = false;
    const auto point_yaw = getPointYaw(trajectory_point, has_valid_yaw);
    const auto point_speed = getPointSpeed(trajectory_point, has_valid_speed);
    const auto point_curvature = getPointCurvature(trajectory_point, has_valid_curvature);
    const auto point_acceleration =
      getPointAcceleration(trajectory_point, has_valid_acceleration);

    geometry_msgs::msg::PoseStamped reference_pose;
    reference_pose.header = trajectory_msg.header;
    reference_pose.pose.position.x = trajectory_point.x;
    reference_pose.pose.position.y = trajectory_point.y;
    reference_pose.pose.position.z = trajectory_point.z;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, has_valid_yaw ? point_yaw : 0.0f);
    reference_pose.pose.orientation = tf2::toMsg(quaternion);

    geometry_msgs::msg::PoseStamped transformed_pose = reference_pose;
    if (should_transform) {
      try {
        transformed_pose = costmap_ros_->getTfBuffer()->transform(
          reference_pose, target_frame, tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          logger_, *parent_.lock()->get_clock(), 2000,
          "Failed to transform Ego trajectory from '%s' to '%s': %s",
          trajectory_msg.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        return {};
      }
    }

    EgoTrajPoint point;
    point.t = getPointTime(trajectory_point, i, trajectory_dt);
    point.x = static_cast<float>(transformed_pose.pose.position.x);
    point.y = static_cast<float>(transformed_pose.pose.position.y);
    point.yaw = static_cast<float>(tf2::getYaw(transformed_pose.pose.orientation));
    point.speed = std::min(
      std::max(has_valid_speed ? point_speed : 0.0f, 0.0f),
      static_cast<float>(max_reference_speed_));
    point.curvature = clampSymmetric(point_curvature, max_reference_curvature_);
    point.acceleration = clampSymmetric(point_acceleration, max_reference_acceleration_);

    trajectory.push_back(point);
    yaw_valid.push_back(has_valid_yaw);
    curvature_valid.push_back(has_valid_curvature);
    acceleration_valid.push_back(has_valid_acceleration);
  }

  if (trajectory.empty()) {
    return trajectory;
  }

  const float time_origin = trajectory.front().t;
  for (size_t i = 0; i != trajectory.size(); ++i) {
    auto & point = trajectory[i];
    point.t -= time_origin;
    if (!isReasonableFloat(point.t)) {
      point.t = i == 0 ? 0.0f : trajectory[i - 1].t + trajectory_dt;
    }
    if (i == 0) {
      point.t = 0.0f;
    } else if (point.t <= trajectory[i - 1].t) {
      point.t = trajectory[i - 1].t + trajectory_dt;
    }
  }

  fillMissingYaws(trajectory, yaw_valid);
  fillMissingCurvatures(trajectory, curvature_valid);
  fillMissingAccelerations(trajectory, acceleration_valid);

  return trajectory;
}

EgoTrajectoryCritic::EgoTrajPoint
EgoTrajectoryCritic::sampleTrajectoryByTime(
  const std::vector<EgoTrajPoint> & trajectory,
  float query_time) const
{
  if (trajectory.empty()) {
    return {};
  }

  if (query_time <= trajectory.front().t) {
    return trajectory.front();
  }

  if (query_time >= trajectory.back().t) {
    return trajectory.back();
  }

  const auto upper = std::lower_bound(
    trajectory.begin(), trajectory.end(), query_time,
    [](const EgoTrajPoint & point, float time) {
      return point.t < time;
    });

  if (upper == trajectory.begin()) {
    return trajectory.front();
  }

  const auto & next = *upper;
  const auto & prev = *(upper - 1);
  const float duration = std::max(next.t - prev.t, kEpsilon);
  const float ratio = std::clamp((query_time - prev.t) / duration, 0.0f, 1.0f);

  EgoTrajPoint sample;
  sample.t = query_time;
  sample.x = prev.x + ratio * (next.x - prev.x);
  sample.y = prev.y + ratio * (next.y - prev.y);
  sample.yaw = static_cast<float>(
    angles::normalize_angle(
      prev.yaw + ratio * angles::shortest_angular_distance(prev.yaw, next.yaw)));
  sample.speed = prev.speed + ratio * (next.speed - prev.speed);
  sample.curvature = prev.curvature + ratio * (next.curvature - prev.curvature);
  sample.acceleration = prev.acceleration + ratio * (next.acceleration - prev.acceleration);
  return sample;
}

void EgoTrajectoryCritic::fillMissingYaws(
  std::vector<EgoTrajPoint> & trajectory,
  const std::vector<bool> & yaw_valid) const
{
  if (trajectory.size() < 2) {
    return;
  }

  for (size_t i = 0; i != trajectory.size(); ++i) {
    if (i >= yaw_valid.size() || !yaw_valid[i]) {
      trajectory[i].yaw = estimateYawAt(trajectory, i);
    }
  }
}

void EgoTrajectoryCritic::fillMissingCurvatures(
  std::vector<EgoTrajPoint> & trajectory,
  const std::vector<bool> & curvature_valid) const
{
  if (trajectory.size() < 3) {
    for (auto & point : trajectory) {
      point.curvature = 0.0f;
    }
    return;
  }

  std::vector<float> estimated_curvatures(trajectory.size(), 0.0f);
  for (size_t i = 1; i + 1 < trajectory.size(); ++i) {
    const auto & prev = trajectory[i - 1];
    const auto & next = trajectory[i + 1];
    const float ds = std::hypot(next.x - prev.x, next.y - prev.y);
    const float dyaw = static_cast<float>(
      angles::shortest_angular_distance(prev.yaw, next.yaw));
    estimated_curvatures[i] = clampSymmetric(
      dyaw / std::max(ds, kEpsilon), max_reference_curvature_);
  }

  estimated_curvatures.front() = estimated_curvatures[1];
  estimated_curvatures.back() = estimated_curvatures[estimated_curvatures.size() - 2];

  for (size_t i = 0; i != trajectory.size(); ++i) {
    if (i >= curvature_valid.size() || !curvature_valid[i]) {
      trajectory[i].curvature = estimated_curvatures[i];
    }
    trajectory[i].curvature = clampSymmetric(trajectory[i].curvature, max_reference_curvature_);
  }
}

void EgoTrajectoryCritic::fillMissingAccelerations(
  std::vector<EgoTrajPoint> & trajectory,
  const std::vector<bool> & acceleration_valid) const
{
  if (trajectory.size() < 2) {
    return;
  }

  std::vector<float> estimated_accelerations(trajectory.size(), 0.0f);
  for (size_t i = 1; i != trajectory.size(); ++i) {
    const float dt = std::max(trajectory[i].t - trajectory[i - 1].t, kEpsilon);
    estimated_accelerations[i] = clampSymmetric(
      (trajectory[i].speed - trajectory[i - 1].speed) / dt,
      max_reference_acceleration_);
  }
  estimated_accelerations.front() = estimated_accelerations[1];

  for (size_t i = 0; i != trajectory.size(); ++i) {
    if (i >= acceleration_valid.size() || !acceleration_valid[i]) {
      trajectory[i].acceleration = estimated_accelerations[i];
    }
    trajectory[i].acceleration = clampSymmetric(
      trajectory[i].acceleration, max_reference_acceleration_);
  }
}

float EgoTrajectoryCritic::estimateYawAt(
  const std::vector<EgoTrajPoint> & trajectory,
  size_t idx) const
{
  if (trajectory.size() < 2) {
    return 0.0f;
  }

  const size_t prev_idx = idx == 0 ? 0 : idx - 1;
  const size_t next_idx = std::min(idx + 1, trajectory.size() - 1);
  const float dx = trajectory[next_idx].x - trajectory[prev_idx].x;
  const float dy = trajectory[next_idx].y - trajectory[prev_idx].y;
  if (std::hypot(dx, dy) < kEpsilon) {
    return trajectory[idx].yaw;
  }

  return std::atan2(dy, dx);
}

float EgoTrajectoryCritic::clampSymmetric(float value, double limit) const
{
  if (!isReasonableFloat(value)) {
    return 0.0f;
  }

  const auto limit_f = static_cast<float>(std::max(limit, 0.0));
  if (limit_f <= 0.0f) {
    return 0.0f;
  }

  return std::clamp(value, -limit_f, limit_f);
}

size_t EgoTrajectoryCritic::findClosestReferenceIndex(
  const std::vector<EgoTrajPoint> & trajectory,
  float x, float y) const
{
  size_t closest_idx = 0;
  float closest_dist_sq = std::numeric_limits<float>::max();

  for (size_t i = 0; i != trajectory.size(); ++i) {
    const float dx = trajectory[i].x - x;
    const float dy = trajectory[i].y - y;
    const float dist_sq = dx * dx + dy * dy;
    if (dist_sq < closest_dist_sq) {
      closest_dist_sq = dist_sq;
      closest_idx = i;
    }
  }

  return closest_idx;
}

int64_t EgoTrajectoryCritic::debugThrottleMs() const
{
  return static_cast<int64_t>(std::max(debug_log_period_, 0.001) * 1000.0);
}

bool EgoTrajectoryCritic::debugShouldRun(double & last_run_time, double current_time) const
{
  const double period = std::max(debug_log_period_, 0.001);
  if (last_run_time >= 0.0 &&
    current_time >= last_run_time &&
    current_time - last_run_time < period)
  {
    return false;
  }

  last_run_time = current_time;
  return true;
}

void EgoTrajectoryCritic::debugCheckReferenceTrajectory(
  const std::vector<EgoTrajPoint> & trajectory,
  const std::string & frame_id,
  double trajectory_dt) const
{
  if (!debug_enabled_ || !debug_check_trajectory_on_callback_) {
    return;
  }

  const auto node = parent_.lock();
  if (!node) {
    return;
  }

  if (!debugShouldRun(last_reference_debug_time_, node->now().seconds())) {
    return;
  }

  const auto throttle_ms = debugThrottleMs();
  const auto target_frame = costmap_ros_->getGlobalFrameID();

  if (trajectory.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Ego trajectory has %zu point(s); need at least 2 points for scoring.",
      trajectory.size());
  }

  if (frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Ego trajectory frame is empty; check publisher header.frame_id.");
  } else if (frame_id != target_frame) {
    RCLCPP_INFO_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Ego trajectory frame '%s' differs from costmap global frame '%s'; transform was attempted.",
      frame_id.c_str(), target_frame.c_str());
  }

  if (trajectory_dt <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Ego trajectory_dt %.6f is non-positive; time alignment may be wrong.",
      trajectory_dt);
  } else if (trajectory_dt > 0.5) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Ego trajectory_dt %.6f is large; MPPI may undersample the Ego reference in time.",
      trajectory_dt);
  } else if (trajectory_dt < 0.001) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Ego trajectory_dt %.6f is very small; check time_step/reference_dt.",
      trajectory_dt);
  }

  if (trajectory.size() < 2) {
    return;
  }

  float min_gap = kMaxReasonableFloat;
  float max_gap = 0.0f;
  float sum_gap = 0.0f;
  float min_speed = kMaxReasonableFloat;
  float max_speed = 0.0f;
  float sum_speed = 0.0f;
  float max_yaw_jump = 0.0f;
  float max_speed_jump = 0.0f;
  float max_abs_curvature = 0.0f;
  float max_abs_acceleration = 0.0f;
  size_t nonfinite_count = 0;
  size_t first_nonfinite_idx = 0;

  for (size_t i = 0; i != trajectory.size(); ++i) {
    const auto & point = trajectory[i];
    const bool point_finite =
      isReasonableFloat(point.x) && isReasonableFloat(point.y) &&
      isReasonableFloat(point.yaw) && isReasonableFloat(point.speed) &&
      isReasonableFloat(point.curvature) && isReasonableFloat(point.acceleration);
    if (!point_finite) {
      if (nonfinite_count == 0) {
        first_nonfinite_idx = i;
      }
      ++nonfinite_count;
    }

    min_speed = std::min(min_speed, point.speed);
    max_speed = std::max(max_speed, point.speed);
    sum_speed += point.speed;
    max_abs_curvature = std::max(max_abs_curvature, std::fabs(point.curvature));
    max_abs_acceleration = std::max(max_abs_acceleration, std::fabs(point.acceleration));

    if (i == 0) {
      continue;
    }

    const auto & prev = trajectory[i - 1];
    const float gap = std::hypot(point.x - prev.x, point.y - prev.y);
    min_gap = std::min(min_gap, gap);
    max_gap = std::max(max_gap, gap);
    sum_gap += gap;
    max_yaw_jump = std::max(
      max_yaw_jump,
      static_cast<float>(std::fabs(angles::shortest_angular_distance(prev.yaw, point.yaw))));
    max_speed_jump = std::max(max_speed_jump, std::fabs(point.speed - prev.speed));
  }

  const auto segment_count = static_cast<float>(trajectory.size() - 1);
  const float mean_gap = sum_gap / std::max(segment_count, 1.0f);
  const float mean_speed = sum_speed / static_cast<float>(trajectory.size());

  if (nonfinite_count > 0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Non-finite or out-of-range Ego trajectory value detected at first index %zu "
      "(%zu total); check publisher values and fast-math assumptions.",
      first_nonfinite_idx, nonfinite_count);
  }

  if (max_gap > static_cast<float>(debug_max_point_gap_)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Large point gap detected: max_gap=%.3f > %.3f; possible unordered trajectory "
      "points or frame jump.",
      max_gap, debug_max_point_gap_);
  }

  if (min_gap < static_cast<float>(debug_min_point_gap_)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Small point gap detected: min_gap=%.6f < %.6f; possible duplicate points.",
      min_gap, debug_min_point_gap_);
  }

  if (max_yaw_jump > static_cast<float>(debug_max_yaw_jump_)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Yaw jump detected: max_yaw_jump=%.3f > %.3f; possible angle wrapping issue.",
      max_yaw_jump, debug_max_yaw_jump_);
  }

  if (max_speed_jump > static_cast<float>(debug_max_speed_jump_)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Reference speed jump detected: max_speed_jump=%.3f > %.3f; check Ego velocity.",
      max_speed_jump, debug_max_speed_jump_);
  }

  if (max_speed < velocity_direction_min_speed_ &&
    mean_gap > static_cast<float>(debug_min_point_gap_))
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Reference speed is nearly zero while geometry moves; velocity may not be filled.");
  }

  if (max_abs_curvature > static_cast<float>(debug_max_curvature_)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Reference curvature is large: max_abs_curvature=%.3f > %.3f; check trajectory "
      "smoothness or yaw continuity.",
      max_abs_curvature, debug_max_curvature_);
  }

  if (max_abs_acceleration > static_cast<float>(debug_max_acceleration_)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Reference acceleration is large: max_abs_acceleration=%.3f > %.3f; check speed "
      "profile and trajectory_dt.",
      max_abs_acceleration, debug_max_acceleration_);
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *node->get_clock(), throttle_ms,
    "Ego reference summary: size=%zu frame='%s' target_frame='%s' dt=%.4f "
    "gap[min/max/mean]=%.3f/%.3f/%.3f speed[min/max/mean]=%.3f/%.3f/%.3f "
    "max_yaw_jump=%.3f max_curvature=%.3f max_acceleration=%.3f",
    trajectory.size(), frame_id.c_str(), target_frame.c_str(), trajectory_dt,
    min_gap, max_gap, mean_gap, min_speed, max_speed, mean_speed,
    max_yaw_jump, max_abs_curvature, max_abs_acceleration);
}

void EgoTrajectoryCritic::debugCheckScoreRuntime(
  const CriticData & data,
  const std::vector<EgoTrajPoint> & trajectory,
  size_t start_idx,
  double trajectory_dt,
  size_t time_steps) const
{
  if (!debug_enabled_ || !debug_check_score_runtime_) {
    return;
  }

  const auto node = parent_.lock();
  if (!node || trajectory.empty() || start_idx >= trajectory.size() || time_steps == 0) {
    return;
  }

  const auto throttle_ms = debugThrottleMs();
  const auto & start_ref = trajectory[start_idx];
  const auto robot_x = static_cast<float>(data.state.pose.pose.position.x);
  const auto robot_y = static_cast<float>(data.state.pose.pose.position.y);
  const float start_distance = std::hypot(robot_x - start_ref.x, robot_y - start_ref.y);

  RCLCPP_INFO_THROTTLE(
    logger_, *node->get_clock(), throttle_ms,
    "Ego score match: robot=(%.3f, %.3f) start_idx=%zu ref=(t=%.3f, x=%.3f, y=%.3f) "
    "distance=%.3f",
    robot_x, robot_y, start_idx, start_ref.t, start_ref.x, start_ref.y, start_distance);

  if (start_distance > static_cast<float>(debug_large_error_distance_)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Robot is far from closest Ego reference point: distance=%.3f > %.3f; possible "
      "frame mismatch, stale trajectory, or wrong transform.",
      start_distance, debug_large_error_distance_);
  }

  if (data.model_dt <= 0.0f) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "MPPI model_dt %.6f is non-positive; time alignment is invalid.",
      data.model_dt);
  } else {
    const float dt_ratio = static_cast<float>(
      data.model_dt / std::max(trajectory_dt, static_cast<double>(kEpsilon)));
    if (dt_ratio > 10.0f || dt_ratio < 0.1f) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), throttle_ms,
        "MPPI model_dt and Ego trajectory_dt differ greatly: model_dt=%.6f "
        "trajectory_dt=%.6f ratio=%.3f; time alignment may be wrong.",
        data.model_dt, trajectory_dt, dt_ratio);
    }
  }

  std::array<size_t, 3> debug_steps{};
  size_t debug_step_count = 0;
  const auto add_debug_step = [&](size_t step) {
      step = std::min(step, time_steps - 1);
      for (size_t i = 0; i != debug_step_count; ++i) {
        if (debug_steps[i] == step) {
          return;
        }
      }
      debug_steps[debug_step_count++] = step;
    };
  add_debug_step(0);
  add_debug_step(time_steps / 2);
  add_debug_step(time_steps - 1);

  std::array<float, 3> debug_query_times{};
  std::array<EgoTrajPoint, 3> debug_refs{};
  std::array<size_t, 3> debug_ref_indices{};
  std::ostringstream ref_stream;
  ref_stream << "Ego time samples:";
  for (size_t i = 0; i != debug_step_count; ++i) {
    const auto step = debug_steps[i];
    const float query_time = start_ref.t + static_cast<float>(step) *
      std::max(data.model_dt, kEpsilon);
    const auto ref = sampleTrajectoryByTime(trajectory, query_time);
    const auto upper = std::lower_bound(
      trajectory.begin(), trajectory.end(), query_time,
      [](const EgoTrajPoint & point, float time) {
        return point.t < time;
      });
    const auto ref_iter = upper == trajectory.end() ? std::prev(trajectory.end()) : upper;
    const auto ref_idx = static_cast<size_t>(std::distance(trajectory.begin(), ref_iter));
    debug_query_times[i] = query_time;
    debug_refs[i] = ref;
    debug_ref_indices[i] = ref_idx;
    ref_stream << " step=" << step << " mppi_time=" << step * data.model_dt
               << " ref_idx~=" << ref_idx << " ref_time=" << ref.t
               << " ref=(" << ref.x << "," << ref.y << "," << ref.yaw
               << "," << ref.speed << ")";
  }
  RCLCPP_INFO_THROTTLE(
    logger_, *node->get_clock(), throttle_ms, "%s", ref_stream.str().c_str());

  const size_t batch_size = data.costs.shape(0);
  const size_t candidate_count = std::min(debug_sample_candidates_, batch_size);
  std::vector<size_t> candidate_indices;
  candidate_indices.reserve(candidate_count);
  if (candidate_count == 1) {
    candidate_indices.push_back(0);
  } else if (candidate_count > 1) {
    for (size_t i = 0; i != candidate_count; ++i) {
      const size_t candidate = i * (batch_size - 1) / (candidate_count - 1);
      if (candidate_indices.empty() || candidate_indices.back() != candidate) {
        candidate_indices.push_back(candidate);
      }
    }
  }

  size_t large_error_count = 0;
  float max_position_error = 0.0f;
  float max_yaw_error = 0.0f;
  float max_speed_error = 0.0f;

  std::ostringstream candidate_stream;
  candidate_stream << "Ego candidate samples:";
  for (const auto candidate : candidate_indices) {
    for (size_t i = 0; i != debug_step_count; ++i) {
      const auto step = debug_steps[i];
      const auto & ref = debug_refs[i];
      const float x = data.trajectories.x(candidate, step);
      const float y = data.trajectories.y(candidate, step);
      const float yaw = data.trajectories.yaws(candidate, step);
      const float vx = data.state.vx(candidate, step);
      const float vy = data.state.vy(candidate, step);
      const float speed = std::hypot(vx, vy);
      const float position_error = std::hypot(x - ref.x, y - ref.y);
      const float yaw_error = static_cast<float>(
        std::fabs(angles::shortest_angular_distance(yaw, ref.yaw)));
      const float speed_error = std::fabs(speed - ref.speed);

      if (position_error > static_cast<float>(debug_large_error_distance_)) {
        ++large_error_count;
      }
      max_position_error = std::max(max_position_error, position_error);
      max_yaw_error = std::max(max_yaw_error, yaw_error);
      max_speed_error = std::max(max_speed_error, speed_error);

      candidate_stream << " c=" << candidate << " step=" << step
                       << " ref_idx~=" << debug_ref_indices[i]
                       << " query_time=" << debug_query_times[i]
                       << " cand=(" << x << "," << y << "," << yaw << "," << speed
                       << ") ref=(" << ref.x << "," << ref.y << "," << ref.yaw
                       << "," << ref.speed << ") err=(" << position_error << ","
                       << yaw_error << "," << speed_error << ")";
    }
  }

  if (!candidate_indices.empty()) {
    RCLCPP_INFO_THROTTLE(
      logger_, *node->get_clock(), throttle_ms, "%s", candidate_stream.str().c_str());
  }

  if (large_error_count > 0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Large MPPI/Ego position errors detected in sampled candidate points: count=%zu "
      "max_position_error=%.3f max_yaw_error=%.3f max_speed_error=%.3f; possible "
      "time offset, frame mismatch, stale trajectory, or wrong start_idx.",
      large_error_count, max_position_error, max_yaw_error, max_speed_error);
  }
}

void EgoTrajectoryCritic::debugCheckAddedCosts(
  const xt::xtensor<float, 1> & added_costs) const
{
  if (!debug_enabled_ || !debug_check_score_runtime_) {
    return;
  }

  const auto node = parent_.lock();
  if (!node || added_costs.shape(0) == 0) {
    return;
  }

  const auto throttle_ms = debugThrottleMs();
  const size_t batch_size = added_costs.shape(0);
  const size_t candidate_count = std::min(
    std::max<size_t>(debug_sample_candidates_, 1), batch_size);
  size_t nonfinite_count = 0;
  size_t first_nonfinite_idx = 0;
  float max_added_cost = 0.0f;
  size_t max_added_idx = 0;

  for (size_t sample = 0; sample != candidate_count; ++sample) {
    const size_t i = candidate_count == 1 ?
      0 : sample * (batch_size - 1) / (candidate_count - 1);
    const float added_cost = added_costs(i);
    if (!isReasonableFloat(added_cost)) {
      if (nonfinite_count == 0) {
        first_nonfinite_idx = i;
      }
      ++nonfinite_count;
      continue;
    }

    if (added_cost > max_added_cost) {
      max_added_cost = added_cost;
      max_added_idx = i;
    }
  }

  if (nonfinite_count > 0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "Non-finite or out-of-range cost detected in EgoTrajectoryCritic sampled added cost "
      "at first candidate %zu (%zu sampled); check weights, divisions, and fast-math "
      "assumptions.",
      first_nonfinite_idx, nonfinite_count);
  }

  if (max_added_cost > static_cast<float>(debug_max_cost_)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), throttle_ms,
      "EgoTrajectoryCritic sampled added cost is large: max_added_cost=%.3f at candidate "
      "%zu > %.3f; cost may be exploding.",
      max_added_cost, max_added_idx, debug_max_cost_);
  }
}

void EgoTrajectoryCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  std::vector<EgoTrajPoint> trajectory;
  double trajectory_dt;
  rclcpp::Time last_update;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory = ego_trajectory_;
    trajectory_dt = ego_trajectory_dt_;
    last_update = last_trajectory_update_;
  }

  if (trajectory.size() < 2) {
    return;
  }

  auto node = parent_.lock();
  const auto now = node->now();
  const auto reference_age = now - last_update;
  if (max_reference_age_ > 0.0 &&
    reference_age > rclcpp::Duration::from_seconds(max_reference_age_))
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "Ego trajectory age %.3fs exceeds max_reference_age %.3fs; skipping EgoTrajectoryCritic.",
      reference_age.seconds(), max_reference_age_);
    stale_warning_active_ = true;
    return;
  }
  if (stale_warning_active_) {
    RCLCPP_INFO(
      logger_,
      "Ego trajectory recovered; latest reference age %.3fs.",
      reference_age.seconds());
    stale_warning_active_ = false;
  }

  size_t time_steps = data.trajectories.x.shape(1);
  if (time_steps == 0) {
    return;
  }

  const float model_dt = std::max(data.model_dt, kEpsilon);
  if (lookahead_time_ > 0.0) {
    const auto lookahead_steps = static_cast<size_t>(
      std::ceil(lookahead_time_ / static_cast<double>(model_dt)));
    time_steps = std::min(time_steps, std::max<size_t>(lookahead_steps, 1));
  }

  const auto robot_x = static_cast<float>(data.state.pose.pose.position.x);
  const auto robot_y = static_cast<float>(data.state.pose.pose.position.y);
  const size_t start_idx = findClosestReferenceIndex(trajectory, robot_x, robot_y);
  const float start_time = trajectory[start_idx].t;
  const bool debug_runtime =
    debug_enabled_ && debug_check_score_runtime_ &&
    debugShouldRun(last_score_debug_time_, now.seconds());
  if (debug_runtime) {
    debugCheckScoreRuntime(data, trajectory, start_idx, trajectory_dt, time_steps);
  }

  xt::xtensor<float, 1> accumulated_cost = xt::zeros<float>({data.costs.shape(0)});
  size_t sample_count = 0;

  for (size_t t = 0; t < time_steps; t += trajectory_point_step_) {
    const auto ref = sampleTrajectoryByTime(
      trajectory, start_time + static_cast<float>(t) * model_dt);

    const auto traj_x = xt::view(data.trajectories.x, xt::all(), t);
    const auto traj_y = xt::view(data.trajectories.y, xt::all(), t);
    const auto traj_yaw = xt::view(data.trajectories.yaws, xt::all(), t);
    const auto traj_vx = xt::view(data.state.vx, xt::all(), t);
    const auto traj_vy = xt::view(data.state.vy, xt::all(), t);

    auto position_error = xt::sqrt(
      xt::pow(traj_x - ref.x, 2) +
      xt::pow(traj_y - ref.y, 2));
    auto yaw_error = xt::cast<float>(
      xt::fabs(utils::shortest_angular_distance(traj_yaw, ref.yaw)));
    auto candidate_speed = xt::sqrt(xt::pow(traj_vx, 2) + xt::pow(traj_vy, 2));
    auto speed_error = xt::fabs(candidate_speed - ref.speed);

    auto velocity_x = xt::eval(traj_vx);
    auto velocity_y = xt::eval(traj_vy);
    if (!velocity_frame_is_global_) {
      // MPPI state.vx/vy are body-frame velocities in this controller; rotate to global by default.
      velocity_x = xt::eval(traj_vx * xt::cos(traj_yaw) - traj_vy * xt::sin(traj_yaw));
      velocity_y = xt::eval(traj_vx * xt::sin(traj_yaw) + traj_vy * xt::cos(traj_yaw));
    }
    auto velocity_yaw = xt::atan2(velocity_y, velocity_x);
    auto velocity_direction_error = xt::where(
      candidate_speed > velocity_direction_min_speed_,
      xt::cast<float>(xt::fabs(utils::shortest_angular_distance(velocity_yaw, ref.yaw))),
      0.0f);

    xt::xtensor<float, 1> step_cost = xt::eval(
      position_weight_ * position_error +
      yaw_weight_ * yaw_error +
      velocity_weight_ * speed_error +
      velocity_direction_weight_ * velocity_direction_error);

    if (use_curvature_cost_ && curvature_weight_ > 0.0f && t > 0) {
      const auto prev_x = xt::view(data.trajectories.x, xt::all(), t - 1);
      const auto prev_y = xt::view(data.trajectories.y, xt::all(), t - 1);
      const auto prev_yaw = xt::view(data.trajectories.yaws, xt::all(), t - 1);
      const auto ds_candidate = xt::sqrt(
        xt::pow(traj_x - prev_x, 2) +
        xt::pow(traj_y - prev_y, 2));
      const auto dyaw_candidate = utils::shortest_angular_distance(prev_yaw, traj_yaw);
      const auto candidate_curvature = xt::clip(
        dyaw_candidate / xt::maximum(ds_candidate, kEpsilon),
        -static_cast<float>(max_candidate_curvature_),
        static_cast<float>(max_candidate_curvature_));
      const auto curvature_error = xt::fabs(candidate_curvature - ref.curvature);
      step_cost += curvature_weight_ * curvature_error;
    }

    if (use_acceleration_cost_ && acceleration_weight_ > 0.0f && t > 0) {
      const auto prev_vx = xt::view(data.state.vx, xt::all(), t - 1);
      const auto prev_vy = xt::view(data.state.vy, xt::all(), t - 1);
      const auto prev_candidate_speed = xt::sqrt(
        xt::pow(prev_vx, 2) +
        xt::pow(prev_vy, 2));
      const auto candidate_acceleration = xt::clip(
        (candidate_speed - prev_candidate_speed) / model_dt,
        -static_cast<float>(max_candidate_acceleration_),
        static_cast<float>(max_candidate_acceleration_));
      const auto acceleration_error = xt::fabs(candidate_acceleration - ref.acceleration);
      step_cost += acceleration_weight_ * acceleration_error;
    }

    if (max_match_distance_ > 0.0) {
      const auto match_violation = xt::maximum(
        position_error - static_cast<float>(max_match_distance_), 0.0f);
      step_cost += distance_penalty_weight_ * match_violation;
    }

    accumulated_cost += step_cost;
    ++sample_count;
  }

  if (sample_count == 0) {
    return;
  }

  if (debug_runtime) {
    const auto added_cost = xt::eval(
      xt::pow(cost_weight_ * (accumulated_cost / sample_count), power_));
    debugCheckAddedCosts(added_cost);
    data.costs += added_cost;
  } else {
    data.costs += xt::pow(cost_weight_ * (accumulated_cost / sample_count), power_);
  }
}

}  // namespace mppi_ego::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi_ego::critics::EgoTrajectoryCritic,
  mppi_ego::critics::CriticFunction)
