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
#include <cmath>
#include <limits>
#include <utility>

#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mppi_ego::critics
{

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

  reference_dt_ = std::max(reference_dt_, 1e-3);
  max_reference_age_ = std::max(max_reference_age_, 0.0);
  lookahead_time_ = std::max(lookahead_time_, 0.0);
  max_match_distance_ = std::max(max_match_distance_, 0.0);
  max_reference_speed_ = std::max(max_reference_speed_, 0.0);
  velocity_direction_min_speed_ = std::max(velocity_direction_min_speed_, 0.0f);
  trajectory_point_step_ = std::max<size_t>(trajectory_point_step_, 1);

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
  const std::string target_frame = costmap_ros_->getGlobalFrameID();
  const bool should_transform =
    !trajectory_msg.header.frame_id.empty() && trajectory_msg.header.frame_id != target_frame;

  for (const auto & trajectory_point : trajectory_msg.points) {
    geometry_msgs::msg::PoseStamped reference_pose;
    reference_pose.header = trajectory_msg.header;
    reference_pose.pose.position.x = trajectory_point.x;
    reference_pose.pose.position.y = trajectory_point.y;
    reference_pose.pose.position.z = trajectory_point.z;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, trajectory_point.yaw);
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
    point.x = static_cast<float>(transformed_pose.pose.position.x);
    point.y = static_cast<float>(transformed_pose.pose.position.y);
    point.yaw = static_cast<float>(tf2::getYaw(transformed_pose.pose.orientation));
    point.speed = std::min(
      std::max(trajectory_point.velocity, 0.0f),
      static_cast<float>(max_reference_speed_));

    trajectory.push_back(point);
  }

  return trajectory;
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
  const auto reference_age = node->now() - last_update;
  if (max_reference_age_ > 0.0 &&
    reference_age > rclcpp::Duration::from_seconds(max_reference_age_))
  {
    if (!stale_warning_active_) {
      RCLCPP_WARN(
        logger_,
        "Ego trajectory age %.3fs exceeds max_reference_age %.3fs; skipping EgoTrajectoryCritic.",
        reference_age.seconds(), max_reference_age_);
      stale_warning_active_ = true;
    }
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

  if (lookahead_time_ > 0.0) {
    const auto lookahead_steps = static_cast<size_t>(
      std::ceil(lookahead_time_ / static_cast<double>(data.model_dt)));
    time_steps = std::min(time_steps, std::max<size_t>(lookahead_steps, 1));
  }

  const auto robot_x = static_cast<float>(data.state.pose.pose.position.x);
  const auto robot_y = static_cast<float>(data.state.pose.pose.position.y);
  const size_t start_idx = findClosestReferenceIndex(trajectory, robot_x, robot_y);
  const float ref_step_ratio = static_cast<float>(data.model_dt / trajectory_dt);

  xt::xtensor<float, 1> accumulated_cost = xt::zeros<float>({data.costs.shape(0)});
  size_t sample_count = 0;

  for (size_t t = 0; t < time_steps; t += trajectory_point_step_) {
    const auto ref_offset = static_cast<size_t>(std::round(static_cast<float>(t) * ref_step_ratio));
    const auto ref_idx = std::min(start_idx + ref_offset, trajectory.size() - 1);
    const auto & ref = trajectory[ref_idx];

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

    auto velocity_x = traj_vx * xt::cos(traj_yaw) - traj_vy * xt::sin(traj_yaw);
    auto velocity_y = traj_vx * xt::sin(traj_yaw) + traj_vy * xt::cos(traj_yaw);
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

  data.costs += xt::pow(cost_weight_ * (accumulated_cost / sample_count), power_);
}

}  // namespace mppi_ego::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi_ego::critics::EgoTrajectoryCritic,
  mppi_ego::critics::CriticFunction)
