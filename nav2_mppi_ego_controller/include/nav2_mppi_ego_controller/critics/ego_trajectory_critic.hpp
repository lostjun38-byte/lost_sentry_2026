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

#ifndef NAV2_MPPI_EGO_CONTROLLER__CRITICS__EGO_TRAJECTORY_CRITIC_HPP_
#define NAV2_MPPI_EGO_CONTROLLER__CRITICS__EGO_TRAJECTORY_CRITIC_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "ego_planner_msgs/msg/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav2_mppi_ego_controller/critic_function.hpp"
#include "nav2_mppi_ego_controller/tools/utils.hpp"

namespace mppi_ego::critics
{

/**
 * @class mppi_ego::critics::EgoTrajectoryCritic
 * @brief Scores MPPI samples against a time-parameterized Ego-Planner reference.
 *
 * This critic uses the smooth B-spline-derived position, yaw / velocity direction,
 * speed, curvature, and optional acceleration references to bias MPPI samples. It
 * does not directly execute the B-spline; safety and feasibility critics still own
 * the final control trade-off.
 */
class EgoTrajectoryCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  struct EgoTrajPoint
  {
    float t{0.0f};
    float x{0.0f};
    float y{0.0f};
    float yaw{0.0f};
    float speed{0.0f};
    float curvature{0.0f};
    float acceleration{0.0f};
  };

  void trajectoryCallback(const ego_planner_msgs::msg::Trajectory::SharedPtr msg);
  std::vector<EgoTrajPoint> buildTrajectory(
    const ego_planner_msgs::msg::Trajectory & trajectory_msg);
  EgoTrajPoint sampleTrajectoryByTime(
    const std::vector<EgoTrajPoint> & trajectory,
    float query_time) const;
  size_t findClosestReferenceIndex(
    const std::vector<EgoTrajPoint> & trajectory,
    float x, float y) const;
  void fillMissingYaws(
    std::vector<EgoTrajPoint> & trajectory,
    const std::vector<bool> & yaw_valid) const;
  void fillMissingCurvatures(
    std::vector<EgoTrajPoint> & trajectory,
    const std::vector<bool> & curvature_valid) const;
  void fillMissingAccelerations(
    std::vector<EgoTrajPoint> & trajectory,
    const std::vector<bool> & acceleration_valid) const;
  float estimateYawAt(
    const std::vector<EgoTrajPoint> & trajectory,
    size_t idx) const;
  float clampSymmetric(float value, double limit) const;
  void debugCheckReferenceTrajectory(
    const std::vector<EgoTrajPoint> & trajectory,
    const std::string & frame_id,
    double trajectory_dt) const;
  void debugCheckScoreRuntime(
    const CriticData & data,
    const std::vector<EgoTrajPoint> & trajectory,
    size_t start_idx,
    double trajectory_dt,
    size_t time_steps) const;
  void debugCheckAddedCosts(
    const xt::xtensor<float, 1> & added_costs) const;
  bool debugShouldRun(double & last_run_time, double current_time) const;
  int64_t debugThrottleMs() const;

  rclcpp::Subscription<ego_planner_msgs::msg::Trajectory>::SharedPtr ego_trajectory_sub_;

  std::mutex trajectory_mutex_;
  std::vector<EgoTrajPoint> ego_trajectory_;
  double ego_trajectory_dt_{0.1};
  rclcpp::Time last_trajectory_update_;
  bool stale_warning_active_{false};

  std::string ego_trajectory_topic_;
  double max_reference_age_{0.0};
  double lookahead_time_{0.0};
  double max_match_distance_{0.0};
  double reference_dt_{0.0};
  double max_reference_speed_{0.0};
  size_t trajectory_point_step_{1};

  unsigned int power_{1};
  float cost_weight_{1.0f};
  float position_weight_{1.0f};
  float yaw_weight_{0.0f};
  float velocity_weight_{0.0f};
  float velocity_direction_weight_{0.0f};
  float velocity_direction_min_speed_{0.05f};
  float distance_penalty_weight_{10.0f};
  bool use_curvature_cost_{false};
  float curvature_weight_{0.0f};
  double max_reference_curvature_{5.0};
  double max_candidate_curvature_{8.0};
  bool use_acceleration_cost_{false};
  float acceleration_weight_{0.0f};
  double max_reference_acceleration_{5.0};
  double max_candidate_acceleration_{8.0};
  std::string velocity_frame_{"base"};
  bool velocity_frame_is_global_{false};

  bool debug_enabled_{false};
  double debug_log_period_{1.0};
  size_t debug_sample_candidates_{3};
  bool debug_check_trajectory_on_callback_{true};
  bool debug_check_score_runtime_{true};
  double debug_max_point_gap_{1.0};
  double debug_min_point_gap_{0.001};
  double debug_max_yaw_jump_{1.57};
  double debug_max_speed_jump_{2.0};
  double debug_max_curvature_{8.0};
  double debug_max_acceleration_{10.0};
  double debug_max_cost_{1.0e6};
  double debug_large_error_distance_{2.0};
  mutable double last_reference_debug_time_{-1.0};
  mutable double last_score_debug_time_{-1.0};
};

}  // namespace mppi_ego::critics

#endif  // NAV2_MPPI_EGO_CONTROLLER__CRITICS__EGO_TRAJECTORY_CRITIC_HPP_
