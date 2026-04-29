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
 * @brief Scores MPPI samples against an Ego-Planner reference trajectory.
 */
class EgoTrajectoryCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  struct EgoTrajPoint
  {
    float x{0.0f};
    float y{0.0f};
    float yaw{0.0f};
    float speed{0.0f};
  };

  void trajectoryCallback(const ego_planner_msgs::msg::Trajectory::SharedPtr msg);
  std::vector<EgoTrajPoint> buildTrajectory(
    const ego_planner_msgs::msg::Trajectory & trajectory_msg);
  size_t findClosestReferenceIndex(
    const std::vector<EgoTrajPoint> & trajectory,
    float x, float y) const;

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
};

}  // namespace mppi_ego::critics

#endif  // NAV2_MPPI_EGO_CONTROLLER__CRITICS__EGO_TRAJECTORY_CRITIC_HPP_
