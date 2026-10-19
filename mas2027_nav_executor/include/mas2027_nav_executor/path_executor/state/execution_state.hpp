#pragma once

#include <cstdint>
#include <limits>
#include <optional>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/mpc_position_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"

namespace mas2027_nav_executor {

enum class ExecutorStatus : uint8_t
{
  PUBLISHED,
  WAITING_INPUT,
  TRAJECTORY_STALE,
  REFERENCE_FAILED,
  SOLVER_FAILED,
  TERRAIN_BLOCKED,
  DYNAMIC_BLOCKED,
};

struct ExecutorInput
{
  interfaces::msg::MpcPositionCommand::ConstSharedPtr trajectory;
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  double spin_speed{};
  double region_speed_limit{std::numeric_limits<double>::infinity()};
  double region_acceleration_limit{std::numeric_limits<double>::infinity()};
  bool allow_motion{false};
  bool follow_gimbal{false};
  bool reverse_follow_heading{false};  // Face opposite the tangent, keep path velocity unchanged.
  bool hold_translation{false};
  std::optional<double> alignment_heading_odom;
  rclcpp::Time stamp;
};

struct ExecutorOutput
{
  geometry_msgs::msg::Twist command;
  ExecutorStatus status{ExecutorStatus::WAITING_INPUT};
  // Secondary-reference diagnostics for field logs. These do not alter the
  // published Twist and are intentionally kept separate from the command.
  bool secondary_reference_applied{false};
  bool secondary_reference_solve_ok{true};
  bool secondary_reference_narrow{false};
  double secondary_reference_max_angle_deg{0.0};
  double secondary_reference_min_time_scale{1.0};
  double secondary_reference_solve_ms{0.0};
  // Goal-approach diagnostics: whether the measured-braking speed envelope
  // actually capped the MPC reference this tick, and how much room was left
  // to the trajectory end.
  bool goal_approach_capped{false};
  double goal_approach_remaining_m{std::numeric_limits<double>::quiet_NaN()};
  double goal_approach_allowed_speed{std::numeric_limits<double>::quiet_NaN()};
  bool clearance_braking_capped{false};
  double clearance_braking_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double clearance_braking_allowed_speed{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace mas2027_nav_executor
