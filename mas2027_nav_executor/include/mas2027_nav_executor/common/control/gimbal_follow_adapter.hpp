#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace mas2027_nav_executor {
struct GimbalFollowParams
{
  double ready_tolerance_deg{5.0};
  double ready_release_deg{8.0};
  double settle_time_s{0.3};
  bool allow_reverse{true};
  double reverse_switch_hysteresis_deg{10.0};
  // LIO heading trim on top of the MPC yaw-rate feed-forward. This keeps the
  // base_link +X/-X axis on the trajectory even when translation is blocked.
  double heading_error_gain_s{1.0};
  double heading_deadband_deg{0.5};
  double max_heading_rate_deg_s{120.0};
};
struct GimbalFollowOutput
{
  bool follow_gimbal{};
  bool heading_ready{};  // LIO heading only, not a chassis acknowledgement.
  double heading_error_deg{};  // Signed error to the nearer path axis; sign is only turn side.
  bool reverse_heading{};
};

// Protocol adapter and heading gate. The MPC supplies the yaw-rate feed-forward;
// the measured LIO axis error supplies a trim so base_link stays on the path even
// while translation is rejected. Only the integrated position command is sent.
class GimbalFollowAdapter final
{
public:
  explicit GimbalFollowAdapter(GimbalFollowParams params) : params_(params)
  {
    if (!std::isfinite(params.ready_tolerance_deg) || params.ready_tolerance_deg <= 0.0 ||
      !std::isfinite(params.ready_release_deg) || params.ready_release_deg <= params.ready_tolerance_deg ||
      params.ready_release_deg > 180.0 || !std::isfinite(params.settle_time_s) || params.settle_time_s <= 0.0 ||
      !std::isfinite(params.reverse_switch_hysteresis_deg) ||
      params.reverse_switch_hysteresis_deg < 0.0 || params.reverse_switch_hysteresis_deg >= 90.0 ||
      !std::isfinite(params.heading_error_gain_s) || params.heading_error_gain_s < 0.0 ||
      !std::isfinite(params.heading_deadband_deg) || params.heading_deadband_deg < 0.0 ||
      !std::isfinite(params.max_heading_rate_deg_s) || params.max_heading_rate_deg_s <= 0.0) {
      throw std::invalid_argument("Invalid gimbal_follow heading tolerances, gains or settle time");
    }
  }
  GimbalFollowOutput updateHeading(std::optional<double> heading_error_deg, double dt)
  {
    if (!heading_error_deg || !std::isfinite(*heading_error_deg) || !std::isfinite(dt) || dt <= 0.0) {
      reverse_heading_ = false;
      following_ = false;  // No heading: never integrate, even when the test flag stays set.
      holding_ = false;
      settled_s_ = 0.0;
      heading_ready_ = false;
      heading_error_deg_ = 0.0;
      command_deg_ = 0.0;
      has_command_ = false;
      return {false, false, 0.0};
    }
    const double forward_error = std::remainder(*heading_error_deg, 360.0);
    bool reverse = reverse_heading_;
    if (!params_.allow_reverse) {
      reverse = false;
    } else if (!following_) {
      reverse = std::abs(forward_error) > 90.0 + 1e-6;
    } else {
      const double band = params_.reverse_switch_hysteresis_deg;
      if (!reverse && std::abs(forward_error) > 90.0 + band) reverse = true;
      if (reverse && std::abs(forward_error) < 90.0 - band) reverse = false;
    }
    reverse_heading_ = reverse;
    following_ = true;
    holding_ = false;
    has_command_ = true;
    const double error = std::remainder(forward_error + (reverse ? 180.0 : 0.0), 360.0);
    heading_error_deg_ = error;
    if (heading_ready_ && std::abs(error) > params_.ready_release_deg) {
      heading_ready_ = false;
      settled_s_ = 0.0;
    }
    if (!heading_ready_) {
      settled_s_ = std::abs(error) <= params_.ready_tolerance_deg ? settled_s_ + dt : 0.0;
      heading_ready_ = settled_s_ >= params_.settle_time_s;
    }
    return {true, heading_ready_, error, reverse_heading_};
  }
  /// Freeze the last integrated position command while the planner is between
  /// trajectories. The lower board owns the angle loop, so dropping follow and
  /// sending ref=0 here makes it resume its free spin and loses the alignment
  /// baseline. The held output stays in follow mode with zero translation at
  /// the node. Keep the measured readiness and direction hysteresis so a
  /// short planner gap does not restart settling when the same path resumes.
  /// No new angle is integrated until updateHeading() supplies a path.
  GimbalFollowOutput hold()
  {
    holding_ = true;
    return {true, false, 0.0, reverse_heading_};
  }
  bool hasCommand() const { return has_command_; }
  double commandDegrees() const { return command_deg_; }
  double integrateMpcYawRate(double omega_rad_s, double dt)
  {
    if (following_ && !holding_ && std::isfinite(omega_rad_s) && std::isfinite(dt) && dt > 0.0) {
      constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
      double rate_deg_s = omega_rad_s * kRadiansToDegrees;
      if (std::abs(heading_error_deg_) > params_.heading_deadband_deg) {
        rate_deg_s += params_.heading_error_gain_s * heading_error_deg_;
      }
      rate_deg_s = std::clamp(
        rate_deg_s, -params_.max_heading_rate_deg_s, params_.max_heading_rate_deg_s);
      // This is a position offset: wrapping would jump the lower INS target by 360 degrees.
      command_deg_ = std::clamp(command_deg_ + rate_deg_s * dt, -180.0, 180.0);
    }
    return command_deg_;
  }
private:
  GimbalFollowParams params_;
  double command_deg_{}, settled_s_{}, heading_error_deg_{};
  bool heading_ready_{}, following_{}, holding_{}, reverse_heading_{}, has_command_{};
};
}  // namespace mas2027_nav_executor
