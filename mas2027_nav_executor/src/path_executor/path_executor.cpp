#include "mas2027_nav_executor/path_executor/path_executor.hpp"

#include "mas2027_nav_executor/common/environment/clearance_gate.hpp"
#include "mas2027_nav_executor/common/environment/narrow_passage_geometry.hpp"
#include "mas2027_nav_executor/path_executor/monitoring/command_safety.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mas2027_nav_executor {
using minco_controller::Control;
using minco_controller::MPCConfig;
using minco_controller::MpcSolver;
using minco_controller::ReferencePoint;
using minco_controller::State;

namespace {

double normalize_yaw(const double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

template<typename T>
bool transform_vector(
  const T & input,
  T & output,
  const geometry_msgs::msg::TransformStamped & transform)
{
  try {
    tf2::doTransform(input, output, transform);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

}  // namespace

PathExecutor::PathExecutor(
  const MPCConfig & config,
  const PathExecutorParams & params,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<TerrainGrid> terrain,
  std::shared_ptr<rog_map::MapQueryInterface> rog_query)
: config_(config),
  params_(params),
  tf_buffer_(std::move(tf_buffer)),
  terrain_(std::move(terrain)),
  rog_query_(std::move(rog_query)),
  solver_(std::make_unique<MpcSolver>(config)) {}

ExecutorOutput PathExecutor::computeCommand(const ExecutorInput & input)
{
  ExecutorOutput output;
  if (!input.allow_motion) {
    solver_->resetLastControl();
    output.status = ExecutorStatus::WAITING_INPUT;
    return output;
  }
  if (!input.trajectory || !input.odom || input.trajectory->cmds.empty()) {
    solver_->resetLastControl();
    output.status = ExecutorStatus::WAITING_INPUT;
    return output;
  }
  if (std::abs((input.stamp - rclcpp::Time(input.odom->header.stamp)).seconds()) >
    params_.odom_timeout_s) {
    solver_->resetLastControl();
    output.status = ExecutorStatus::WAITING_INPUT;
    return output;
  }

  const auto & q = input.odom->pose.pose.orientation;
  const double quaternion_norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(quaternion_norm_squared) || quaternion_norm_squared <= 1e-12) {
    solver_->resetLastControl();
    return output;
  }
  const double yaw = tf2::getYaw(input.odom->pose.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  State current;
  current.x = input.odom->pose.pose.position.x;
  current.y = input.odom->pose.pose.position.y;
  current.yaw = yaw;
  current.vx = c * input.odom->twist.twist.linear.x - s * input.odom->twist.twist.linear.y;
  current.vy = s * input.odom->twist.twist.linear.x + c * input.odom->twist.twist.linear.y;
  current.omega = input.odom->twist.twist.angular.z;

  if (!std::isfinite(current.x) || !std::isfinite(current.y) || !std::isfinite(current.yaw) ||
    !std::isfinite(current.vx) || !std::isfinite(current.vy) || !std::isfinite(current.omega)) {
    solver_->resetLastControl();
    return output;
  }

  // 被打断（轨迹过期/参考/求解/安全门否决）时不清零加速度锚点，而是锚到实测车速：清零会让恢复
  // 被 |u_0| <= a_max·dt 限死而急刹、再逐拍爬回；锚实测车速后 (v_cmd-v_actual)/dt 才是真加速度。
  const auto anchor_to_measured = [this, &current]() {
    solver_->setLastControl(Eigen::Vector3d(current.vx, current.vy, current.omega));
  };

  if ((input.stamp - rclcpp::Time(input.trajectory->header.stamp)).seconds() >
    params_.trajectory_timeout_s)
  {
    anchor_to_measured();
    output.status = ExecutorStatus::TRAJECTORY_STALE;
    return output;
  }

  std::vector<ReferencePoint> reference;
  std::vector<interfaces::msg::PositionCommand> transformed_commands;
  size_t nearest_command_index = 0U;
  if (!buildReference(*input.trajectory, current, reference,
    input.follow_gimbal, input.reverse_follow_heading,
    &nearest_command_index, &transformed_commands)) {
    anchor_to_measured();
    output.status = ExecutorStatus::REFERENCE_FAILED;
    return output;
  }
  if (input.follow_gimbal && input.hold_translation) {
    if (!input.alignment_heading_odom || !std::isfinite(*input.alignment_heading_odom)) {
      anchor_to_measured();
      output.status = ExecutorStatus::REFERENCE_FAILED;
      return output;
    }
    for (auto & point : reference) {
      point.pos = {current.x, current.y};
      point.vel.setZero();
      point.yaw = *input.alignment_heading_odom;
      point.yaw_rate = 0.0;
    }
  }
  // 目标接近减速带：规划侧已经用实测刹车能力封顶了种子，这里再按已提交
  // 轨迹的剩余弧长对 MPC 参考封一次顶，兜住"重规划来不及"的那几拍。
  output.goal_approach_capped = applyGoalApproachCap(
    reference, transformed_commands,
    &output.goal_approach_remaining_m, &output.goal_approach_allowed_speed);
  MPCConfig effective_config = config_;
  if (std::isfinite(input.region_speed_limit)) {
    const double limit = input.region_speed_limit;
    effective_config.vx_min = std::max(effective_config.vx_min, -limit);
    effective_config.vx_max = std::min(effective_config.vx_max, limit);
    effective_config.vy_min = std::max(effective_config.vy_min, -limit);
    effective_config.vy_max = std::min(effective_config.vy_max, limit);
  }
  if (std::isfinite(input.region_acceleration_limit)) {
    const double limit = input.region_acceleration_limit;
    effective_config.ax_min = std::max(effective_config.ax_min, -limit);
    effective_config.ax_max = std::min(effective_config.ax_max, limit);
    effective_config.ay_min = std::max(effective_config.ay_min, -limit);
    effective_config.ay_max = std::min(effective_config.ay_max, limit);
  }
  solver_->setConfig(effective_config);
  // Narrow-corridor continuation: find the first narrow sample on the committed
  // route. Keep the narrow floor active ahead of the entrance, but use the
  // measured braking envelope to arrive there at narrow_passage_speed instead
  // of driving the entire lookahead at that speed.
  // This remains local to the committed narrow route; open-space driving uses
  // the full threshold.
  const double full_clearance =
    effectiveClearanceThreshold(params_.rog_map_clearance);
  bool current_narrow = false;
  bool current_narrow_eligible = false;
  double current_narrow_clearance = 0.0;
  if (params_.narrow_passage_enable && rog_query_) {
    const auto current_clearance =
      rog_query_->query(Eigen::Vector3d(current.x, current.y, 0.0));
    current_narrow_eligible = current_clearance.ok &&
      std::isfinite(current_clearance.distance) &&
      current_clearance.distance >= params_.narrow_passage_clearance;
    current_narrow = current_narrow_eligible &&
      current_clearance.distance < full_clearance;
    if (current_narrow) current_narrow_clearance = current_clearance.distance;
  }
  bool narrow_ahead = false;
  double narrow_entrance_distance = std::numeric_limits<double>::infinity();
  double nearest_command_offset = 0.0;
  if (params_.narrow_passage_enable && rog_query_ &&
      nearest_command_index < transformed_commands.size()) {
    constexpr double kNarrowPassageLookahead = 1.5;
    double ahead_distance = 0.0;
    bool safe_narrow_ahead = false;
    Eigen::Vector2d previous(current.x, current.y);
    const auto & nearest = transformed_commands[nearest_command_index].position;
    nearest_command_offset = (Eigen::Vector2d(nearest.x, nearest.y) - previous).norm();
    for (size_t index = nearest_command_index; index < transformed_commands.size(); ++index) {
      const auto & command = transformed_commands[index];
      const Eigen::Vector2d point(command.position.x, command.position.y);
      const double previous_distance = ahead_distance;
      ahead_distance += (point - previous).norm();
      previous = point;
      if (ahead_distance > kNarrowPassageLookahead) {
        break;
      }
      const auto clearance = rog_query_->query(Eigen::Vector3d(point.x(), point.y(), 0.0));
      if (!clearance.ok || !std::isfinite(clearance.distance)) {
        break;
      }
      if (clearance.distance >= params_.narrow_passage_clearance &&
          clearance.distance < full_clearance) {
        safe_narrow_ahead = true;
        // The threshold may be crossed anywhere between the previous sample
        // and this one. Use the preceding distance for conservative braking.
        narrow_entrance_distance = std::min(narrow_entrance_distance, previous_distance);
      }
    }
    narrow_ahead = safe_narrow_ahead;
  }
  const bool narrow_continuation = current_narrow || narrow_ahead;
  // Match the command gate's near-field base. In a narrow cell its dynamic
  // requirement starts at max(floor, current clearance - 0.02); capping against
  // the floor alone can still produce a command that the gate rejects.
  const double command_base_clearance = current_narrow ?
    std::max(params_.narrow_passage_clearance,
      current_narrow_clearance - kEsdfJitterTolerance) :
    (narrow_ahead ? params_.narrow_passage_clearance : full_clearance);
  applyClearanceSpeedCap(reference, command_base_clearance);
  // Keep the previous published control as the acceleration anchor for both
  // solves. The provisional QP must not change the second QP's constraints.
  const Eigen::Vector3d previous_control = solver_->getLastControl();
  Control control;
  std::vector<State> predicted;
  const bool secondary_enabled = params_.secondary_reference_enable &&
    (params_.secondary_reference_angle_enable || params_.secondary_reference_narrow_enable) &&
    !(input.follow_gimbal && input.hold_translation);
  if (!solver_->solve(current, reference, control, secondary_enabled ? &predicted : nullptr)) {
    anchor_to_measured();
    output.status = ExecutorStatus::SOLVER_FAILED;
    return output;
  }
  if (secondary_enabled && predicted.size() == reference.size() + 1U) {
    std::vector<double> time_scales(reference.size(), 1.0);
    bool resample = false;
    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    const double trigger_cos = std::cos(
      params_.secondary_reference_angle_deg * 3.14159265358979323846 / 180.0);
    // min_time_scale is a hard floor. narrow_time_scale may cap the reference
    // only above it; this also keeps direct PathExecutor use safe when the
    // node-level range validation is bypassed.
    const double narrow_floor = std::max(
      params_.secondary_reference_min_time_scale,
      params_.secondary_reference_narrow_time_scale);
    double max_angle_deg = 0.0;
    for (size_t step = 0U; step < reference.size(); ++step) {
      const Eigen::Vector2d desired = reference[step].vel;
      const Eigen::Vector2d planned(
        (predicted[step + 1U].x - predicted[step].x) / config_.dt,
        (predicted[step + 1U].y - predicted[step].y) / config_.dt);
      if (desired.allFinite() && planned.allFinite() &&
        desired.norm() > 0.05 && planned.norm() > 0.05) {
        const double cosine = std::clamp(
          planned.dot(desired) / (planned.norm() * desired.norm()), -1.0, 1.0);
        max_angle_deg = std::max(max_angle_deg, std::acos(cosine) * kRadiansToDegrees);
        if (params_.secondary_reference_angle_enable && cosine < trigger_cos) {
          time_scales[step] = std::max(params_.secondary_reference_min_time_scale, cosine);
          resample = true;
        }
      }
      if (params_.secondary_reference_narrow_enable && narrow_continuation) {
        time_scales[step] = std::min(time_scales[step], narrow_floor);
        if (time_scales[step] < 1.0 - 1.0e-9) {
          resample = true;
        }
      }
    }
    output.secondary_reference_narrow =
      params_.secondary_reference_narrow_enable && narrow_continuation;
    output.secondary_reference_max_angle_deg = max_angle_deg;
    if (resample) {
      std::vector<ReferencePoint> slower_reference;
      if (sampleReference(transformed_commands, nearest_command_index, time_scales,
        slower_reference, input.follow_gimbal, input.reverse_follow_heading)) {
        // 二次参考是重新采样的，必须再过一次接近减速包络，否则它会顶掉主参考的封顶。
        applyGoalApproachCap(slower_reference, transformed_commands);
        applyClearanceSpeedCap(slower_reference, command_base_clearance);
        output.secondary_reference_applied = true;
        output.secondary_reference_min_time_scale =
          *std::min_element(time_scales.begin(), time_scales.end());
        solver_->setLastControl(previous_control);
        Control refined_control;
        const auto solve_start = std::chrono::steady_clock::now();
        const bool refined_ok = solver_->solve(current, slower_reference, refined_control);
        output.secondary_reference_solve_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - solve_start).count();
        if (refined_ok) {
          control = refined_control;
          output.secondary_reference_solve_ok = true;
        } else {
          // The first feasible QP remains available if resampling is infeasible.
          output.secondary_reference_solve_ok = false;
          solver_->setLastControl(Eigen::Vector3d(control.vx, control.vy, control.omega));
        }
      }
    }
  }
  if (std::isfinite(input.region_speed_limit)) {
    const double speed = std::hypot(control.vx, control.vy);
    if (speed > input.region_speed_limit) {
      const double scale = input.region_speed_limit / speed;
      control.vx *= scale;
      control.vy *= scale;
      solver_->setLastControl(Eigen::Vector3d(control.vx, control.vy, control.omega));
    }
  }
  if (narrow_continuation) {
    const double speed = std::hypot(control.vx, control.vy);
    double cap = params_.narrow_passage_speed;
    if (!current_narrow && current_narrow_eligible && narrow_ahead &&
        params_.clearance_braking.valid() &&
        std::isfinite(rog_query_->resolution()) && rog_query_->resolution() > 0.0) {
      const double decel = params_.clearance_braking.deceleration;
      const double delay = params_.clearance_braking.reaction_time;
      const double target = params_.narrow_passage_speed;
      const double distance = std::max(0.0,
        narrow_entrance_distance - nearest_command_offset - rog_query_->resolution());
      // distance >= (v - target) * delay + (v^2 - target^2)/(2 * decel)
      cap = std::sqrt((target + decel * delay) * (target + decel * delay) +
        2.0 * decel * distance) -
        decel * delay;
    }
    cap = std::min(cap, input.region_speed_limit);
    if (std::isfinite(cap) && cap > 0.0 && speed > cap) {
      const double scale = cap / speed;
      control.vx *= scale;
      control.vy *= scale;
      solver_->setLastControl(Eigen::Vector3d(control.vx, control.vy, control.omega));
    }
  }

  // Bound the actual instruction as well as the MPC reference. A low ESDF
  // value at the end of the *unscaled* command horizon must not force the
  // current command to zero when a slower command would stop short of it.
  // Check the horizon again at each candidate speed; the command gate below
  // still performs its independent final safety check.
  if (params_.clearance_braking.valid() && rog_query_) {
    const double speed = std::hypot(control.vx, control.vy);
    if (speed > 1e-6) {
      const double horizon = std::max(config_.dt, 0.35);
      const int steps = std::max(1, static_cast<int>(std::ceil(horizon / config_.dt)));
      double limiting_distance = std::numeric_limits<double>::quiet_NaN();
      const auto braking_safe = [&](double vx, double vy, double * limiting,
          Eigen::Vector2d * gradient, double * allowed_toward) {
        for (int i = 0; i <= steps; ++i) {
          const double t = horizon * static_cast<double>(i) / steps;
          const auto clearance = rog_query_->query(Eigen::Vector3d(
            current.x + t * vx, current.y + t * vy, 0.0));
          if (!clearance.ok || !std::isfinite(clearance.distance)) continue;
          const double toward = obstacleApproachSpeed(
            vx, vy,
            clearance.gradient.x(), clearance.gradient.y());
          if (toward <= 1e-6) continue;
          const double allowed = clearanceSpeedLimit(
            std::max(0.0, clearance.distance - command_base_clearance -
              kClearanceComparisonTolerance), params_.clearance_braking);
          if (toward > allowed) {
            if (limiting) *limiting = clearance.distance;
            if (gradient) *gradient = clearance.gradient.head<2>();
            if (allowed_toward) *allowed_toward = allowed;
            return false;
          }
        }
        return true;
      };
      Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
      double allowed_toward = 0.0;
      if (!braking_safe(control.vx, control.vy, &limiting_distance,
          &gradient, &allowed_toward)) {
        double candidate_vx = control.vx;
        double candidate_vy = control.vy;
        bool projected = false;
        for (int iteration = 0; iteration < 8; ++iteration) {
          if (!limitObstacleApproachVelocity(candidate_vx, candidate_vy,
              gradient.x(), gradient.y(), allowed_toward)) {
            break;
          }
          projected = true;
          if (braking_safe(candidate_vx, candidate_vy, nullptr,
              &gradient, &allowed_toward)) {
            break;
          }
        }
        // Conflicting walls or a discontinuous field may leave no projected
        // command that passes the whole horizon. Fall back to the existing
        // scalar search; the final command gate still checks every output.
        if (!projected || !braking_safe(candidate_vx, candidate_vy, nullptr,
            nullptr, nullptr)) {
          candidate_vx = control.vx;
          candidate_vy = control.vy;
          double safe_scale = 0.0;
          double unsafe_scale = 1.0;
          for (int iteration = 0; iteration < 10; ++iteration) {
            const double middle = 0.5 * (safe_scale + unsafe_scale);
            if (braking_safe(middle * control.vx, middle * control.vy,
                nullptr, nullptr, nullptr)) {
              safe_scale = middle;
            } else {
              unsafe_scale = middle;
            }
          }
          candidate_vx *= safe_scale;
          candidate_vy *= safe_scale;
        }
        control.vx = candidate_vx;
        control.vy = candidate_vy;
        output.clearance_braking_capped = true;
        output.clearance_braking_distance_m = limiting_distance;
        output.clearance_braking_allowed_speed = std::hypot(control.vx, control.vy);
        solver_->setLastControl(Eigen::Vector3d(control.vx, control.vy, control.omega));
      }
    }
  }

  if (input.follow_gimbal && input.hold_translation) {
    control.vx = 0.0;
    control.vy = 0.0;
    solver_->setLastControl(Eigen::Vector3d(0.0, 0.0, control.omega));
  }
  CommandSafetyDetail safety_detail;
  output.status = checkCommandSafety(
    terrain_, rog_query_, tf_buffer_, params_.odom_frame, params_.rog_map_clearance,
    params_.narrow_passage_enable, params_.narrow_passage_clearance,
    narrow_ahead, params_.rog_map_timeout_s, config_.dt, current, control,
    input.stamp, &safety_detail, params_.clearance_braking);
  last_safety_detail_ = safety_detail;
  if (output.status != ExecutorStatus::PUBLISHED) {
    anchor_to_measured();
    return output;
  }

  if (std::hypot(control.vx, control.vy) >= params_.deadzone_speed) {
    if (params_.output_in_body_frame) {
      output.command.linear.x = c * control.vx + s * control.vy;
      output.command.linear.y = -s * control.vx + c * control.vy;
    } else {
      output.command.linear.x = control.vx;
      output.command.linear.y = control.vy;
    }
  }
  output.command.angular.z = std::clamp(
    control.omega + (input.follow_gimbal ? 0.0 : input.spin_speed), config_.omega_min, config_.omega_max);
  output.status = ExecutorStatus::PUBLISHED;
  return output;
}

std::optional<double> PathExecutor::narrowPassageHeadingErrorDegrees(
  const interfaces::msg::MpcPositionCommand & trajectory,
  double current_x, double current_y, double current_yaw)
{
  if (!params_.narrow_passage_alignment_enable || !rog_query_ ||
    !std::isfinite(current_x) || !std::isfinite(current_y) || !std::isfinite(current_yaw) ||
    !std::isfinite(params_.narrow_passage_alignment_lookahead) ||
    params_.narrow_passage_alignment_lookahead <= 0.0 ||
    !std::isfinite(params_.narrow_passage_alignment_hold_distance) ||
    params_.narrow_passage_alignment_hold_distance < 0.0 ||
    !std::isfinite(params_.narrow_passage_clearance) ||
    params_.narrow_passage_clearance <= 0.0) {
    resetNarrowPassageAlignment();
    return std::nullopt;
  }
  updateNarrowPassageAlignmentPose(current_x, current_y);
  std::vector<interfaces::msg::PositionCommand> commands;
  if (!transformedCommands(trajectory, commands) || commands.empty()) {
    if (narrow_alignment_active_) {
      const double target_yaw = std::atan2(last_narrow_tangent_.y(), last_narrow_tangent_.x());
      constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
      return std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw)) *
        kRadiansToDegrees;
    }
    return std::nullopt;
  }

  size_t nearest = 0U;
  double nearest_squared = std::numeric_limits<double>::infinity();
  for (size_t i = 0U; i < commands.size(); ++i) {
    const double dx = commands[i].position.x - current_x;
    const double dy = commands[i].position.y - current_y;
    const double squared = dx * dx + dy * dy;
    if (squared < nearest_squared) {
      nearest_squared = squared;
      nearest = i;
    }
  }

  const double full_clearance = effectiveClearanceThreshold(params_.rog_map_clearance);
  const double alignment_clearance = alignedBodyClearanceRequirement();
  // Do not align to the robot's current tight cell alone. If the route opens
  // immediately after a wall-hugging start, rotating the chassis toward that
  // old tangent causes unnecessary oscillation. Require a low-clearance point
  // slightly ahead; inside a real corridor the next samples still qualify.
  constexpr double kMinimumAlignmentAhead = 0.15;
  // The longer lookahead keeps an active alignment aware of an upcoming
  // section, but a distant point must not rotate the chassis while the robot
  // is still traversing open space.
  constexpr double kNearNarrowDistance = 0.50;
  constexpr double kExitNarrowDistance = 0.75;
  constexpr double kExitClearanceHysteresis = 0.03;
  double traveled = 0.0;
  size_t first_narrow = commands.size();
  double nearest_narrow_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector2d first_narrow_tangent = Eigen::Vector2d::Zero();
  const auto tangent_at = [&](size_t index) {
    // A single MINCO segment can change direction sharply when the local
    // route is refreshed. Use a short chord around the narrow sample so
    // small waypoint shifts do not repeatedly cancel heading readiness.
    constexpr double kTangentHalfWindow = 0.15;
    size_t before = index;
    size_t after = index;
    double before_length = 0.0;
    double after_length = 0.0;
    while (before > 0U && before_length < kTangentHalfWindow) {
      before_length += std::hypot(
        commands[before].position.x - commands[before - 1U].position.x,
        commands[before].position.y - commands[before - 1U].position.y);
      --before;
    }
    while (after + 1U < commands.size() && after_length < kTangentHalfWindow) {
      after_length += std::hypot(
        commands[after + 1U].position.x - commands[after].position.x,
        commands[after + 1U].position.y - commands[after].position.y);
      ++after;
    }
    const Eigen::Vector2d smoothed(
      commands[after].position.x - commands[before].position.x,
      commands[after].position.y - commands[before].position.y);
    if (smoothed.squaredNorm() > 1.0e-12) return smoothed.normalized().eval();
    for (size_t i = index; i > 0U; --i) {
      const Eigen::Vector2d delta(
        commands[i].position.x - commands[i - 1U].position.x,
        commands[i].position.y - commands[i - 1U].position.y);
      if (delta.squaredNorm() > 1.0e-12) return delta.normalized().eval();
    }
    for (size_t i = index + 1U; i < commands.size(); ++i) {
      const Eigen::Vector2d delta(
        commands[i].position.x - commands[i - 1U].position.x,
        commands[i].position.y - commands[i - 1U].position.y);
      if (delta.squaredNorm() > 1.0e-12) return delta.normalized().eval();
    }
    return Eigen::Vector2d::Zero().eval();
  };
  for (size_t i = nearest; i < commands.size(); ++i) {
    if (i > nearest) {
      traveled += std::hypot(
        commands[i].position.x - commands[i - 1U].position.x,
        commands[i].position.y - commands[i - 1U].position.y);
    }
    if (traveled > params_.narrow_passage_alignment_lookahead) {
      break;
    }
    const auto clearance = rog_query_->query(Eigen::Vector3d(
      commands[i].position.x, commands[i].position.y, 0.0));
    if (!clearance.ok || !std::isfinite(clearance.distance)) {
      break;
    }
    if (clearance.distance >= params_.narrow_passage_clearance &&
      clearance.distance < alignment_clearance) {
      const Eigen::Vector2d tangent = tangent_at(i);
      if (tangent.squaredNorm() <= 1.0e-12 || !hasOpposingSideObstacles(
          *rog_query_, Eigen::Vector2d(commands[i].position.x, commands[i].position.y),
          tangent, clearance.distance)) {
        continue;
      }
      nearest_narrow_distance = std::min(nearest_narrow_distance, traveled);
      if (first_narrow == commands.size() &&
        traveled >= kMinimumAlignmentAhead && traveled <= kNearNarrowDistance) {
        first_narrow = i;
        first_narrow_tangent = tangent;
      }
    }
  }

  const auto current_clearance = rog_query_->query(Eigen::Vector3d(current_x, current_y, 0.0));
  if (narrow_alignment_active_ &&
    narrow_alignment_traveled_m_ >= params_.narrow_passage_alignment_hold_distance &&
    current_clearance.ok && std::isfinite(current_clearance.distance) &&
    current_clearance.distance >=
      std::max(full_clearance + kExitClearanceHysteresis, rotationClearanceRequirement()) &&
    nearest_narrow_distance > kExitNarrowDistance) {
    releaseNarrowPassageAlignment();
    return std::nullopt;
  }
  // After leaving one corridor, a narrow point still far ahead on the same
  // route must not reactivate alignment on the very next control tick.
  if (!narrow_alignment_active_ && narrow_alignment_rearm_near_only_) {
    if (!std::isfinite(nearest_narrow_distance)) {
      narrow_alignment_rearm_near_only_ = false;
    } else if (nearest_narrow_distance > kNearNarrowDistance) {
      return std::nullopt;
    }
  }

  if (first_narrow_tangent.squaredNorm() > 1.0e-12 && first_narrow < commands.size()) {
    if (!narrow_alignment_active_) {
      last_alignment_pose_ = {current_x, current_y};
      narrow_alignment_traveled_m_ = 0.0;
      // A square body must keep one heading through this constrained section.
      // Replanning can change the first narrow waypoint and its tangent even
      // while the walls and chassis have not moved. Latch the entry tangent
      // until the corridor has genuinely ended and free spin is safe again.
      last_narrow_tangent_ = first_narrow_tangent;
    } else if (last_narrow_tangent_.dot(first_narrow_tangent) < -0.5) {
      // Returning through the same slot reverses travel direction but keeps
      // the same body axis. Flip only the tangent sign so the adapter can
      // select reverse travel without turning the square in place.
      last_narrow_tangent_ = -last_narrow_tangent_;
    }
    narrow_alignment_active_ = true;
    narrow_alignment_rearm_near_only_ = false;
  }
  if (!narrow_alignment_active_) {
    return std::nullopt;
  }
  const double target_yaw = std::atan2(last_narrow_tangent_.y(), last_narrow_tangent_.x());
  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  return std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw)) *
    kRadiansToDegrees;
}

void PathExecutor::resetNarrowPassageAlignment()
{
  if (narrow_alignment_active_) last_alignment_release_m_ = narrow_alignment_traveled_m_;
  narrow_alignment_active_ = false;
  last_narrow_tangent_.setZero();
  last_alignment_pose_.setZero();
  narrow_alignment_traveled_m_ = 0.0;
  narrow_alignment_rearm_near_only_ = false;
}

void PathExecutor::releaseNarrowPassageAlignment()
{
  resetNarrowPassageAlignment();
  narrow_alignment_rearm_near_only_ = true;
}

bool PathExecutor::releaseNarrowPassageAlignmentAtGoal()
{
  if (!narrow_alignment_active_ ||
    narrow_alignment_traveled_m_ < params_.narrow_passage_alignment_hold_distance) {
    return false;
  }
  // Releasing follow_gimbal hands yaw back to the lower board's free spin.
  // Distance traveled says nothing about room for the corners to rotate.
  const auto clearance = rog_query_ ? rog_query_->query(Eigen::Vector3d(
    last_alignment_pose_.x(), last_alignment_pose_.y(), 0.0)) : rog_map::QueryResult{};
  if (!clearance.ok || !std::isfinite(clearance.distance) ||
    clearance.distance < rotationClearanceRequirement()) {
    return false;
  }
  releaseNarrowPassageAlignment();
  return true;
}

double PathExecutor::rotationClearanceRequirement() const
{
  // EDT measures occupied cell centers; include the half diagonal of a cell.
  const double resolution = rog_query_ ? rog_query_->resolution() : 0.0;
  return params_.swept_radius + params_.rotation_margin +
    (std::isfinite(resolution) && resolution > 0.0 ?
      resolution / std::sqrt(2.0) : 0.0);
}

double PathExecutor::alignedBodyClearanceRequirement() const
{
  // For a square body, corner radius / sqrt(2) is the half width while it
  // travels aligned with the corridor. EDT measures occupied cell centers;
  // retain the map-cell uncertainty and the configured clearance margin.
  const double resolution = rog_query_ ? rog_query_->resolution() : 0.0;
  return params_.swept_radius / std::sqrt(2.0) + params_.rotation_margin +
    (std::isfinite(resolution) && resolution > 0.0 ?
      resolution / std::sqrt(2.0) : 0.0);
}

void PathExecutor::updateNarrowPassageAlignmentPose(double x, double y)
{
  if (!narrow_alignment_active_ || !std::isfinite(x) || !std::isfinite(y)) return;
  const Eigen::Vector2d pose(x, y);
  const double step = (pose - last_alignment_pose_).norm();
  if (step > 0.50) {
    // A localization jump is not distance driven by the chassis.
    last_alignment_pose_ = pose;
  } else if (step >= 0.005) {
    narrow_alignment_traveled_m_ += step;
    last_alignment_pose_ = pose;
  }
}

bool PathExecutor::transformedCommands(
  const interfaces::msg::MpcPositionCommand & trajectory,
  std::vector<interfaces::msg::PositionCommand> & commands) const
{
  commands = trajectory.cmds;
  const std::string source_frame = trajectory.header.frame_id.empty() ?
    params_.odom_frame : trajectory.header.frame_id;
  if (source_frame == params_.odom_frame) return true;
  if (!tf_buffer_) return false;

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(params_.odom_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }
  const double yaw_offset = tf2::getYaw(transform.transform.rotation);
  for (auto & command : commands) {
    geometry_msgs::msg::PointStamped point_in;
    geometry_msgs::msg::PointStamped point_out;
    geometry_msgs::msg::Vector3Stamped velocity_in;
    geometry_msgs::msg::Vector3Stamped velocity_out;
    geometry_msgs::msg::Vector3Stamped acceleration_in;
    geometry_msgs::msg::Vector3Stamped acceleration_out;
    geometry_msgs::msg::Vector3Stamped jerk_in;
    geometry_msgs::msg::Vector3Stamped jerk_out;
    point_in.header.frame_id = source_frame;
    point_in.point = command.position;
    velocity_in.header.frame_id = source_frame;
    velocity_in.vector = command.velocity;
    acceleration_in.header.frame_id = source_frame;
    acceleration_in.vector = command.acceleration;
    jerk_in.header.frame_id = source_frame;
    jerk_in.vector = command.jerk;
    if (!transform_vector(point_in, point_out, transform) ||
      !transform_vector(velocity_in, velocity_out, transform) ||
      !transform_vector(acceleration_in, acceleration_out, transform) ||
      !transform_vector(jerk_in, jerk_out, transform)) {
      return false;
    }
    command.position = point_out.point;
    command.velocity = velocity_out.vector;
    command.acceleration = acceleration_out.vector;
    command.jerk = jerk_out.vector;
    command.yaw = normalize_yaw(command.yaw + yaw_offset);
  }
  return true;
}

bool PathExecutor::applyClearanceSpeedCap(
  std::vector<ReferencePoint> & reference, double base_clearance) const
{
  if (!params_.clearance_braking.valid() || !rog_query_) return false;
  bool capped = false;
  for (auto & point : reference) {
    const double speed = point.vel.norm();
    if (!point.pos.allFinite() || !point.vel.allFinite() || speed <= 1e-6) continue;
    const auto clearance = rog_query_->query(Eigen::Vector3d(point.pos.x(), point.pos.y(), 0.0));
    if (!clearance.ok || !std::isfinite(clearance.distance)) continue;
    const double toward = obstacleApproachSpeed(
      point.vel.x(), point.vel.y(), clearance.gradient.x(), clearance.gradient.y());
    if (toward <= 1e-6) continue;
    const double allowed_toward = clearanceSpeedLimit(
      std::max(0.0, clearance.distance - base_clearance -
        kClearanceComparisonTolerance), params_.clearance_braking);
    if (toward > allowed_toward) {
      double vx = point.vel.x();
      double vy = point.vel.y();
      if (limitObstacleApproachVelocity(
          vx, vy, clearance.gradient.x(), clearance.gradient.y(), allowed_toward)) {
        point.vel = Eigen::Vector2d(vx, vy);
      } else {
        // An unusable gradient cannot identify the wall tangent.
        point.vel *= allowed_toward / toward;
      }
      capped = true;
    }
  }
  return capped;
}

bool PathExecutor::applyGoalApproachCap(
  std::vector<minco_controller::ReferencePoint> & reference,
  const std::vector<interfaces::msg::PositionCommand> & commands,
  double * remaining_out, double * allowed_out) const
{
  if (remaining_out != nullptr) {
    *remaining_out = std::numeric_limits<double>::quiet_NaN();
  }
  if (allowed_out != nullptr) {
    *allowed_out = std::numeric_limits<double>::quiet_NaN();
  }
  if (!params_.goal_approach.valid() || reference.empty() || commands.size() < 2U) {
    return false;
  }

  // 沿控制点累计弧长；参考点按最近控制点换算"到轨迹终点的剩余弧长"。
  std::vector<double> arc(commands.size(), 0.0);
  for (size_t index = 1; index < commands.size(); ++index) {
    const auto & previous = commands[index - 1U].position;
    const auto & current = commands[index].position;
    const double length = std::hypot(current.x - previous.x, current.y - previous.y);
    arc[index] = arc[index - 1U] + (std::isfinite(length) ? length : 0.0);
  }
  const double total = arc.back();
  if (!(std::isfinite(total) && total > 1e-6)) {
    // 原地转向 / 纯对齐轨迹没有可减速的行程。
    return false;
  }

  bool capped = false;
  // 诊断值取第一个参考点（≈ 机器人当前所在处）：日志里最有用的是"离终点还剩
  // 多少米、此刻允许多快"，而不是预测窗末尾那个必然为 0 的剩余距离。
  double current_remaining = std::numeric_limits<double>::quiet_NaN();
  double current_allowed = std::numeric_limits<double>::quiet_NaN();
  for (auto & point : reference) {
    if (!point.pos.allFinite() || !point.vel.allFinite()) {
      continue;
    }
    size_t nearest = 0U;
    double best_squared = std::numeric_limits<double>::infinity();
    for (size_t index = 0; index < commands.size(); ++index) {
      const double dx = commands[index].position.x - point.pos.x();
      const double dy = commands[index].position.y - point.pos.y();
      const double squared = dx * dx + dy * dy;
      if (squared < best_squared) {
        best_squared = squared;
        nearest = index;
      }
    }
    const double remaining = std::max(0.0, total - arc[nearest]);
    const double allowed = goalApproachSpeedLimit(remaining, params_.goal_approach);
    if (!std::isfinite(allowed)) {
      continue;
    }
    const double speed = point.vel.norm();
    if (speed > allowed) {
      // 只压速度，不改位置/航向：路径几何仍由 MINCO 决定，接近段只是更慢。
      point.vel *= allowed / speed;
      capped = true;
    }
    if (!std::isfinite(current_remaining)) {
      current_remaining = remaining;
      current_allowed = allowed;
    }
  }
  if (remaining_out != nullptr) {
    *remaining_out = current_remaining;
  }
  if (allowed_out != nullptr) {
    *allowed_out = current_allowed;
  }
  return capped;
}

bool PathExecutor::buildReference(
  const interfaces::msg::MpcPositionCommand & trajectory,
  const State & current,
  std::vector<ReferencePoint> & reference, bool follow_gimbal, bool reverse_heading,
  size_t * nearest_index, std::vector<interfaces::msg::PositionCommand> * transformed) const
{
  std::vector<interfaces::msg::PositionCommand> commands;
  if (!transformedCommands(trajectory, commands) || commands.empty()) return false;

  size_t nearest = 0;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < commands.size(); ++index) {
    const double dx = commands[index].position.x - current.x;
    const double dy = commands[index].position.y - current.y;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      nearest = index;
    }
  }
  if (nearest_index != nullptr) {
    *nearest_index = nearest;
  }
  if (transformed != nullptr) {
    *transformed = commands;
  }

  return sampleReference(commands, nearest, {}, reference, follow_gimbal, reverse_heading);
}

bool PathExecutor::sampleReference(
  const std::vector<interfaces::msg::PositionCommand> & commands,
  size_t nearest_index, const std::vector<double> & time_scales,
  std::vector<ReferencePoint> & reference, bool follow_gimbal, bool reverse_heading) const
{
  if (commands.empty() || config_.horizon <= 0 ||
    (time_scales.size() != 0U &&
    time_scales.size() != static_cast<size_t>(config_.horizon))) {
    return false;
  }
  if (!std::isfinite(params_.planner_frequency) || params_.planner_frequency <= 0.0 ||
    !std::isfinite(config_.dt) || config_.dt <= 0.0 ||
    !std::isfinite(params_.control_delay_s) || params_.control_delay_s < 0.0) {
    return false;
  }
  for (const double scale : time_scales) {
    if (!std::isfinite(scale) || scale <= 0.0 || scale > 1.0) {
      return false;
    }
  }
  const double planner_dt = 1.0 / params_.planner_frequency;
  const double current_time = static_cast<double>(nearest_index) * planner_dt + params_.control_delay_s;
  double elapsed = 0.0;
  reference.clear();
  reference.reserve(static_cast<size_t>(config_.horizon));
  for (int step = 0; step < config_.horizon; ++step) {
    if (step > 0) {
      elapsed += config_.dt * (time_scales.empty() ? 1.0 : time_scales[step - 1]);
    }
    const double target_time = current_time + elapsed;
    const double index_float = std::clamp(
      target_time / planner_dt, 0.0, static_cast<double>(commands.size() - 1U));
    const size_t index = static_cast<size_t>(std::floor(index_float));
    const size_t next = std::min(index + 1U, commands.size() - 1U);
    const double alpha = index_float - static_cast<double>(index);
    const double dt = alpha * planner_dt;
    const auto & command = commands[index];
    ReferencePoint point;
    point.pos = Eigen::Vector2d(command.position.x, command.position.y) +
      Eigen::Vector2d(command.velocity.x, command.velocity.y) * dt +
      Eigen::Vector2d(command.acceleration.x, command.acceleration.y) * (0.5 * dt * dt) +
      Eigen::Vector2d(command.jerk.x, command.jerk.y) * (dt * dt * dt / 6.0);
    point.vel = Eigen::Vector2d(command.velocity.x, command.velocity.y) +
      Eigen::Vector2d(command.acceleration.x, command.acceleration.y) * dt +
      Eigen::Vector2d(command.jerk.x, command.jerk.y) * (0.5 * dt * dt);
    const double yaw_delta = normalize_yaw(commands[next].yaw - command.yaw);
    point.yaw = command.yaw + alpha * yaw_delta;
    point.yaw_rate = command.yaw_dot + alpha * (commands[next].yaw_dot - command.yaw_dot);
    if (follow_gimbal) {
      Eigen::Vector2d tangent = point.vel;
      const double speed_squared = tangent.squaredNorm();
      if (!tangent.allFinite()) return false;
      if (speed_squared > 1e-8) {
        const Eigen::Vector2d acceleration(command.acceleration.x + command.jerk.x * dt,
          command.acceleration.y + command.jerk.y * dt);
        point.yaw_rate = (tangent.x() * acceleration.y() - tangent.y() * acceleration.x()) / speed_squared;
      } else {
        tangent.setZero();
        for (size_t j = index + 1; j < commands.size(); ++j) {
          tangent = {commands[j].position.x - command.position.x, commands[j].position.y - command.position.y};
          if (tangent.allFinite() && tangent.squaredNorm() > 1e-12) break;
        }
        if (tangent.squaredNorm() <= 1e-12) {
          for (size_t j = index; j > 0; --j) {
            tangent = {command.position.x - commands[j - 1].position.x, command.position.y - commands[j - 1].position.y};
            if (tangent.allFinite() && tangent.squaredNorm() > 1e-12) break;
          }
        }
        if (!tangent.allFinite() || tangent.squaredNorm() <= 1e-12) return false;
        point.yaw_rate = 0.0;
      }
      // One direction choice for the whole prediction horizon; reverse changes only yaw.
      point.yaw = normalize_yaw(std::atan2(tangent.y(), tangent.x()) +
        (reverse_heading ? 3.14159265358979323846 : 0.0));
      if (!std::isfinite(point.yaw_rate)) return false;
      point.yaw_rate = std::clamp(point.yaw_rate, config_.omega_min, config_.omega_max);
    }
    if (!time_scales.empty()) {
      // The MPC prediction still advances at config_.dt. Match feedforward
      // speed and yaw rate to the slowed position and heading references.
      point.vel *= time_scales[step];
      point.yaw_rate *= time_scales[step];
    }
    reference.push_back(point);
  }
  return true;
}

}  // namespace mas2027_nav_executor
