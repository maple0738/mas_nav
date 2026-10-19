#include "mas2027_nav_executor/common/environment/region_control.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "tf2/utils.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mas2027_nav_executor {
namespace {

struct Rigid2D
{
  double c{1.0};
  double s{};
  Eigen::Vector2d translation{Eigen::Vector2d::Zero()};

  Eigen::Vector2d apply(const Eigen::Vector2d & p) const
  {
    return {c * p.x() - s * p.y() + translation.x(),
      s * p.x() + c * p.y() + translation.y()};
  }
};

Rigid2D lookupRigid(const tf2_ros::Buffer & buffer,
  const std::string & target, const std::string & source)
{
  if (target == source) return {};
  const auto transform = buffer.lookupTransform(target, source, tf2::TimePointZero);
  const double yaw = tf2::getYaw(transform.transform.rotation);
  return {std::cos(yaw), std::sin(yaw),
    {transform.transform.translation.x, transform.transform.translation.y}};
}

const RegionRule * findRule(const std::array<RegionRule, 3> & rules, uint8_t label)
{
  for (const auto & rule : rules) {
    if (rule.label == label) return &rule;
  }
  return nullptr;
}

std::optional<uint8_t> onlineLabelAt(
  const nav_msgs::msg::OccupancyGrid & grid, const Eigen::Vector2d & point)
{
  const auto & info = grid.info;
  if (!point.allFinite() || info.width == 0 || info.height == 0 ||
      !std::isfinite(info.resolution) || info.resolution <= 0.0F ||
      grid.data.size() != static_cast<size_t>(info.width) * info.height ||
      std::abs(info.origin.orientation.x) > 1e-6 ||
      std::abs(info.origin.orientation.y) > 1e-6 ||
      std::abs(info.origin.orientation.z) > 1e-6 ||
      std::abs(info.origin.orientation.w - 1.0) > 1e-6) return std::nullopt;
  const double fx = (point.x() - info.origin.position.x) / info.resolution;
  const double fy = (point.y() - info.origin.position.y) / info.resolution;
  if (!std::isfinite(fx) || !std::isfinite(fy) || fx < 0.0 || fy < 0.0 ||
      fx >= info.width || fy >= info.height) return std::nullopt;
  const auto x = static_cast<size_t>(fx);
  const auto y = static_cast<size_t>(fy);
  const int8_t value = grid.data[y * info.width + x];
  // An ordinary observation is not enough to erase a manually marked
  // special region; only positively identified online regions take priority.
  if (value != 5 && value != 6 && value != 7) return std::nullopt;
  return static_cast<uint8_t>(value);
}

double projectProgress(const RegionPlan & plan, const Eigen::Vector2d & position,
  double minimum_s, double maximum_s)
{
  double best_distance = std::numeric_limits<double>::infinity();
  double best_s = 0.0;
  for (size_t i = 1; i < plan.positions_odom.size(); ++i) {
    if (plan.arc_lengths[i] < minimum_s || plan.arc_lengths[i - 1] > maximum_s) continue;
    const Eigen::Vector2d delta = plan.positions_odom[i] - plan.positions_odom[i - 1];
    const double length_squared = delta.squaredNorm();
    double u = length_squared > 1e-12 ?
      std::clamp((position - plan.positions_odom[i - 1]).dot(delta) / length_squared, 0.0, 1.0) : 0.0;
    if (plan.arc_lengths[i] > plan.arc_lengths[i - 1]) {
      const double segment_length = plan.arc_lengths[i] - plan.arc_lengths[i - 1];
      u = std::clamp(u,
        std::clamp((minimum_s - plan.arc_lengths[i - 1]) / segment_length, 0.0, 1.0),
        std::clamp((maximum_s - plan.arc_lengths[i - 1]) / segment_length, 0.0, 1.0));
    }
    const double distance = (position - plan.positions_odom[i - 1] - u * delta).squaredNorm();
    if (distance < best_distance) {
      best_distance = distance;
      best_s = plan.arc_lengths[i - 1] + u * (plan.arc_lengths[i] - plan.arc_lengths[i - 1]);
    }
  }
  return best_s;
}

}  // namespace

static std::optional<RegionPlan> annotateRegionsImpl(
  const interfaces::msg::MpcPositionCommand & trajectory,
  const TerrainGrid::Snapshot * terrain,
  const tf2_ros::Buffer & tf_buffer,
  const std::array<RegionRule, 3> & rules,
  const std::string & odom_frame,
  const nav_msgs::msg::OccupancyGrid * online_labels)
{
  if (trajectory.cmds.size() < 2) return std::nullopt;
  const std::string source = trajectory.header.frame_id.empty() ? odom_frame :
    trajectory.header.frame_id;
  Rigid2D to_map, to_odom;
  std::optional<Rigid2D> to_online;
  try {
    if (terrain) to_map = lookupRigid(tf_buffer, terrain->cost.header.frame_id, source);
    to_odom = lookupRigid(tf_buffer, odom_frame, source);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
  if (online_labels && !online_labels->header.frame_id.empty() &&
      online_labels->info.width > 0 && online_labels->info.height > 0 &&
      std::isfinite(online_labels->info.resolution) && online_labels->info.resolution > 0.0F &&
      online_labels->data.size() ==
        static_cast<size_t>(online_labels->info.width) * online_labels->info.height) {
    try {
      to_online = lookupRigid(tf_buffer, online_labels->header.frame_id, source);
    } catch (const tf2::TransformException &) {
      // Static annotation remains available while the online frame is unresolved.
    }
  }

  RegionPlan plan;
  plan.trajectory_id = trajectory.cmds.front().trajectory_id;
  plan.positions_odom.reserve(trajectory.cmds.size());
  plan.arc_lengths.reserve(trajectory.cmds.size());
  double arc = 0.0;
  for (size_t i = 0; i < trajectory.cmds.size(); ++i) {
    const auto & cmd = trajectory.cmds[i];
    if (cmd.trajectory_id != plan.trajectory_id ||
      !std::isfinite(cmd.position.x) || !std::isfinite(cmd.position.y)) return std::nullopt;
    const Eigen::Vector2d source_point(cmd.position.x, cmd.position.y);
    plan.positions_odom.push_back(to_odom.apply(source_point));
    if (i > 0) arc += (plan.positions_odom[i] - plan.positions_odom[i - 1]).norm();
    plan.arc_lengths.push_back(arc);
  }

  const double static_resolution = terrain ? terrain->cost.info.resolution : 0.05;
  const double spacing = 0.5 * std::min<double>(static_resolution,
    to_online ? online_labels->info.resolution : static_resolution);
  if (!std::isfinite(spacing) || spacing <= 0.0) return std::nullopt;
  std::optional<RegionSegment> active;
  const auto finish = [&](double s) {
    if (!active) return;
    active->exit_s = s;
    active->release_s = s + findRule(rules, active->label)->release_distance;
    plan.segments.push_back(*active);
    active.reset();
  };
  for (size_t i = 1; i < trajectory.cmds.size(); ++i) {
    const Eigen::Vector2d a(trajectory.cmds[i - 1].position.x,
      trajectory.cmds[i - 1].position.y);
    const Eigen::Vector2d b(trajectory.cmds[i].position.x,
      trajectory.cmds[i].position.y);
    const double length = (b - a).norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int j = (i == 1 ? 0 : 1); j <= samples; ++j) {
      const double u = static_cast<double>(j) / samples;
      const double s = plan.arc_lengths[i - 1] + u * length;
      const Eigen::Vector2d sample = a + u * (b - a);
      const auto static_label = terrain ? terrain->terrainLabelAt(to_map.apply(sample)) :
        std::optional<uint8_t>(0U);
      if (!static_label) return std::nullopt;
      const auto observed_label = to_online ?
        onlineLabelAt(*online_labels, to_online->apply(sample)) : std::nullopt;
      const uint8_t label = observed_label.value_or(*static_label);
      const RegionRule * rule = findRule(rules, label);
      if (active && (!rule || active->label != label)) finish(s);
      if (rule && !active) {
        if (rule->mode < 0 || rule->mode > 255) return std::nullopt;
        active = RegionSegment{label, static_cast<uint8_t>(rule->mode),
          rule->max_speed, rule->max_acceleration,
          std::max(0.0, s - rule->prepare_distance),
          std::max(0.0, s - rule->activation_distance),
          std::max(0.0, s - rule->commit_distance), s, 0.0, 0.0};
      }
    }
  }
  if (active) {
    active->exit_s = std::numeric_limits<double>::infinity();
    active->release_s = std::numeric_limits<double>::infinity();
    plan.segments.push_back(*active);
  }
  std::vector<RegionSegment> merged;
  for (const auto & segment : plan.segments) {
    if (!merged.empty() && segment.active_s < merged.back().release_s) {
      if (segment.mode != merged.back().mode) return std::nullopt;
      auto & previous = merged.back();
      previous.exit_s = std::max(previous.exit_s, segment.exit_s);
      previous.release_s = std::max(previous.release_s, segment.release_s);
      previous.max_speed = std::min(previous.max_speed, segment.max_speed);
      previous.max_acceleration = std::min(previous.max_acceleration,
        segment.max_acceleration);
    } else {
      merged.push_back(segment);
    }
  }
  plan.segments = std::move(merged);
  return plan;
}

std::optional<RegionPlan> annotateRegions(
  const interfaces::msg::MpcPositionCommand & trajectory,
  const TerrainGrid::Snapshot & terrain,
  const tf2_ros::Buffer & tf_buffer,
  const std::array<RegionRule, 3> & rules,
  const std::string & odom_frame,
  const nav_msgs::msg::OccupancyGrid * online_labels)
{
  return annotateRegionsImpl(trajectory, &terrain, tf_buffer, rules, odom_frame, online_labels);
}

std::optional<RegionPlan> annotateOnlineRegions(
  const interfaces::msg::MpcPositionCommand & trajectory,
  const tf2_ros::Buffer & tf_buffer,
  const std::array<RegionRule, 3> & rules,
  const std::string & odom_frame,
  const nav_msgs::msg::OccupancyGrid * online_labels)
{
  return annotateRegionsImpl(trajectory, nullptr, tf_buffer, rules, odom_frame, online_labels);
}

RegionController::RegionController(uint8_t normal_mode, double normal_speed,
  double normal_acceleration, double speed_blend_rate, double acceleration_blend_rate)
: normal_mode_(normal_mode), normal_speed_(normal_speed),
  normal_acceleration_(normal_acceleration), speed_blend_rate_(speed_blend_rate),
  acceleration_blend_rate_(acceleration_blend_rate), mode_(normal_mode),
  speed_limit_(normal_speed), acceleration_limit_(normal_acceleration),
  target_speed_limit_(normal_speed), target_acceleration_limit_(normal_acceleration) {}

void RegionController::setPlan(std::shared_ptr<const RegionPlan> plan)
{
  if (plan_ == plan) return;
  alignment_segment_.reset();
  if (plan_ && plan && plan_->trajectory_id == plan->trajectory_id) {
    // The online semantic map can change while the trajectory stays the same.
    // Preserve path progress but reselect the active segment from the new plan.
    const uint8_t held_mode = held_segment_ && *held_segment_ < plan_->segments.size() ?
      plan_->segments[*held_segment_].mode : normal_mode_;
    plan_ = std::move(plan);
    held_segment_.reset();
    if (held_mode != normal_mode_) {
      for (size_t i = 0; i < plan_->segments.size(); ++i) {
        const auto & segment = plan_->segments[i];
        if (segment.mode == held_mode && progress_ >= segment.active_s &&
            progress_ < segment.release_s) {
          held_segment_ = i;
          break;
        }
      }
    }
    return;
  }
  plan_ = std::move(plan);
  held_segment_.reset();
  progress_ = 0.0;
  phase_ = mode_ == normal_mode_ ? RegionPhase::NORMAL : RegionPhase::RELEASING;
  last_position_odom_.reset();
}

uint8_t RegionController::update(const Eigen::Vector2d & position_odom)
{
  alignment_segment_.reset();
  if (!plan_ || plan_->positions_odom.size() < 2 || !position_odom.allFinite()) return mode_;
  const double moved = last_position_odom_ ?
    (position_odom - *last_position_odom_).norm() : 0.25;
  const double maximum_s = progress_ + 2.0 * moved + 0.02;
  progress_ = std::max(progress_, projectProgress(*plan_, position_odom,
    std::max(0.0, progress_ - 0.05), maximum_s));
  last_position_odom_ = position_odom;
  if (held_segment_) {
    const auto & held = plan_->segments[*held_segment_];
    if (progress_ < held.release_s) {
      alignment_segment_ = held_segment_;
      mode_ = held.mode;
      target_speed_limit_ = held.max_speed;
      target_acceleration_limit_ = held.max_acceleration;
      label_ = held.label;
      phase_ = progress_ < held.commit_s ? RegionPhase::ARMED :
        progress_ < held.enter_s ? RegionPhase::COMMITTED :
        progress_ < held.exit_s ? RegionPhase::INSIDE : RegionPhase::RELEASING;
      return mode_;
    }
    held_segment_.reset();
  }
  for (size_t i = 0; i < plan_->segments.size(); ++i) {
    const auto & segment = plan_->segments[i];
    if (progress_ < segment.prepare_s) break;
    if (progress_ < segment.active_s) {
      alignment_segment_ = i;
      target_speed_limit_ = segment.max_speed;
      target_acceleration_limit_ = segment.max_acceleration;
      label_ = segment.label;
      mode_ = normal_mode_;
      phase_ = RegionPhase::PREPARING;
      return mode_;
    }
    if (progress_ < segment.release_s) {
      alignment_segment_ = i;
      held_segment_ = i;
      mode_ = segment.mode;
      target_speed_limit_ = segment.max_speed;
      target_acceleration_limit_ = segment.max_acceleration;
      label_ = segment.label;
      phase_ = progress_ < segment.commit_s ? RegionPhase::ARMED :
        progress_ < segment.enter_s ? RegionPhase::COMMITTED :
        progress_ < segment.exit_s ? RegionPhase::INSIDE : RegionPhase::RELEASING;
      return mode_;
    }
  }
  mode_ = normal_mode_;
  target_speed_limit_ = normal_speed_;
  target_acceleration_limit_ = normal_acceleration_;
  label_.reset();
  phase_ = RegionPhase::NORMAL;
  return mode_;
}

std::optional<double> RegionController::trajectoryHeadingErrorDegrees(double lio_yaw_rad) const
{
  if (!plan_ || !std::isfinite(lio_yaw_rad) ||
    !alignment_segment_ || *alignment_segment_ >= plan_->segments.size() ||
    plan_->positions_odom.size() != plan_->arc_lengths.size()) return std::nullopt;
  // Follow the current MINCO path tangent, including changes inside a curved region.
  // At a vertex prefer the outgoing segment; at the end retain the last nonzero tangent.
  const double target_s = progress_;
  for (size_t i = 1; i < plan_->positions_odom.size(); ++i) {
    if (plan_->arc_lengths[i] <= target_s + 1e-6) continue;
    const Eigen::Vector2d direction = plan_->positions_odom[i] - plan_->positions_odom[i - 1];
    if (!direction.allFinite() || direction.squaredNorm() <= 1e-12) continue;
    const double offset = std::atan2(direction.y(), direction.x()) - lio_yaw_rad;
    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    return std::atan2(std::sin(offset), std::cos(offset)) * kRadiansToDegrees;
  }
  for (size_t i = plan_->positions_odom.size(); i > 1; --i) {
    const Eigen::Vector2d direction = plan_->positions_odom[i - 1] - plan_->positions_odom[i - 2];
    if (!direction.allFinite() || direction.squaredNorm() <= 1e-12) continue;
    const double offset = std::atan2(direction.y(), direction.x()) - lio_yaw_rad;
    return std::atan2(std::sin(offset), std::cos(offset)) * (180.0 / 3.14159265358979323846);
  }
  return std::nullopt;
}

void RegionController::tickProfile(double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0) return;
  const auto blend = [dt](double current, double target, double rate) {
    return current + std::clamp(target - current, -rate * dt, rate * dt);
  };
  speed_limit_ = blend(speed_limit_, target_speed_limit_, speed_blend_rate_);
  acceleration_limit_ = blend(
    acceleration_limit_, target_acceleration_limit_, acceleration_blend_rate_);
  // A faster special-region profile must not raise the ordinary-mode limit before
  // the special mode is requested, or remain raised after the release point.
  if (mode_ == normal_mode_) {
    speed_limit_ = std::min(speed_limit_, normal_speed_);
    acceleration_limit_ = std::min(acceleration_limit_, normal_acceleration_);
  }
  if (phase_ == RegionPhase::COMMITTED || phase_ == RegionPhase::INSIDE ||
    phase_ == RegionPhase::RELEASING) {
    if (std::isfinite(target_speed_limit_)) {
      speed_limit_ = std::min(speed_limit_, target_speed_limit_);
    }
    if (std::isfinite(target_acceleration_limit_)) {
      acceleration_limit_ = std::min(acceleration_limit_, target_acceleration_limit_);
    }
  }
}

}  // namespace mas2027_nav_executor
