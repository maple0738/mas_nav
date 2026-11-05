#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include "interfaces/msg/mpc_position_command.hpp"
#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"
#include "tf2_ros/buffer.h"
#include "mas2027_nav_executor/path_planner/search/directed_terrain.hpp"

namespace mas2027_nav_executor {

// Heading requests and planner-gap holding use one semantic policy.
inline bool requiresPathAlignment(std::optional<uint8_t> label)
{
  return label == 5U || label == 6U || label == 7U;
}

struct RegionRule
{
  uint8_t label{};
  int mode{-1};
  double max_speed{};
  double max_acceleration{};
  double prepare_distance{};
  double activation_distance{};
  double commit_distance{};
  double release_distance{};
};

enum class RegionPhase : uint8_t
{
  NORMAL = 0,
  PREPARING = 1,
  ARMED = 2,
  COMMITTED = 3,
  INSIDE = 4,
  RELEASING = 5,
};

struct RegionSegment
{
  uint8_t label{};
  uint8_t mode{};
  double max_speed{};
  double max_acceleration{};
  double prepare_s{};
  double active_s{};
  double commit_s{};
  double enter_s{};
  double exit_s{};
  double release_s{};
  // The trajectory may end inside the region, making exit_s/release_s infinite.
  double release_distance{};
  Eigen::Vector2d entry_position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d entry_axis{Eigen::Vector2d::Zero()};
};

struct RegionPlan
{
  uint32_t trajectory_id{};
  std::vector<Eigen::Vector2d> positions_odom;
  std::vector<double> arc_lengths;
  std::vector<RegionSegment> segments;
};

std::optional<RegionPlan> annotateRegions(
  const interfaces::msg::MpcPositionCommand & trajectory,
  const TerrainGrid::Snapshot & terrain,
  const tf2_ros::Buffer & tf_buffer,
  const std::array<RegionRule, 3> & rules,
  const std::string & odom_frame,
  const nav_msgs::msg::OccupancyGrid * online_labels = nullptr,
  TerrainSampler terrain_sampler_odom = {});

// No static terrain: use fresh online 5/6 labels, otherwise ordinary mode.
std::optional<RegionPlan> annotateOnlineRegions(
  const interfaces::msg::MpcPositionCommand & trajectory,
  const tf2_ros::Buffer & tf_buffer,
  const std::array<RegionRule, 3> & rules,
  const std::string & odom_frame,
  const nav_msgs::msg::OccupancyGrid * online_labels = nullptr,
  TerrainSampler terrain_sampler_odom = {});

class RegionController final
{
public:
  RegionController(uint8_t normal_mode, double normal_speed, double normal_acceleration,
    double speed_blend_rate, double acceleration_blend_rate);
  void setPlan(std::shared_ptr<const RegionPlan> plan);
  // Front-entry slopes keep their configured alignment window. Other/legacy
  // regions may prepare earlier using the measured stopping distance.
  uint8_t update(const Eigen::Vector2d & position_odom,
    double minimum_prepare_distance = 0.0);
  // Once an undulating segment is entered, keep its mode and path-axis
  // alignment for this many metres without requiring further labels.
  void setUndulatingAlignmentLatch(bool enable, double distance)
  {
    if (!std::isfinite(distance) || distance < 0.0)
      throw std::invalid_argument("undulating alignment latch distance must be finite and non-negative");
    undulating_latch_enable_ = enable;
    undulating_latch_distance_ = distance;
  }
  void setFrontEntryGate(bool enable, double lateral_tolerance = 0.15, double cosine = 0.8,
    double lateral_hysteresis = 0.05)
  {
    front_entry_enable_ = enable;
    front_entry_lateral_ = lateral_tolerance;
    front_entry_cosine_ = cosine;
    front_entry_lateral_hysteresis_ = lateral_hysteresis;
  }
  bool alignmentActive() const
  { return alignment_segment_.has_value() || retained_alignment_active_; }
  // Remaining path length to the next slope's alignment window, while still
  // spinning. The executor can brake toward it without requesting follow yet.
  std::optional<double> distanceToSlopeAlignment() const;
  void tickProfile(double dt);
  uint8_t mode() const { return mode_; }
  double speedLimit() const { return speed_limit_; }
  double accelerationLimit() const { return acceleration_limit_; }
  double progress() const { return progress_; }
  std::optional<uint8_t> label() const { return label_; }
  RegionPhase phase() const { return phase_; }
  // Labels 5/6/7 request path tangent minus LIO yaw, degrees [-180, 180].
  // Ordinary geometry alone never requests heading alignment.
  std::optional<double> trajectoryHeadingErrorDegrees(double lio_yaw_rad) const;

private:
  bool frontEntryAllowed(const RegionSegment &, const Eigen::Vector2d &) const;
  bool front_entry_enable_{false};
  double front_entry_lateral_{0.15}, front_entry_cosine_{0.8};
  double front_entry_lateral_hysteresis_{0.05};
  uint8_t normal_mode_{};
  double normal_speed_{};
  double normal_acceleration_{};
  double speed_blend_rate_{};
  double acceleration_blend_rate_{};
  uint8_t mode_{};
  double speed_limit_{std::numeric_limits<double>::infinity()};
  double acceleration_limit_{std::numeric_limits<double>::infinity()};
  double target_speed_limit_{std::numeric_limits<double>::infinity()};
  double target_acceleration_limit_{std::numeric_limits<double>::infinity()};
  double progress_{};
  std::optional<uint8_t> label_;
  RegionPhase phase_{RegionPhase::NORMAL};
  std::shared_ptr<const RegionPlan> plan_;
  // Plans are immutable after publication. Validate their arc ordering once.
  bool arc_lengths_searchable_{false};
  std::optional<size_t> held_segment_;
  std::optional<size_t> alignment_segment_;
  std::optional<Eigen::Vector2d> last_position_odom_;
  struct AlignmentMemory
  {
    RegionSegment segment;
    Eigen::Vector2d position_odom;
    double release_distance{};
    uint8_t mode{};
    uint32_t trajectory_id{};
  };
  std::optional<AlignmentMemory> alignment_memory_;
  struct UndulatingLatch
  {
    Eigen::Vector2d start_position;
    double distance{};
    uint8_t label{7};
    uint8_t mode{7};
    double max_speed{};
    double max_acceleration{};
  };
  std::optional<UndulatingLatch> undulating_latch_;
  std::optional<double> undulating_released_progress_;
  bool undulating_latch_enable_{true};
  double undulating_latch_distance_{3.2};
  bool retained_alignment_active_{};
};

}  // namespace mas2027_nav_executor
