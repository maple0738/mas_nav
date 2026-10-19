#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include "interfaces/msg/mpc_position_command.hpp"
#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"
#include "tf2_ros/buffer.h"

namespace mas2027_nav_executor {

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
  const nav_msgs::msg::OccupancyGrid * online_labels = nullptr);

// No static terrain: use fresh online 5/6 labels, otherwise ordinary mode.
std::optional<RegionPlan> annotateOnlineRegions(
  const interfaces::msg::MpcPositionCommand & trajectory,
  const tf2_ros::Buffer & tf_buffer,
  const std::array<RegionRule, 3> & rules,
  const std::string & odom_frame,
  const nav_msgs::msg::OccupancyGrid * online_labels = nullptr);

class RegionController final
{
public:
  RegionController(uint8_t normal_mode, double normal_speed, double normal_acceleration,
    double speed_blend_rate, double acceleration_blend_rate);
  void setPlan(std::shared_ptr<const RegionPlan> plan);
  uint8_t update(const Eigen::Vector2d & position_odom);
  void tickProfile(double dt);
  uint8_t mode() const { return mode_; }
  double speedLimit() const { return speed_limit_; }
  double accelerationLimit() const { return acceleration_limit_; }
  double progress() const { return progress_; }
  std::optional<uint8_t> label() const { return label_; }
  RegionPhase phase() const { return phase_; }
  // Current forward trajectory tangent minus LIO yaw; output degrees [-180, 180].
  std::optional<double> trajectoryHeadingErrorDegrees(double lio_yaw_rad) const;

private:
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
  std::optional<size_t> held_segment_;
  std::optional<size_t> alignment_segment_;
  std::optional<Eigen::Vector2d> last_position_odom_;
};

}  // namespace mas2027_nav_executor
