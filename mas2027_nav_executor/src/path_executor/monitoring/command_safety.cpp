#include "mas2027_nav_executor/path_executor/monitoring/command_safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "mas2027_nav_executor/common/environment/clearance_gate.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mas2027_nav_executor {

ExecutorStatus checkCommandSafety(
  const std::shared_ptr<TerrainGrid> & grid,
  const std::shared_ptr<rog_map::MapQueryInterface> & rog_query,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & odom_frame,
  double rog_map_clearance,
  bool narrow_passage_enable,
  double narrow_passage_clearance,
  bool narrow_passage_ahead,
  double rog_map_timeout_s,
  double dt,
  const minco_controller::State & current,
  const minco_controller::Control & control,
  const rclcpp::Time & stamp,
  CommandSafetyDetail * detail,
  const ClearanceBrakingParams & braking)
{
  auto reject = [detail](ExecutorStatus status, const char * reason, double value = 0.0,
                   double threshold = 0.0) {
    if (detail) {
      detail->reason = reason;
      detail->value = value;
      detail->threshold = threshold;
    }
    return status;
  };
  const auto terrain = grid ? grid->snapshot() : nullptr;
  if ((grid && (!terrain || !tf)) || !rog_query) {
    return reject(ExecutorStatus::TERRAIN_BLOCKED, "grid_or_frame_missing");
  }
  const double rog_stamp_s = rog_query->snapshotStampSeconds();
  const double rog_age_s = std::isfinite(rog_stamp_s) ?
    std::abs(stamp.seconds() - rog_stamp_s) : std::numeric_limits<double>::infinity();
  if (!std::isfinite(rog_age_s) || rog_age_s > rog_map_timeout_s) {
    return reject(ExecutorStatus::DYNAMIC_BLOCKED, "rog_map_stale", rog_age_s, rog_map_timeout_s);
  }
  if (!std::isfinite(dt) || dt <= 0.0 ||
    !std::isfinite(current.x) || !std::isfinite(current.y) ||
    !std::isfinite(control.vx) || !std::isfinite(control.vy)) {
    return reject(ExecutorStatus::DYNAMIC_BLOCKED, "invalid_command");
  }
  const double horizon = std::max(dt, 0.35);
  if (terrain) {
    try {
      const auto transform = tf->lookupTransform(
        terrain->cost.header.frame_id, odom_frame, tf2::TimePointZero);
      const double map_yaw = tf2::getYaw(transform.transform.rotation);
      const double mc = std::cos(map_yaw), ms = std::sin(map_yaw);
      const Eigen::Vector2d position(
        mc * current.x - ms * current.y + transform.transform.translation.x,
        ms * current.x + mc * current.y + transform.transform.translation.y);
      const Eigen::Vector2d velocity(
        mc * control.vx - ms * control.vy,
        ms * control.vx + mc * control.vy);
      if (!terrain->transition(position, position + horizon * velocity)) {
        return reject(ExecutorStatus::TERRAIN_BLOCKED, "terrain_transition");
      }
    } catch (const tf2::TransformException &) {
      return reject(ExecutorStatus::TERRAIN_BLOCKED, "tf_unavailable");
    }
  }
  // 净空判据与规划侧发布前整条轨迹检查一致：近场半径取车体安全半径 rog_map_clearance。
  // 场内不比实测净空更差即可，出近场必须满足该阈值；否则贴墙停下会让指令在 t=0 被否决。
  // 有效阈值须减掉 ESDF 抖动余量 kEsdfJitterTolerance，与发布门同值。
  const double effective_clearance = effectiveClearanceThreshold(rog_map_clearance);
  const auto current_clearance =
    rog_query->query(Eigen::Vector3d(current.x, current.y, 0.0));
  const bool current_narrow = narrow_passage_enable && current_clearance.ok &&
    std::isfinite(current_clearance.distance) &&
    current_clearance.distance < effective_clearance &&
    current_clearance.distance >= narrow_passage_clearance;
  // The committed trajectory can enter the narrow band before the body's own
  // clearance drops below the full threshold. In that case the executor has
  // already capped speed; use the same floor for the short command horizon so
  // the gate does not brake exactly at the corridor mouth. If the current pose
  // is already tighter than the floor, never grant the ahead relaxation.
  const bool ahead_narrow = narrow_passage_enable && narrow_passage_ahead &&
    current_clearance.ok && std::isfinite(current_clearance.distance) &&
    current_clearance.distance >= narrow_passage_clearance;
  const bool narrow_continuation = current_narrow || ahead_narrow;
  const double narrow_gate_clearance = current_narrow ?
    current_clearance.distance : narrow_passage_clearance;
  const double command_clearance = narrow_continuation ?
    std::max(narrow_passage_clearance, narrow_gate_clearance - 0.02) : effective_clearance;
  const double command_near_field = narrow_continuation ?
    std::numeric_limits<double>::infinity() : rog_map_clearance;
  const ClearanceRequirement clearance_gate = makeClearanceRequirement(
    command_clearance, command_near_field, current_clearance.distance, current_clearance.ok,
    0.02, narrow_continuation ? narrow_passage_clearance : 0.0);
  const double travel_speed = std::hypot(control.vx, control.vy);
  const int command_steps = std::max(1, static_cast<int>(std::ceil(horizon / dt)));
  for (int i = 0; i <= command_steps; ++i) {
    const double t = horizon * i / command_steps;
    const Eigen::Vector3d odom_point(current.x + t * control.vx,
      current.y + t * control.vy, 0.0);
    const auto clearance = rog_query->query(odom_point);
    const double toward_obstacle = obstacleApproachSpeed(
      control.vx, control.vy, clearance.gradient.x(), clearance.gradient.y());
    // Near-field relaxation is only for escaping an already tight pose. It
    // must not reduce the footprint requirement while driving toward a wall.
    double base_required = braking.valid() && toward_obstacle > 1e-3 ?
      command_clearance : clearance_gate.requiredAt(t * travel_speed);
    // Below the physical floor, permit only a command that increases ESDF
    // distance. Near-field relaxation must not authorize sliding along a wall
    // while already inside the requested body gap.
    if (braking.valid() && current_clearance.ok &&
        current_clearance.distance < narrow_passage_clearance) {
      const double escape_rate = control.vx * clearance.gradient.x() +
        control.vy * clearance.gradient.y();
      if (!(std::isfinite(escape_rate) && escape_rate > 1e-3)) {
        base_required = std::max(base_required, narrow_passage_clearance);
      }
    }
    const double required = base_required + brakingClearance(toward_obstacle, braking);
    const bool clearance_ok = clearance.ok &&
      clearance.distance + kClearanceComparisonTolerance >= required;
    if (!clearance_ok) {
      return reject(ExecutorStatus::DYNAMIC_BLOCKED,
        clearance.ok ? "clearance" : "clearance_out_of_map",
        clearance.distance, required);
    }
  }
  return reject(ExecutorStatus::PUBLISHED, "none");
}

}  // namespace mas2027_nav_executor
