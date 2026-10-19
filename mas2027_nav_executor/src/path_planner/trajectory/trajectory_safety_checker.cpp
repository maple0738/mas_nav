#include "mas2027_nav_executor/path_planner/trajectory/trajectory_safety_checker.hpp"

#include "data_structure/base/trajectory.h"
#include "mas2027_nav_executor/common/environment/clearance_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace minco_planner {

void TrajectorySafetyChecker::configure(double sample_dt, rclcpp::Logger logger)
{
  sample_dt_ = sample_dt > 1e-6 ? sample_dt : 0.05;
  logger_ = logger;
}

void TrajectorySafetyChecker::setQuery(std::shared_ptr<rog_map::MapQueryInterface> dynamic_query)
{
  dynamic_query_ = std::move(dynamic_query);
}

void TrajectorySafetyChecker::setNarrowPassage(bool enable, double minimum_clearance)
{
  narrow_passage_enable_ = enable && std::isfinite(minimum_clearance) && minimum_clearance > 0.0;
  narrow_passage_clearance_ = narrow_passage_enable_ ? minimum_clearance : 0.0;
}

bool TrajectorySafetyChecker::ensureQueryAvailable() const
{
  if (dynamic_query_) {
    return true;
  }
  RCLCPP_ERROR_THROTTLE(logger_,
    *rclcpp::Clock::make_shared(),
    1000,
    "[MincoPlanner] ROGMap dynamic query is unavailable for trajectory safety check.");
  return false;
}

bool TrajectorySafetyChecker::checkPoint(const Eigen::Vector3d & pos, double check_dist) const
{
  // 先按 layer 致命/膨胀代价否决，再比 ROGMap field 净空与 check_dist（已扣除 inflation_radius）。
  if (!ensureQueryAvailable()) {
    return false;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!dynamic_query_->worldToMap(pos.x(), pos.y(), mx, my)) {
    return false;
  }
  const unsigned char cost = dynamic_query_->value(mx, my);
  if (cost == kLethalCost || cost == kInscribedCost) {
    return false;
  }
  double esdf_dist = 0.0;
  Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
  const auto query = dynamic_query_->query(pos);
  if (!query.ok) {
    RCLCPP_WARN_THROTTLE(logger_,
      *rclcpp::Clock::make_shared(),
      1000,
      "[MincoPlanner] Trajectory safety query failed: %s",
      rog_map::queryStatusName(query.status));
    return false;
  }
  esdf_dist = query.distance;
  esdf_grad = query.gradient;
  (void)esdf_grad;
  const bool finite_clearance = std::isfinite(esdf_dist);
  const bool clearance_ok = finite_clearance &&
    esdf_dist + mas2027_nav_executor::kClearanceComparisonTolerance >= check_dist;
  if (finite_clearance && !clearance_ok) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "Trajectory clearance %.3f m below required %.3f m at (%.2f, %.2f)",
      esdf_dist, check_dist, pos.x(), pos.y());
  }
  return clearance_ok;
}

bool TrajectorySafetyChecker::checkTrajectory(
  const traj_opt::Trajectory & traj, double check_dist, double near_field,
  const std::vector<Eigen::Vector3d> & reference_path) const
{
  if (!ensureQueryAvailable()) {
    return false;
  }

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return true;
  }

  // 近场判据：起点处的实测净空是“机器人现在到底有多近”的权威值。拿不到就不放宽。
  const Eigen::Vector3d start_pos = traj.getPos(0.0);
  double start_clearance = 0.0;
  bool start_clearance_ok = false;
  if (near_field > 1e-6) {
    const auto start_query = dynamic_query_->query(start_pos);
    if (start_query.ok) {
      start_clearance = start_query.distance;
      start_clearance_ok = true;
    } else {
      // 起点查不到净空（如 OUT_OF_MAP）不等于轨迹不安全：只关闭近场放宽，逐点完整判据仍生效。
      RCLCPP_WARN_THROTTLE(logger_,
        *rclcpp::Clock::make_shared(),
        1000,
        "[MincoPlanner] Near-field clearance query failed: %s; near-field relaxation disabled for this check",
        rog_map::queryStatusName(start_query.status));
    }
  }

  // 只按起点净空判断会漏掉“起点在开阔处、前方 1 m 才是窄口”的常见情况：
  // 全局折线已经证明计划走廊可以走，但 MINCO 的整条轨迹检查仍用 0.26 m
  // 否决它，表现为到了窄口前反复停顿。这里额外检查参考折线；只有轨迹采样
  // 仍靠近该折线的窄段时，才把要求降到窄道地板，偏离走廊仍按完整阈值。
  struct ReferenceSample
  {
    Eigen::Vector3d point;
    double clearance{0.0};
    double arc{0.0};
    bool near_narrow_corridor{false};
  };
  std::vector<ReferenceSample> reference_samples;
  double reference_min_clearance = std::numeric_limits<double>::infinity();
  bool has_safe_narrow_sample = false;
  if (narrow_passage_enable_ && reference_path.size() >= 2U) {
    reference_samples.reserve(reference_path.size());
    double reference_arc = 0.0;
    Eigen::Vector3d previous_point = Eigen::Vector3d::Zero();
    bool have_previous_point = false;
    for (const auto & point : reference_path) {
      if (!point.allFinite()) {
        continue;
      }
      if (have_previous_point) {
        reference_arc += (point - previous_point).head<2>().norm();
      }
      previous_point = point;
      have_previous_point = true;
      const auto query = dynamic_query_->query(point);
      if (!query.ok || !std::isfinite(query.distance)) {
        continue;
      }
      reference_samples.push_back({point, query.distance, reference_arc, false});
      reference_min_clearance = std::min(reference_min_clearance, query.distance);
      has_safe_narrow_sample = has_safe_narrow_sample ||
        (query.distance >= narrow_passage_clearance_ && query.distance < check_dist);
    }
  }
  const bool start_narrow =
    narrow_passage_enable_ && start_clearance_ok &&
    start_clearance < check_dist && start_clearance >= narrow_passage_clearance_;
  const bool planned_narrow =
    narrow_passage_enable_ && has_safe_narrow_sample;
  const bool narrow_continuation = start_narrow || planned_narrow;
  // The narrow sample is not an isolated point: the approach and exit along
  // the same planned route belong to the same corridor. Without this context,
  // a trajectory before the first tight sample can be judged at the full
  // threshold even when it stays on the same narrow route.
  if (planned_narrow) {
    constexpr double kNarrowCorridorContext = 0.50;
    for (auto & sample : reference_samples) {
      for (const auto & narrow : reference_samples) {
        if (narrow.clearance >= narrow_passage_clearance_ &&
          narrow.clearance < check_dist &&
          std::abs(sample.arc - narrow.arc) <= kNarrowCorridorContext) {
          sample.near_narrow_corridor = true;
          break;
        }
      }
    }
  }

  // 起点已经贴窄时，继续沿用原有近场豁免；计划走廊窄时不需要把近场
  // 无限延伸，而是按采样点到参考折线的距离逐点放行。
  const double effective_near_field = start_narrow ?
    std::numeric_limits<double>::infinity() : near_field;
  const double effective_near_floor = start_narrow ? narrow_passage_clearance_ : 0.0;
  const mas2027_nav_executor::ClearanceRequirement gate =
    mas2027_nav_executor::makeClearanceRequirement(
      check_dist, effective_near_field, start_clearance, start_clearance_ok, 0.02,
      effective_near_floor);
  constexpr double kCorridorLateralTolerance = 0.20;
  const auto requiredAt = [&](const Eigen::Vector3d & pos, double arc) {
    double required = gate.requiredAt(arc);
    if (!planned_narrow) {
      return required;
    }
    double nearest_squared = std::numeric_limits<double>::infinity();
    const ReferenceSample * nearest = nullptr;
    for (const auto & sample : reference_samples) {
      const double squared = (sample.point - pos).head<2>().squaredNorm();
      if (squared < nearest_squared) {
        nearest_squared = squared;
        nearest = &sample;
      }
    }
    if (nearest != nullptr && nearest->near_narrow_corridor &&
        nearest_squared <= kCorridorLateralTolerance * kCorridorLateralTolerance) {
      required = std::min(required, narrow_passage_clearance_);
    }
    return required;
  };
  if (narrow_continuation) {
    RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "[MincoPlanner] Narrow-passage continuation active: source=%s start clearance %.3f m, "
      "reference min %.3f m, full requirement %.3f m, continuation floor %.3f m; "
      "command speed is capped by executor",
      planned_narrow ? (start_narrow ? "start+route" : "route") : "start",
      start_clearance, reference_min_clearance, gate.required, narrow_passage_clearance_);
  } else if (gate.nearFieldEnabled() && gate.near_required < gate.required) {
    RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "[MincoPlanner] Near-field exemption active: start clearance %.3f m, "
      "near requirement lowered to %.3f m (full requirement %.3f m); "
      "inside %.2f m of the start only 'not worse than now' is enforced",
      start_clearance, gate.near_required, gate.required, gate.near_field);
  }

  // 沿轨迹累积弧长：近场是按「离起点多远」划分的，不是按时间。
  double arc = 0.0;
  Eigen::Vector3d previous = start_pos;
  if (!checkPoint(previous, requiredAt(previous, arc))) {
    return false;
  }
  for (double t = sample_dt_; t <= dur; t += sample_dt_) {
    const Eigen::Vector3d pos = traj.getPos(t);
    arc += (pos - previous).norm();
    previous = pos;
    if (!checkPoint(pos, requiredAt(pos, arc))) {
      return false;
    }
  }
  const Eigen::Vector3d last = traj.getPos(dur);
  arc += (last - previous).norm();
  return checkPoint(last, requiredAt(last, arc));
}

double TrajectorySafetyChecker::getDistance(const Eigen::Vector3d & pos) const
{
  if (!dynamic_query_) {
    return 0.0;
  }
  double dist = 0.0;
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  const auto query = dynamic_query_->query(pos);
  dist = query.distance;
  grad = query.gradient;
  (void)grad;
  return query.ok ? dist : std::numeric_limits<double>::quiet_NaN();
}

bool TrajectorySafetyChecker::projectOutOfObstacle(Eigen::Vector3d & pos, double margin) const
{
  if (!dynamic_query_) {
    return false;
  }
  double esdf_dist = 0.0;
  Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
  const auto query = dynamic_query_->query(pos);
  if (!query.ok) {
    return false;
  }
  esdf_dist = query.distance;
  esdf_grad = query.gradient;
  if (esdf_dist < 0.0 && esdf_grad.norm() > 1e-6) {
    pos += (margin - esdf_dist) * esdf_grad.normalized();
    return true;
  }
  return false;
}

}  // namespace minco_planner
