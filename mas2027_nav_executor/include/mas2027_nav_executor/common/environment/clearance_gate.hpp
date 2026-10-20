#ifndef MAS2027_NAV_EXECUTOR__COMMON__ENVIRONMENT__CLEARANCE_GATE_HPP_
#define MAS2027_NAV_EXECUTOR__COMMON__ENVIRONMENT__CLEARANCE_GATE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace mas2027_nav_executor {

/// 【唯一定义处】ESDF 逐帧抖动余量：发布前整条轨迹与执行前指令两道硬门共用此值，
/// 否则重现“规划放行、执行刹车”的假否决。0.05 曾把有效硬阈值压到车体半宽以下并导致撞墙。
inline constexpr double kEsdfJitterTolerance = 0.02;

/// ESDF 插值与日志格式化会制造“实测 0.259999、阈值 0.260000”的伪否决。
/// 该容差只用于浮点比较，不改变任何配置阈值；1 mm 远小于地图分辨率 5 cm。
inline constexpr double kClearanceComparisonTolerance = 0.001;

/// 完整净空要求 → 两道硬门实际使用的有效阈值。
inline double effectiveClearanceThreshold(double full_requirement)
{
  return std::max(0.0, full_requirement - kEsdfJitterTolerance);
}

/// 近场净空判据：检查都从机器人当前位置开始采样，若已停在比要求净空更近处（如靠墙停放），
/// “起点净空不足”会否决每条轨迹与指令，车一动不动；故对已占住的那段放宽：弧长 < near_field 只要求
/// 不比当前实测净空更差，之外仍须满足完整 required。slack 默认 0.02 m（介于 ESDF 抖动与地图格
/// 0.05 m 之间）；ESDF 满足 1-Lipschitz 保证该放宽最多只会丢掉 slack，不会放过真正撞障的轨迹。
struct ClearanceRequirement
{
  /// 近场之外必须满足的净空。
  double required{0.0};
  /// 近场之内必须满足的净空（由当前实测净空减 slack 推出）。
  double near_required{0.0};
  /// 近场弧长半径（约定取车体安全半径 collision_dist）；<= 0 表示关闭近场放宽，全程用 required。
  double near_field{0.0};

  /// 按「离起点多远」取当前采样点该满足的净空。
  double requiredAt(double arc_from_start) const
  {
    return (near_field > 1e-6 && arc_from_start < near_field) ? near_required : required;
  }

  bool nearFieldEnabled() const { return near_field > 1e-6; }
};

/// 指令净空的制动模型。减速度和延迟须来自底盘实测；deceleration<=0 关闭速度项。
struct ClearanceBrakingParams
{
  double deceleration{0.0};
  double reaction_time{0.0};

  bool valid() const
  {
    return std::isfinite(deceleration) && deceleration > 0.0 &&
      std::isfinite(reaction_time) && reaction_time >= 0.0;
  }
};

inline double brakingClearance(double speed, const ClearanceBrakingParams & params)
{
  if (!params.valid() || !std::isfinite(speed)) return 0.0;
  const double v = std::abs(speed);
  return v * params.reaction_time + v * v / (2.0 * params.deceleration);
}

/// ESDF gradient points away from the nearest obstacle. Only motion toward it
/// consumes stopping clearance. A flat gradient has no defined approach
/// direction; the command gate still checks ESDF at every point of its short
/// horizon, where a directional gradient can become available.
inline double obstacleApproachSpeed(
  double vx, double vy, double gradient_x, double gradient_y)
{
  const double speed = std::hypot(vx, vy);
  const double gradient_norm = std::hypot(gradient_x, gradient_y);
  if (!std::isfinite(speed) || !std::isfinite(gradient_norm)) {
    return speed;
  }
  if (gradient_norm < 1e-3) {
    return 0.0;
  }
  return std::max(0.0, -(vx * gradient_x + vy * gradient_y) / gradient_norm);
}

/// Remove only the excess velocity toward the nearest obstacle. This keeps
/// motion along a wall when a small lateral correction reaches its braking
/// limit. Returns false when the gradient cannot define a safe direction.
inline bool limitObstacleApproachVelocity(
  double & vx, double & vy, double gradient_x, double gradient_y,
  double allowed_toward)
{
  const double gradient_norm = std::hypot(gradient_x, gradient_y);
  if (!std::isfinite(vx) || !std::isfinite(vy) ||
    !std::isfinite(gradient_norm) || gradient_norm < 1e-3 ||
    !std::isfinite(allowed_toward) || allowed_toward < 0.0) {
    return false;
  }
  const double unit_x = gradient_x / gradient_norm;
  const double unit_y = gradient_y / gradient_norm;
  const double toward = std::max(0.0, -(vx * unit_x + vy * unit_y));
  if (toward > allowed_toward) {
    const double correction = toward - allowed_toward;
    vx += correction * unit_x;
    vy += correction * unit_y;
  }
  return true;
}

/// 从可用 ESDF 净空反求允许速度；非法参数关闭封顶，非法净空交由硬门拒绝。
inline double clearanceSpeedLimit(double clearance, const ClearanceBrakingParams & params)
{
  if (!params.valid() || !std::isfinite(clearance)) {
    return std::numeric_limits<double>::infinity();
  }
  const double a_tau = params.deceleration * params.reaction_time;
  return std::sqrt(a_tau * a_tau + 2.0 * params.deceleration *
    std::max(0.0, clearance)) - a_tau;
}

/// 组装判据。current_clearance_ok 为 false（拿不到当前净空）时不放宽：全程按 required 判。
/// `minimum_near_required` 是窄道续行的硬地板；只有调用方已经确认机器人当前就在
/// 窄道内时才应传入，避免把普通近场豁免变成全局降阈值。
inline ClearanceRequirement makeClearanceRequirement(
  double required, double near_field, double current_clearance, bool current_clearance_ok,
  double slack = 0.02, double minimum_near_required = 0.0)
{
  ClearanceRequirement gate;
  gate.required = std::max(0.0, required);
  gate.near_field = std::max(0.0, near_field);
  const double safe_slack = std::max(0.0, slack);
  if (current_clearance_ok && std::isfinite(current_clearance)) {
    const double lowered = std::max({0.0, current_clearance - safe_slack,
      std::max(0.0, minimum_near_required)});
    gate.near_required = std::min(gate.required, lowered);
  } else {
    gate.near_required = gate.required;
  }
  return gate;
}

}  // namespace mas2027_nav_executor

#endif  // MAS2027_NAV_EXECUTOR__COMMON__ENVIRONMENT__CLEARANCE_GATE_HPP_
