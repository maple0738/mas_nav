#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace mas2027_nav_executor {

/// 目标接近减速带（规划种子与 MPC 参考共用）。
///
/// 物理依据：v² ≤ 2·a·(d − stop_margin)，d 是沿已提交路线到局部终点的剩余弧长。
/// 2026-10-18 的 `20261018_194203` 实车 bag 标定：底盘实际只能提供约
/// 1.4–2.6 m/s² 的减速度（含响应延迟时等效 0.6–1.0 m/s²），而规划种子与 MPC
/// 原先把刹车按 `max_acceleration = 5.0` 编余量，于是参考线在离目标 0.3 m
/// 处仍有约 1.1 m/s，实测过冲 0.6–1.1 m 并往复震荡，9 个目标只有 5 个报到达。
/// 同一份错误模型也让"0.26 m 净空就能停住"的假设失效，是撞墙的同一根因。
///
/// `min_speed` 是爬行下限：参考速度整段降到 0 会让 MPC 在目标前僵住；
/// 真正的停车由路线末点（规划侧 stop_at_end）完成。
struct GoalApproachParams
{
  /// 用于封顶的减速度（m/s²）。取实测值，不要用底盘标称值。
  double deceleration{1.2};
  /// 到达终点前保留的余量（m）：剩余弧长小于它时只允许 min_speed 爬行。
  /// 0.30 m 让"爬行段"吸收控制周期(50 ms)+重规划周期(0.2 s)的延迟，
  /// 过冲量与 v_creep·τ 同量级（~3 cm）而不是 v²/(2a)。
  double stop_margin{0.30};
  /// 允许的最小参考速度（m/s）。必须小于任务层的到达判据 0.3 m/s。
  double min_speed{0.15};
  /// 反应时间（s）：命令到实际减速的延迟（控制周期 + 重规划周期）。
  /// 包络按"先等 reaction_time 再以 deceleration 刹停"求解。
  double reaction_time{0.20};

  bool valid() const
  {
    return std::isfinite(deceleration) && deceleration > 1e-3 &&
           std::isfinite(stop_margin) && stop_margin >= 0.0 &&
           std::isfinite(min_speed) && min_speed >= 0.0 &&
           std::isfinite(reaction_time) && reaction_time >= 0.0;
  }
};

/// 剩余弧长 `remaining` 处允许的最大平移速度（m/s）。
/// 参数非法或剩余距离非有限时返回 +inf，等价于不封顶（fail open，因为
/// 这里只是减速包络，不是安全门；安全门仍然独立生效）。
inline double goalApproachSpeedLimit(double remaining, const GoalApproachParams & params)
{
  if (!params.valid() || !std::isfinite(remaining)) {
    return std::numeric_limits<double>::infinity();
  }
  const double usable = std::max(0.0, remaining - std::max(0.0, params.stop_margin));
  const double a = params.deceleration;
  // 反应时间内仍以 v 前进，之后才以 a 刹停：
  //   v·τ + v²/(2a) ≤ usable  ⇒  v ≤ sqrt(2a·usable + (aτ)²) − aτ
  const double tau_term = a * std::max(0.0, params.reaction_time);
  const double cap = std::sqrt(2.0 * a * usable + tau_term * tau_term) - tau_term;
  return std::max(params.min_speed, cap);
}

/// 该剩余距离下减速带是否真的在起作用（供诊断/日志使用）。
inline bool goalApproachActive(double remaining, double speed, const GoalApproachParams & params)
{
  if (!params.valid() || !std::isfinite(remaining) || !std::isfinite(speed)) {
    return false;
  }
  return speed > goalApproachSpeedLimit(remaining, params) + 1e-6;
}

}  // namespace mas2027_nav_executor
