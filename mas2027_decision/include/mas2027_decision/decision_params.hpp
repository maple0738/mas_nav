#pragma once

#include "mas2027_decision/point_table.hpp"
#include "mas2027_decision/types.hpp"

namespace mas2027_decision {

// 决策参数。阈值全部是"可现场调"的量，不要写死在 core 里。
struct DecisionParams {
  // ---- 时间窗（相对比赛开始，秒）----
  // 比赛时间未知时：回血窗口视为一直打开（允许回血），末段冲锋不会触发。
  double heal_start_s{60.0};   // 开局多少秒内不回血
  double end_rush_s{300.0};    // 超过它进入末段，低血改为冲锋

  // ---- 血量比例（0..1）----
  double heal_enter{0.30};     // 低于它开始回血
  double heal_exit{0.60};      // 回到它才退出回血（必须高于 heal_enter）
  double end_rush_hp{0.50};    // 末段冲锋的血量门槛

  // ---- 弹药 ----
  int low_bullet{20};          // 剩余弹药低于它时去回血点补弹

  // ---- 基地受击 ----
  double base_attack_drop{300.0};  // 相对基准血量一次性掉这么多算被攻击
  double base_clear_s{5.0};        // 事件保持秒数，到点后自动解除守家

  // ---- 巡逻 ----
  double patrol_period_s{20.0};    // 在 patrol_a / patrol_b 各停留的秒数

  // ---- 目标去重 ----
  double goal_dedupe_m{0.30};      // 目标移动超过该距离才重发
  double goal_republish_s{3.0};    // 兜底重发周期，秒

  PointTable points{};             // 点表（坐标含义由 goal_frame 决定）
};

}  // namespace mas2027_decision
