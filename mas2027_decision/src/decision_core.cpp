#include "mas2027_decision/decision_core.hpp"

#include <algorithm>
#include <cmath>

namespace mas2027_decision {

// 意图转可读字符串，供日志和 /debug/decision_intent 使用。
const char* toString(Intent intent) {
  switch (intent) {
    case Intent::IDLE:
      return "IDLE";
    case Intent::HEAL:
      return "HEAL";
    case Intent::DEFEND_BASE:
      return "DEFEND_BASE";
    case Intent::ATTACK_OUTPOST:
      return "ATTACK_OUTPOST";
    case Intent::HELP_OUTPOST:
      return "HELP_OUTPOST";
    case Intent::PATROL:
      return "PATROL";
    case Intent::RUSH_ENEMY_BASE:
      return "RUSH_ENEMY_BASE";
  }
  return "UNKNOWN";
}

// 清空所有跨帧状态。参数和点表不受影响。
void DecisionCore::reset() {
  heal_latched_ = false;
  base_under_attack_ = false;
  base_attack_hold_s_ = 0.0;
  base_hp_ref_ = -1.0;
  ally_outpost_hp_ = -1.0;
  has_last_tick_ = false;
}

// 每帧更新跨帧状态：
//   - 缓存己方前哨站血量（数据可能中断，保留最近一次有效值）；
//   - 维护基地血量基准，检测"一次性掉血超过 base_attack_drop"；
//   - 受击事件保持 base_clear_s 秒后自动解除。
// obs: 本帧观测；dt 由 now_s 与上一帧的差求得，首帧记 0。
void DecisionCore::updateMemo(const Observation& obs) {
  double dt = 0.0;
  if (has_last_tick_) {
    dt = std::max(0.0, obs.now_s - last_tick_s_);
  }
  last_tick_s_ = obs.now_s;

  if (obs.ally_outpost_hp >= 0.0) {
    ally_outpost_hp_ = obs.ally_outpost_hp;
  }

  if (obs.ally_base_hp < 0.0) {
    return;
  }

  if (base_hp_ref_ < 0.0) {
    base_hp_ref_ = obs.ally_base_hp;
    return;
  }

  if (obs.ally_base_hp < base_hp_ref_ - params_.base_attack_drop) {
    base_under_attack_ = true;
    base_attack_hold_s_ = 0.0;
    base_hp_ref_ = obs.ally_base_hp;
    return;
  }

  if (base_under_attack_) {
    base_attack_hold_s_ += dt;
    if (base_attack_hold_s_ >= params_.base_clear_s) {
      base_under_attack_ = false;
      base_attack_hold_s_ = 0.0;
    }
  } else {
    // 基准血量只缓慢跟随，避免一次掉血把基准带偏后漏判后续攻击。
    base_hp_ref_ = std::max(obs.ally_base_hp, base_hp_ref_ - 10.0 * dt);
  }
}

// 按优先级选择意图，每帧从零判断：
//   1. 末段低血 -> 冲锋敌方基地
//   2. 回血 latch（0.30 进入 / 0.60 退出）
//   3. 基地受击事件 -> 守家
//   4. 弹药不足 -> 回血点
//   5. 敌方前哨站激活 -> 打前哨站
//   6. 己方前哨站需要支援 -> 支援
//   7. 默认巡逻
// obs: 本帧世界状态（比赛时间、血量、弹药、基地血量）。
Intent DecisionCore::select(const Observation& obs) {
  updateMemo(obs);
  has_last_tick_ = true;

  const bool time_known = obs.game_time_s >= 0.0;
  const bool in_heal_window =
      !time_known || (obs.game_time_s >= params_.heal_start_s &&
                      obs.game_time_s < params_.end_rush_s);

  // 1) 末段低血：直冲敌方基地
  if (time_known && obs.game_time_s >= params_.end_rush_s &&
      obs.hp_ratio < params_.end_rush_hp) {
    heal_latched_ = false;
    return Intent::RUSH_ENEMY_BASE;
  }

  // 2) 回血（带回差，避免在阈值上抖动）
  if (!heal_latched_ && in_heal_window && obs.hp_ratio < params_.heal_enter) {
    heal_latched_ = true;
  }
  if (heal_latched_ && obs.hp_ratio >= params_.heal_exit) {
    heal_latched_ = false;
  }
  if (heal_latched_) {
    return Intent::HEAL;
  }

  // 3) 基地被攻击：事件保持一段时间
  if (base_under_attack_) {
    return Intent::DEFEND_BASE;
  }

  // 4) 弹药不足
  if (obs.bullets < params_.low_bullet) {
    return Intent::HEAL;
  }

  // 5) 敌方前哨站
  if (obs.enemy_outpost_active) {
    return Intent::ATTACK_OUTPOST;
  }

  // 6) 己方前哨站需要支援
  if (ally_outpost_hp_ > 10.0) {
    return Intent::HELP_OUTPOST;
  }

  // 7) 默认巡逻
  return Intent::PATROL;
}

// 意图 -> 目标点。巡逻按 now_s 在 patrol_a / patrol_b 之间轮换。
// intent: 要翻译的意图。
// obs: 只用 now_s；其余字段当前未使用。
Goal DecisionCore::step(Intent intent, const Observation& obs) const {
  const PointTable& pts = params_.points;
  switch (intent) {
    case Intent::HEAL:
      return {true, pts.heal};
    case Intent::DEFEND_BASE:
      return {true, pts.home};
    case Intent::ATTACK_OUTPOST:
      return {true, pts.outpost_attack};
    case Intent::HELP_OUTPOST:
      return {true, pts.outpost_help};
    case Intent::RUSH_ENEMY_BASE:
      return {true, pts.enemy_base};
    case Intent::PATROL: {
      const double period = std::max(1.0, params_.patrol_period_s);
      const bool first =
          std::fmod(std::max(0.0, obs.now_s), 2.0 * period) < period;
      return {true, first ? pts.patrol_a : pts.patrol_b};
    }
    case Intent::IDLE:
      break;
  }
  return {};
}

}  // namespace mas2027_decision
