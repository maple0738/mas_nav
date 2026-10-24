#pragma once

#include <cmath>
#include <cstdint>

namespace mas2027_decision {

// 二维点，只表达位置，不带朝向。
struct Vec2 {
  double x{0.0};
  double y{0.0};
};

// 两点直线距离（米）。
inline double distance(const Vec2& a, const Vec2& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

// 目标点：位置 + 到位朝向（弧度）。yaw 为 0 表示不指定朝向。
struct Waypoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 决策层每帧看到的世界状态。所有字段都带一个"未知"表示，避免用默认值误导决策。
struct Observation {
  double now_s{0.0};           // 本地时钟，秒（巡逻切换用，不是比赛时间）
  double game_time_s{-1.0};    // 比赛已进行时间，<0 表示未知（时间窗判断用）
  double hp_ratio{1.0};        // 血量比例 0..1，由 current_hp / max_hp 得到
  int bullets{0};              // 剩余允许发弹量
  double max_hp{200.0};        // 最大血量，裁判系统未知时的近似值
  double ally_base_hp{-1.0};   // 己方基地血量，<0 表示未知
  double ally_outpost_hp{-1.0};  // 己方前哨站血量，<0 表示未知
  bool enemy_outpost_active{false};  // 敌方前哨站是否还活着（是则去打）
  bool referee_valid{false};   // 裁判数据是否已接通；false 时其余字段来自 debug 注入
  Vec2 pos{};                  // 当前位姿，场坐标系（map 或示教系）
  bool pose_valid{false};      // 位姿是否可用（TF/里程计是否就绪）
};

// 决策层输出的意图。select() 每帧按优先级重新选一个，不是状态机里的"当前状态"。
// 优先级从高到低：RUSH_ENEMY_BASE > HEAL > DEFEND_BASE > HEAL(弹药不足)
//                 > ATTACK_OUTPOST > HELP_OUTPOST > PATROL
enum class Intent : uint8_t {
  IDLE,             // 未决策/初始化占位，不会产生目标点
  HEAL,             // 回血：血量低于 heal_enter，或弹药低于 low_bullet，去 heal 点
  DEFEND_BASE,      // 守家：己方基地刚被打，去 home 点
  ATTACK_OUTPOST,   // 打前哨站：敌方前哨站激活，去 outpost_attack 点
  HELP_OUTPOST,     // 支援前哨站：己方前哨站需要帮忙，去 outpost_help 点
  PATROL,           // 巡逻：默认行为，在 patrol_a / patrol_b 之间按时间切换
  RUSH_ENEMY_BASE,  // 末段冲锋：比赛最后阶段且血量低，直冲敌方基地
};

// 调试输出用，把意图转成可读字符串。
const char* toString(Intent intent);

// step() 的输出：valid=false 表示该意图不产生目标（目前只有 IDLE）。
struct Goal {
  bool valid{false};
  Waypoint wp{};
};

}  // namespace mas2027_decision
