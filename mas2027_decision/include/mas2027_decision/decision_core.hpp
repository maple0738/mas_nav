#pragma once

#include "mas2027_decision/decision_params.hpp"
#include "mas2027_decision/types.hpp"

namespace mas2027_decision {

// 纯逻辑决策核心：不依赖 ROS，可直接单元测试。
//
// 设计要点：select() 每帧从零判断，不保存"当前状态"；只有各意图自己的
// latch/hysteresis 跨帧保留。因此任务中断（进攻途中回血、巡逻途中回防）
// 不需要写任何状态转移，优先级一变就会切过去。
class DecisionCore {
 public:
  // 设置/更新参数（阈值 + 点表）。每帧调用也安全，内部只做赋值。
  // params: 决策阈值和点表。注意 points.loaded 不在这里校验，
  //         "点表是否可用"由节点侧的 points_ready_ 负责。
  void setParams(const DecisionParams& params) { params_ = params; }

  // 核心入口：根据当前观测选出意图。每帧从零判断，不读上一帧的意图，
  // 所以任务中断（进攻途中回血、巡逻途中回防）不需要写状态转移。
  // obs: 本帧世界状态（比赛时间、血量、弹药、基地血量、位姿）。
  // 返回: 本帧应执行的 Intent。
  Intent select(const Observation& obs);

  // 把意图翻译成导航目标点。
  // intent: select() 的返回值。
  // obs: 只用 now_s 决定巡逻点在 a/b 之间切换；其余字段当前未使用。
  // 返回: Goal；valid=false 表示该意图不产生目标（目前只有 IDLE）。
  Goal step(Intent intent, const Observation& obs) const;

  // 调试用：是否处于回血 latch 中。
  // true = 已经低于 heal_enter 开始回血，还没回到 heal_exit。
  bool healLatched() const { return heal_latched_; }

  // 调试用：基地受击事件是否仍在保持期内。
  // true = 检测到基地掉血后的 base_clear_s 计时还没走完。
  bool baseUnderAttack() const { return base_under_attack_; }

  // 清空所有跨帧状态（回血 latch、基地事件、基准血量、计时）。
  // 换场、重开比赛或重新加载点表后调用。
  void reset();

 private:
  // 每帧更新跨帧状态。回血 latch 不在这里（在 select 里），
  // 这里只处理：己方前哨站血量缓存、基地血量基准、基地受击事件的保持计时。
  // obs: 本帧观测；用 now_s 计算与上一帧的时间差 dt。
  void updateMemo(const Observation& obs);

  DecisionParams params_{};         // 当前参数快照（阈值 + 点表）
  bool heal_latched_{false};        // 回血回差：true 表示正在回血，直到 hp >= heal_exit
  bool base_under_attack_{false};   // 基地受击事件是否保持中
  double base_attack_hold_s_{0.0};  // 事件已保持的秒数，达到 base_clear_s 后解除
  double base_hp_ref_{-1.0};        // 基地血量基准，<0 表示还没收到有效值
  double ally_outpost_hp_{-1.0};    // 己方前哨站血量缓存，<0 表示未知
  double last_tick_s_{0.0};         // 上一帧的本地时间，用来算 dt
  bool has_last_tick_{false};       // 是否已经有上一帧（首帧 dt 记为 0）
};

}  // namespace mas2027_decision
