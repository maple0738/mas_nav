#include <gtest/gtest.h>

#include "mas2027_decision/decision_core.hpp"

namespace mas2027_decision {
namespace {

DecisionParams makeParams() {
  DecisionParams p;
  p.heal_start_s = 60.0;
  p.end_rush_s = 300.0;
  p.heal_enter = 0.30;
  p.heal_exit = 0.60;
  p.end_rush_hp = 0.50;
  p.low_bullet = 20;
  p.base_attack_drop = 300.0;
  p.base_clear_s = 5.0;
  p.patrol_period_s = 20.0;
  p.points.heal = {1.0, 1.0, 0.0};
  p.points.home = {2.0, 2.0, 0.0};
  p.points.outpost_attack = {3.0, 3.0, 0.0};
  p.points.outpost_help = {4.0, 4.0, 0.0};
  p.points.enemy_base = {5.0, 5.0, 0.0};
  p.points.patrol_a = {6.0, 6.0, 0.0};
  p.points.patrol_b = {7.0, 7.0, 0.0};
  p.points.loaded = true;
  return p;
}

Observation makeObs(double now_s) {
  Observation obs;
  obs.now_s = now_s;
  obs.game_time_s = 120.0;
  obs.hp_ratio = 1.0;
  obs.bullets = 100;
  obs.ally_base_hp = 5000.0;
  obs.ally_outpost_hp = -1.0;
  obs.enemy_outpost_active = false;
  obs.pose_valid = true;
  return obs;
}

// 回血回差：0.20 进入，0.45（介于 enter/exit 之间）保持，0.65 才退出。
TEST(DecisionCore, HealLatchHasHysteresis) {
  DecisionCore core;
  core.setParams(makeParams());

  auto obs = makeObs(100.0);
  obs.hp_ratio = 0.20;
  EXPECT_EQ(core.select(obs), Intent::HEAL);
  EXPECT_TRUE(core.healLatched());

  obs.now_s = 100.1;
  obs.hp_ratio = 0.45;  // 介于 enter 与 exit 之间，应保持回血
  EXPECT_EQ(core.select(obs), Intent::HEAL);

  obs.now_s = 100.2;
  obs.hp_ratio = 0.65;  // 超过 exit 才退出
  EXPECT_EQ(core.select(obs), Intent::PATROL);
  EXPECT_FALSE(core.healLatched());
}

// 开局 60 s 内即使血量很低也不回血，避免一开始就缩回基地。
TEST(DecisionCore, NoHealBeforeHealWindow) {
  DecisionCore core;
  core.setParams(makeParams());

  auto obs = makeObs(10.0);
  obs.game_time_s = 10.0;  // 开局 10 s，仍在回血窗口之外
  obs.hp_ratio = 0.20;
  EXPECT_EQ(core.select(obs), Intent::PATROL);
}

// 末段（>= end_rush_s）低血优先级最高，直接冲锋；血量恢复后回到巡逻。
TEST(DecisionCore, EndRushOverridesEverything) {
  DecisionCore core;
  core.setParams(makeParams());

  auto obs = makeObs(310.0);
  obs.game_time_s = 310.0;
  obs.hp_ratio = 0.40;
  EXPECT_EQ(core.select(obs), Intent::RUSH_ENEMY_BASE);

  obs.hp_ratio = 0.80;
  EXPECT_EQ(core.select(obs), Intent::PATROL);
}

// 基地一次掉 400 血触发守家，保持 5 s 后自动解除回到巡逻。
TEST(DecisionCore, BaseAttackHoldThenClear) {
  DecisionCore core;
  core.setParams(makeParams());

  auto obs = makeObs(0.0);
  EXPECT_EQ(core.select(obs), Intent::PATROL);

  // 基地一次性掉 400 血
  obs.now_s = 1.0;
  obs.ally_base_hp = 4600.0;
  EXPECT_EQ(core.select(obs), Intent::DEFEND_BASE);
  EXPECT_TRUE(core.baseUnderAttack());

  // 保持期内仍在守家
  obs.now_s = 3.0;
  EXPECT_EQ(core.select(obs), Intent::DEFEND_BASE);

  // 事件保持结束后回到巡逻
  obs.now_s = 8.0;
  EXPECT_EQ(core.select(obs), Intent::PATROL);
  EXPECT_FALSE(core.baseUnderAttack());
}

// 巡逻按 now_s 在 patrol_a / patrol_b 之间轮换，且都能生成有效目标。
TEST(DecisionCore, PatrolAlternatesAndProducesGoals) {
  DecisionCore core;
  core.setParams(makeParams());

  auto obs = makeObs(0.0);
  Intent intent = core.select(obs);
  ASSERT_EQ(intent, Intent::PATROL);
  Goal a = core.step(intent, obs);

  obs.now_s = 25.0;
  Goal b = core.step(intent, obs);

  EXPECT_TRUE(a.valid);
  EXPECT_TRUE(b.valid);
  EXPECT_NEAR(a.wp.x, 6.0, 1e-9);
  EXPECT_NEAR(b.wp.x, 7.0, 1e-9);
}

// 弹药低于 low_bullet 时去回血点补弹。
TEST(DecisionCore, LowBulletsGoToHealPoint) {
  DecisionCore core;
  core.setParams(makeParams());

  auto obs = makeObs(120.0);
  obs.bullets = 5;
  const Intent intent = core.select(obs);
  EXPECT_EQ(intent, Intent::HEAL);
  const Goal goal = core.step(intent, obs);
  EXPECT_TRUE(goal.valid);
  EXPECT_NEAR(goal.wp.x, 1.0, 1e-9);
}

}  // namespace
}  // namespace mas2027_decision
