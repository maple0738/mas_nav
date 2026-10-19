#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "mas2027_nav_executor/common/control/goal_approach.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/arc_length_speed_profile.hpp"

namespace mas2027_nav_executor {
namespace {

constexpr double kEps = 1e-6;

TEST(GoalApproach, SpeedEnvelopeFollowsMeasuredBraking)
{
  GoalApproachParams params;
  params.deceleration = 1.2;
  params.stop_margin = 0.30;
  params.min_speed = 0.15;
  params.reaction_time = 0.20;
  ASSERT_TRUE(params.valid());

  // v·τ + v²/(2a) = 剩余距离 - stop_margin
  const auto envelope = [&](double usable) {
      const double tau = params.deceleration * params.reaction_time;
      return std::sqrt(2.0 * params.deceleration * usable + tau * tau) - tau;
    };
  EXPECT_NEAR(goalApproachSpeedLimit(1.0, params), envelope(0.70), 1e-9);
  EXPECT_NEAR(goalApproachSpeedLimit(0.5, params), envelope(0.20), 1e-9);
  // 余量以内只允许爬行
  EXPECT_NEAR(goalApproachSpeedLimit(0.30, params), params.min_speed, 1e-9);
  EXPECT_NEAR(goalApproachSpeedLimit(0.0, params), params.min_speed, 1e-9);
  // 反应时间让包络更保守：不含 τ 的公式会给出更大的允许速度
  EXPECT_LT(goalApproachSpeedLimit(1.0, params), std::sqrt(2.0 * 1.2 * 0.70));
  // 目标点周围 0.3 m 需要的是爬行速度，不是 1 m/s 以上
  EXPECT_LT(goalApproachSpeedLimit(0.3, params), 0.2);
}

TEST(GoalApproach, InvalidParamsFailOpen)
{
  GoalApproachParams disabled;
  disabled.deceleration = 0.0;
  EXPECT_FALSE(disabled.valid());
  EXPECT_TRUE(std::isinf(goalApproachSpeedLimit(0.2, disabled)));

  GoalApproachParams nan_margin;
  nan_margin.stop_margin = std::nan("");
  EXPECT_FALSE(nan_margin.valid());
  EXPECT_TRUE(std::isinf(goalApproachSpeedLimit(0.2, nan_margin)));
  EXPECT_TRUE(std::isinf(goalApproachSpeedLimit(std::nan(""), GoalApproachParams{})));
}

TEST(GoalApproach, ActiveFlagIsMonotoneInSpeed)
{
  GoalApproachParams params;
  const double limit = goalApproachSpeedLimit(0.5, params);
  EXPECT_FALSE(goalApproachActive(0.5, limit - 0.01, params));
  EXPECT_TRUE(goalApproachActive(0.5, limit + 0.01, params));
  EXPECT_FALSE(goalApproachActive(0.5, std::nan(""), params));
}

TEST(ArcLengthSpeedProfile, SymmetricOverloadKeepsTheOldTrapezoid)
{
  // 8 段 × 0.5 m、限速 2 m/s、从 0 起步到停：对称 4 m/s² 下与手算一致。
  std::vector<double> lengths(8, 0.5);
  std::vector<double> limits(8, 2.0);
  std::vector<double> speeds, times;
  ASSERT_TRUE(buildArcLengthSpeedProfile(lengths, limits, 0.0, true, 4.0,
    speeds, times));
  ASSERT_EQ(speeds.size(), 9U);
  ASSERT_EQ(times.size(), 8U);
  EXPECT_NEAR(speeds.front(), 0.0, kEps);
  EXPECT_NEAR(speeds.back(), 0.0, kEps);
  for (size_t i = 0; i < speeds.size(); ++i) {
    EXPECT_LE(speeds[i], 2.0 + kEps);
  }
  // 加速段 v² ≤ 2·a·s，刹车段同样受 2·a·(剩余距离) 约束
  double travelled = 0.0;
  for (size_t i = 0; i < lengths.size(); ++i) {
    travelled += lengths[i];
    EXPECT_LE(speeds[i] * speeds[i], 2.0 * 4.0 * travelled + kEps);
    EXPECT_LE(speeds[i] * speeds[i], 2.0 * 4.0 * (8.0 - travelled) + kEps);
  }
  for (const double t : times) {
    EXPECT_GE(t, 0.1);
  }
}

TEST(ArcLengthSpeedProfile, AsymmetricBrakingShortensTheStoppingEnvelope)
{
  // 同一条 4 m 直线、起步 2.5 m/s：刹车能力 1.0 m/s² 时必须在前 3.1 m 就
  // 开始收速，而 5 m/s² 的对称结果会一路顶着 3 m/s 限速到最后一米。
  std::vector<double> lengths(16, 0.25);   // 4 m
  std::vector<double> limits(16, 3.0);
  std::vector<double> speeds, times;
  // 3 m/s 在 4 m 内停不住（需要 4.5 m），故起点取 2.5 m/s 保证可行。
  ASSERT_TRUE(buildArcLengthSpeedProfile(lengths, limits, 2.5, true, 5.0, 1.0,
    speeds, times));

  double travelled = 0.0;
  for (size_t i = 0; i <= lengths.size(); ++i) {
    const double remaining = 4.0 - travelled;
    // 每个节点都满足 v² ≤ 2·a_brake·剩余距离（含段末点 0）
    EXPECT_LE(speeds[i] * speeds[i], 2.0 * 1.0 * remaining + 1e-3)
      << "node " << i << " speed " << speeds[i] << " remaining " << remaining;
    if (i < lengths.size()) {
      travelled += lengths[i];
    }
  }
  EXPECT_NEAR(speeds.back(), 0.0, kEps);

  // 同样输入用 5 m/s² 刹车时，速度衰减明显更晚：节点速度不低对称结果。
  std::vector<double> symmetric_speeds, symmetric_times;
  ASSERT_TRUE(buildArcLengthSpeedProfile(lengths, limits, 2.5, true, 5.0, 5.0,
    symmetric_speeds, symmetric_times));
  EXPECT_GT(symmetric_speeds[lengths.size() / 2], speeds[lengths.size() / 2]);
}

TEST(ArcLengthSpeedProfile, RejectsUnreachableStartSpeed)
{
  std::vector<double> lengths{0.2};
  std::vector<double> limits{1.0};
  std::vector<double> speeds, times;
  // 0.2 m 内从 3 m/s 停住需要 22.5 m/s²；刹车能力 1.0 时无解。
  EXPECT_FALSE(buildArcLengthSpeedProfile(lengths, limits, 3.0, true, 5.0, 1.0,
    speeds, times));
}

TEST(ArcLengthSpeedProfile, RejectsNonPositiveBrakeDeceleration)
{
  std::vector<double> lengths{1.0};
  std::vector<double> limits{2.0};
  std::vector<double> speeds, times;
  EXPECT_FALSE(buildArcLengthSpeedProfile(lengths, limits, 0.0, true, 5.0, 0.0,
    speeds, times));
  EXPECT_FALSE(buildArcLengthSpeedProfile(lengths, limits, 0.0, true, 0.0, 1.0,
    speeds, times));
}

}  // namespace
}  // namespace mas2027_nav_executor
