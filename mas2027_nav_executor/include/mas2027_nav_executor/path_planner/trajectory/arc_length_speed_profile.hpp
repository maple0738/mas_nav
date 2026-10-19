#pragma once

#include <vector>

namespace mas2027_nav_executor {

// Arc-length v² envelope for the MINCO time seed. The optimized polynomial
// remains the authoritative executable trajectory and is checked afterwards.
//
// `max_acceleration` 只用于加速段与段内峰值的加速部分；`brake_deceleration`
// 用于反向刹车余量和段内减速部分。两者必须分开：底盘实测只能提供约
// 1.4–2.6 m/s² 的减速度（20261018_194203 bag 标定），而 planner 的
// max_acceleration 是 5.0，混用会让种子贴着 5 m/s² 编刹车余量，目标点必然
// 过冲（实测过冲 0.6–1.1 m 后往复震荡）。
bool buildArcLengthSpeedProfile(
  const std::vector<double> & lengths,
  const std::vector<double> & segment_speed_limits,
  double start_speed,
  bool stop_at_end,
  double max_acceleration,
  double brake_deceleration,
  std::vector<double> & node_speeds,
  std::vector<double> & segment_times);

/// 兼容重载：无法区分加减速时退化为对称能力（刹车减速度 = max_acceleration）。
inline bool buildArcLengthSpeedProfile(
  const std::vector<double> & lengths,
  const std::vector<double> & segment_speed_limits,
  double start_speed,
  bool stop_at_end,
  double max_acceleration,
  std::vector<double> & node_speeds,
  std::vector<double> & segment_times)
{
  return buildArcLengthSpeedProfile(lengths, segment_speed_limits, start_speed, stop_at_end,
    max_acceleration, max_acceleration, node_speeds, segment_times);
}

}  // namespace mas2027_nav_executor
