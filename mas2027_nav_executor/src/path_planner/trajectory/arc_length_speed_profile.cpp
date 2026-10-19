#include "mas2027_nav_executor/path_planner/trajectory/arc_length_speed_profile.hpp"

#include <algorithm>
#include <cmath>

namespace mas2027_nav_executor {

bool buildArcLengthSpeedProfile(
  const std::vector<double> & lengths,
  const std::vector<double> & segment_speed_limits,
  double start_speed,
  bool stop_at_end,
  double max_acceleration,
  double brake_deceleration,
  std::vector<double> & node_speeds,
  std::vector<double> & segment_times)
{
  node_speeds.clear();
  segment_times.clear();
  const size_t count = lengths.size();
  if (count == 0 || count != segment_speed_limits.size() || !std::isfinite(start_speed) ||
    start_speed < 0.0 || !std::isfinite(max_acceleration) || max_acceleration <= 0.0 ||
    !std::isfinite(brake_deceleration) || brake_deceleration <= 0.0) return false;
  for (size_t i = 0; i < count; ++i) {
    if (!std::isfinite(lengths[i]) || lengths[i] < 0.0 ||
      !std::isfinite(segment_speed_limits[i]) || segment_speed_limits[i] <= 0.0) return false;
  }
  const double a_acc = max_acceleration;
  const double a_brake = brake_deceleration;

  std::vector<double> cap(count + 1U);
  cap[0] = segment_speed_limits[0];
  for (size_t i = 1; i < count; ++i) {
    cap[i] = std::min(segment_speed_limits[i - 1], segment_speed_limits[i]);
  }
  cap[count] = stop_at_end ? 0.0 : segment_speed_limits.back();
  // 反向刹车余量必须用真实刹车能力：底盘做不到 5 m/s²，用 5 编出来的余量
  // 在实车上等价于"最后一个控制周期才开始刹"。
  for (size_t i = count; i-- > 0;) {
    cap[i] = std::min(cap[i], std::sqrt(cap[i + 1] * cap[i + 1] +
      2.0 * a_brake * lengths[i]));
  }
  if (start_speed > cap[0] + 1e-6) return false;

  node_speeds.resize(count + 1U);
  segment_times.resize(count);
  node_speeds[0] = start_speed;
  for (size_t i = 0; i < count; ++i) {
    node_speeds[i + 1] = std::min(cap[i + 1], std::sqrt(
      node_speeds[i] * node_speeds[i] + 2.0 * a_acc * lengths[i]));
    if (lengths[i] <= 1e-9) {
      segment_times[i] = 0.1;
      continue;
    }
    const double v0 = node_speeds[i], v1 = node_speeds[i + 1];
    // 段内"先加速到 peak 再减速到 v1"的非对称解：
    //   (peak²−v0²)/(2·a_acc) + (peak²−v1²)/(2·a_brake) = L
    // 对称时退化为原来的 peak² = a·L + (v0²+v1²)/2。
    const double peak_squared = (2.0 * lengths[i] * a_acc * a_brake +
      a_brake * v0 * v0 + a_acc * v1 * v1) / (a_acc + a_brake);
    const double peak = std::min(segment_speed_limits[i],
      std::sqrt(std::max(0.0, peak_squared)));
    if (peak + 1e-6 < std::max(v0, v1)) return false;
    const double accelerate = std::max(0.0, (peak * peak - v0 * v0) / (2.0 * a_acc));
    const double decelerate = std::max(0.0, (peak * peak - v1 * v1) / (2.0 * a_brake));
    const double cruise = std::max(0.0, lengths[i] - accelerate - decelerate);
    segment_times[i] = std::max(0.1, (std::max(0.0, peak - v0)) / a_acc +
      (std::max(0.0, peak - v1)) / a_brake + cruise / std::max(peak, 1e-6));
    if (!std::isfinite(segment_times[i])) return false;
  }
  return true;
}

}  // namespace mas2027_nav_executor
