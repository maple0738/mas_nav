#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "utils/header/eigen_alias.hpp"

namespace mas2027_nav_executor {

inline int selectReplanContinuityPrefix(
  const std::vector<Eigen::Vector3d> & sparse_path,
  const super_utils::vec_Vec3f & shifted_waypoints,
  const super_utils::VecDf & shifted_durations,
  double continuity_time_s)
{
  const int piece_count = static_cast<int>(sparse_path.size()) - 1;
  if (!(continuity_time_s > 0.0) || piece_count <= 1) return 0;

  const int old_point_count = std::max(0, static_cast<int>(shifted_waypoints.size()) - 2);
  const int max_prefix_count = std::min(piece_count - 1, old_point_count);
  int prefix_count = 0;
  double elapsed_s = 0.0;
  for (int index = 0; index < max_prefix_count; ++index) {
    const double duration = index < static_cast<int>(shifted_durations.size()) ?
      shifted_durations[index] : 0.1;
    const double segment_s = std::isfinite(duration) && duration > 0.0 ? duration : 0.1;
    if (elapsed_s + segment_s > continuity_time_s + 1e-9) break;
    elapsed_s += segment_s;
    prefix_count = index + 1;
  }

  constexpr double kMaxPrefixJump = 0.6;
  while (prefix_count > 0) {
    const auto & old_end = shifted_waypoints[static_cast<size_t>(prefix_count)];
    const auto & new_next = sparse_path[static_cast<size_t>(prefix_count + 1)];
    if ((old_end - new_next).head<2>().norm() <= kMaxPrefixJump) break;
    --prefix_count;
  }
  return prefix_count;
}

}
