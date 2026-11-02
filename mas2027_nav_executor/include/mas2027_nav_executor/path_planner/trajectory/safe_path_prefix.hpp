#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "data_structure/base/trajectory.h"

namespace mas2027_nav_executor {

// A bounded, rest-to-rest prefix along the already checked seed. It is only a
// candidate: the caller must run the complete trajectory safety validation.
inline geometry_utils::Trajectory stoppedPathPrefix(
    const std::vector<Eigen::Vector3d> & path, double max_length, double max_speed,
    double max_acceleration, std::vector<Eigen::Vector3d> & prefix) {
  geometry_utils::Trajectory result;
  prefix.clear();
  if (path.size() < 2 || !std::isfinite(max_length) || !std::isfinite(max_speed) ||
      !std::isfinite(max_acceleration) || max_length <= 0 || max_speed <= 0 ||
      max_acceleration <= 0) return result;
  for (const auto & p : path) if (!p.allFinite()) return result;
  prefix.push_back(path.front());
  double remaining = max_length;
  for (size_t i = 1; i < path.size() && remaining > 1e-6; ++i) {
    const Eigen::Vector3d delta = path[i] - prefix.back();
    const double length = std::min(delta.norm(), remaining);
    if (length < 1e-6) continue;
    const Eigen::Vector3d end = prefix.back() + delta.normalized()*length;
    // Quintic smoothstep has max ds/du=1.875 and max |d2s/du2|=10/sqrt(3).
    const double duration = std::max({0.4, 1.875*length/max_speed,
      std::sqrt((10.0/std::sqrt(3.0))*length/max_acceleration)});
    Eigen::MatrixXd c = Eigen::MatrixXd::Zero(3,6);
    const Eigen::Vector3d displacement = end-prefix.back();
    c.col(0) = 6.0*displacement/std::pow(duration,5);
    c.col(1) = -15.0*displacement/std::pow(duration,4);
    c.col(2) = 10.0*displacement/std::pow(duration,3);
    c.col(5) = prefix.back();
    result.emplace_back(duration,c);
    prefix.push_back(end);
    remaining -= length;
  }
  return result;
}

}  // namespace mas2027_nav_executor
