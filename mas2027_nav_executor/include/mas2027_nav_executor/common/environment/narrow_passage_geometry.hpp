#pragma once

#include <cmath>

#include <Eigen/Core>

#include "rog_map/map_query_interface.hpp"

namespace mas2027_nav_executor {

// A low ESDF value alone can mean one wall. Probe along both normals of the
// route: an opposing obstacle must make the distance fall on each side.
inline bool hasOpposingSideObstacles(
  const rog_map::MapQueryInterface & map, const Eigen::Vector2d & point,
  const Eigen::Vector2d & tangent, double center_clearance)
{
  if (!point.allFinite() || !tangent.allFinite() || tangent.squaredNorm() < 1.0e-12 ||
    !std::isfinite(center_clearance)) return false;
  constexpr double kSideOffsets[] = {0.20, 0.35, 0.55};
  constexpr double kMinimumDrop = 0.03;
  const Eigen::Vector2d normal(-tangent.y(), tangent.x());
  bool left = false;
  bool right = false;
  for (const double offset : kSideOffsets) {
    for (const int sign : {-1, 1}) {
      const Eigen::Vector2d sample = point + sign * offset * normal;
      const auto result = map.query(Eigen::Vector3d(sample.x(), sample.y(), 0.0));
      if (!result.ok || !std::isfinite(result.distance)) continue;
      if (result.distance < center_clearance - kMinimumDrop) {
        (sign > 0 ? left : right) = true;
      }
    }
    if (left && right) return true;
  }
  return false;
}

}  // namespace mas2027_nav_executor
