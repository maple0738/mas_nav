#pragma once
#include <utility>

namespace mas2027_nav_executor {

// C1 gate and derivative with respect to forward arc distance (HW port).
inline std::pair<double, double> runup_distance_gate(double distance, double radius = 0.2,
                                                     double transition = 0.1) {
  if (distance <= radius) return {1.0, 0.0};
  if (transition <= 0.0 || distance >= radius + transition) return {0.0, 0.0};
  const double x = (radius + transition - distance) / transition;
  return {x * x * (3.0 - 2.0 * x), -6.0 * x * (1.0 - x) / transition};
}

}  // namespace mas2027_nav_executor
