#pragma once

#include <cmath>

#include <super_utils/eigen_alias.hpp>

namespace rog_map {

// Registered cloud points are in the map frame. Exclude only returns inside
// the chassis square at the pose paired with this cloud, before raycasting.
// The configured radius is the square's corner radius, not a disk to erase
// from the accumulated occupancy map. Precompute the transform once per cloud.
class BodyReturnFilter
{
public:
  BodyReturnFilter(const super_utils::Pose & pose, double corner_radius)
  : x_(pose.first.x()), y_(pose.first.y()), half_side_(corner_radius / std::sqrt(2.0))
  {
    const auto & q = pose.second;
    enabled_ = std::isfinite(corner_radius) && corner_radius > 0.0 &&
      std::isfinite(x_) && std::isfinite(y_) &&
      std::isfinite(q.w()) && std::isfinite(q.x()) &&
      std::isfinite(q.y()) && std::isfinite(q.z());
    if (enabled_) {
      const double yaw = std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
      cos_yaw_ = std::cos(yaw);
      sin_yaw_ = std::sin(yaw);
    }
  }

  bool contains(double x, double y) const
  {
    if (!enabled_ || !std::isfinite(x) || !std::isfinite(y)) {
      return false;
    }
    const double dx = x - x_;
    const double dy = y - y_;
    return std::abs(cos_yaw_ * dx + sin_yaw_ * dy) <= half_side_ &&
      std::abs(-sin_yaw_ * dx + cos_yaw_ * dy) <= half_side_;
  }

private:
  double x_{0.0};
  double y_{0.0};
  double half_side_{0.0};
  double cos_yaw_{1.0};
  double sin_yaw_{0.0};
  bool enabled_{false};
};

}  // namespace rog_map
