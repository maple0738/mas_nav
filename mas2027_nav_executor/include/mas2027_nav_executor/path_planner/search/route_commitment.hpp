#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace minco_planner {

// Keep a traversable committed route when a rolling-map search suddenly asks
// the robot to reverse and take a comparable or longer detour. The returned
// suffix starts at the measured pose; every connecting segment is checked on
// the live map before it can replace the newly searched route.
inline std::optional<std::vector<geometry_msgs::msg::PoseStamped>>
committedRouteForReversal(
  const geometry_msgs::msg::PoseStamped & start,
  const std::vector<geometry_msgs::msg::PoseStamped> & committed,
  const std::vector<geometry_msgs::msg::PoseStamped> & proposed,
  double map_resolution, const std::function<bool(double, double)> & traversable)
{
  if (committed.size() < 2U || proposed.size() < 2U || !traversable ||
    !std::isfinite(map_resolution) || map_resolution <= 0.0 ||
    start.header.frame_id != committed.front().header.frame_id ||
    start.header.frame_id != proposed.front().header.frame_id) {
    return std::nullopt;
  }
  const auto distance = [](const auto & a, const auto & b) {
    return std::hypot(a.pose.position.x - b.pose.position.x,
      a.pose.position.y - b.pose.position.y);
  };
  if (distance(committed.back(), proposed.back()) > 0.30) {
    return std::nullopt;
  }

  size_t nearest = committed.size();
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0U; i + 1U < committed.size(); ++i) {
    const double d = distance(start, committed[i]);
    if (d < nearest_distance) {
      nearest_distance = d;
      nearest = i;
    }
  }
  if (nearest >= committed.size() - 1U ||
    nearest_distance > std::max(0.20, 3.0 * map_resolution)) {
    return std::nullopt;
  }

  std::vector<geometry_msgs::msg::PoseStamped> suffix;
  suffix.reserve(committed.size() - nearest);
  suffix.push_back(start);
  suffix.insert(suffix.end(), committed.begin() + nearest + 1U, committed.end());
  const auto length = [&distance](const auto & path) {
    double sum = 0.0;
    for (size_t i = 1U; i < path.size(); ++i) sum += distance(path[i - 1U], path[i]);
    return sum;
  };
  const double old_length = length(suffix);
  const double new_length = length(proposed);
  if (old_length < 0.40 || new_length < 0.40 || new_length < 0.75 * old_length) {
    return std::nullopt;
  }
  const auto ahead = [&distance](const auto & path) {
    for (size_t i = 1U; i < path.size(); ++i) {
      if (distance(path.front(), path[i]) >= 0.30) {
        return std::pair<double, double>{
          path[i].pose.position.x - path.front().pose.position.x,
          path[i].pose.position.y - path.front().pose.position.y};
      }
    }
    return std::pair<double, double>{
      path.back().pose.position.x - path.front().pose.position.x,
      path.back().pose.position.y - path.front().pose.position.y};
  };
  const auto [old_dx, old_dy] = ahead(suffix);
  const auto [new_dx, new_dy] = ahead(proposed);
  const double magnitude = std::hypot(old_dx, old_dy) * std::hypot(new_dx, new_dy);
  if (magnitude < 1e-6 || (old_dx * new_dx + old_dy * new_dy) / magnitude > -0.25) {
    return std::nullopt;
  }

  const double spacing = std::max(0.01, std::min(0.05, 0.5 * map_resolution));
  for (size_t i = 1U; i < suffix.size(); ++i) {
    const auto & a = suffix[i - 1U].pose.position;
    const auto & b = suffix[i].pose.position;
    const double segment = std::hypot(b.x - a.x, b.y - a.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(segment / spacing)));
    for (int step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / steps;
      if (!traversable(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))) {
        return std::nullopt;
      }
    }
  }
  return suffix;
}

}  // namespace minco_planner
