#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace mas2027_nav_executor {

// Mark the one-cell apron outside the long sides of each observed slope as
// blocked. The local footprint supplies the boundary normal; the fitted
// uphill vector (or a prior map's unsigned long axis) identifies its ends.
inline std::vector<uint8_t> buildSlopeSideMask(int width, int height,
  const std::vector<uint8_t> & labels, const std::vector<float> & uphill_x,
  const std::vector<float> & uphill_y)
{
  const size_t count = width > 0 && height > 0 ?
    static_cast<size_t>(width) * height : 0U;
  std::vector<uint8_t> blocked(count, 0U);
  if (labels.size() != count || uphill_x.size() != count ||
      uphill_y.size() != count) return blocked;
  const auto inside = [width, height](int x, int y) {
    return x >= 0 && y >= 0 && x < width && y < height;
  };
  const auto index = [width](int x, int y) {
    return static_cast<size_t>(y) * width + x;
  };
  constexpr double kEndCosine = 0.78;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (labels[index(x, y)] == 5U) continue;
      bool adjacent = false;
      double axis_x = 0.0, axis_y = 0.0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!inside(x + dx, y + dy)) continue;
          const size_t neighbor = index(x + dx, y + dy);
          if (labels[neighbor] != 5U) continue;
          adjacent = true;
          if (std::hypot(axis_x, axis_y) < 0.5 &&
              std::isfinite(uphill_x[neighbor]) && std::isfinite(uphill_y[neighbor]) &&
              std::hypot(uphill_x[neighbor], uphill_y[neighbor]) >= 0.5) {
            axis_x = uphill_x[neighbor];
            axis_y = uphill_y[neighbor];
          }
        }
      }
      const double axis_norm = std::hypot(axis_x, axis_y);
      if (!adjacent || axis_norm < 0.5) continue;
      double normal_x = 0.0, normal_y = 0.0;
      int support = 0;
      for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
          if (!inside(x + dx, y + dy) || labels[index(x + dx, y + dy)] != 5U) continue;
          ++support;
          const double weight = 1.0 / (1.0 + dx * dx + dy * dy);
          normal_x += dx * weight;
          normal_y += dy * weight;
        }
      }
      if (support < 3) continue;
      const double normal_norm = std::hypot(normal_x, normal_y);
      if (normal_norm < 1e-6 ||
          std::abs(axis_x * normal_x + axis_y * normal_y) /
            (axis_norm * normal_norm) < kEndCosine) blocked[index(x, y)] = 1U;
    }
  }
  return blocked;
}

// Supercover sampling checks both cells touched at a diagonal corner, so a
// path cannot slip between a side obstacle and a slope corner.
inline bool terrainSideTransitionAllowed(int width, int height, double resolution,
  double origin_x, double origin_y, const std::vector<uint8_t> & blocked,
  double from_x, double from_y, double to_x, double to_y, bool outside_blocked)
{
  if (width <= 0 || height <= 0 || resolution <= 0.0 ||
      blocked.size() != static_cast<size_t>(width) * height) return false;
  const auto cell = [=](double x, double y) -> std::optional<std::pair<int, int>> {
    if (!std::isfinite(x) || !std::isfinite(y)) return std::nullopt;
    const double cx = std::floor((x - origin_x) / resolution);
    const double cy = std::floor((y - origin_y) / resolution);
    if (cx < 0.0 || cy < 0.0 || cx >= width || cy >= height) return std::nullopt;
    return std::pair<int, int>(static_cast<int>(cx), static_cast<int>(cy));
  };
  const auto blocked_at = [=, &blocked](int x, int y) {
    if (x < 0 || y < 0 || x >= width || y >= height) return outside_blocked;
    return blocked[static_cast<size_t>(y) * width + x] != 0U;
  };
  const double distance = std::hypot(to_x - from_x, to_y - from_y);
  if (!std::isfinite(distance)) return false;
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / (0.5 * resolution))));
  auto previous = cell(from_x, from_y);
  if ((!previous && outside_blocked) ||
      (previous && blocked_at(previous->first, previous->second))) return false;
  for (int step = 1; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / steps;
    const auto next = cell(from_x + ratio * (to_x - from_x),
      from_y + ratio * (to_y - from_y));
    if ((!next && outside_blocked) ||
        (next && blocked_at(next->first, next->second))) return false;
    if (previous && next && previous->first != next->first &&
        previous->second != next->second &&
        (blocked_at(next->first, previous->second) ||
         blocked_at(previous->first, next->second))) return false;
    previous = next;
  }
  return true;
}

}  // namespace mas2027_nav_executor
