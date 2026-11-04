#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mas2027_nav_executor {

// Exact discrete nearest-point lookup. Blocks are only a pruning device: all
// surviving points retain the original distance calculation and earliest tie.
// A hint seeds an upper bound, never limits the search to a local path segment.
class PolylinePointIndex {
public:
  struct Nearest {
    size_t index{0};
    double squared_distance{std::numeric_limits<double>::infinity()};
    bool valid() const { return std::isfinite(squared_distance); }
  };

  template <typename PointAt> void build(size_t count, PointAt point_at) {
    points_.resize(count);
    blocks_.assign((count + kBlockSize - 1) / kBlockSize, {});
    for (size_t i = 0; i < count; ++i) {
      const Eigen::Vector2d p = point_at(i);
      points_[i] = p;
      if (!p.allFinite())
        continue;
      auto &b = blocks_[i / kBlockSize];
      b.min_x = std::min(b.min_x, p.x());
      b.max_x = std::max(b.max_x, p.x());
      b.min_y = std::min(b.min_y, p.y());
      b.max_y = std::max(b.max_y, p.y());
    }
  }

  Nearest nearest(double x, double y, size_t hint = 0) const {
    Nearest result;
    if (!std::isfinite(x) || !std::isfinite(y))
      return result;
    const auto consider = [&](size_t i) {
      const double dx = points_[i].x() - x, dy = points_[i].y() - y;
      const double squared = dx * dx + dy * dy;
      if (squared < result.squared_distance ||
          (squared == result.squared_distance && i < result.index)) {
        result.index = i;
        result.squared_distance = squared;
      }
    };
    if (hint < points_.size())
      consider(hint);
    for (size_t k = 0; k < blocks_.size(); ++k) {
      const auto &b = blocks_[k];
      if (b.min_x > b.max_x)
        continue;
      const double dx = x < b.min_x   ? b.min_x - x
                        : x > b.max_x ? x - b.max_x
                                      : 0.0;
      const double dy = y < b.min_y   ? b.min_y - y
                        : y > b.max_y ? y - b.max_y
                                      : 0.0;
      // Slack is confined to pruning, never to distance/tie or clearance tests.
      const double limit =
          result.squared_distance + 8 * std::numeric_limits<double>::epsilon() *
                                        std::max(1.0, result.squared_distance);
      if (dx * dx + dy * dy > limit)
        continue;
      for (size_t i = k * kBlockSize;
           i < std::min(points_.size(), (k + 1) * kBlockSize); ++i)
        consider(i);
    }
    return result;
  }

private:
  static constexpr size_t kBlockSize = 16;
  struct Block {
    double min_x{std::numeric_limits<double>::infinity()};
    double min_y{std::numeric_limits<double>::infinity()};
    double max_x{-std::numeric_limits<double>::infinity()};
    double max_y{-std::numeric_limits<double>::infinity()};
  };
  std::vector<Eigen::Vector2d> points_;
  std::vector<Block> blocks_;
};

} // namespace mas2027_nav_executor
