#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <algorithm>
#include <vector>

#include "rog_map/terrain_direction.hpp"

namespace mas2027_nav_executor {

struct DirectedTerrainConfig {
  bool enable{false};
  double min_alignment_cosine{0.8};
  // Reserve straight approach space on both ends of a bidirectional ramp.
  double approach_distance{0.5};
};
using TerrainSampler = std::function<rog_map::TerrainDirectionSample(double, double)>;

// Extend observed ordinary/free cells along each physical ramp axis. This
// reserves the front runway before a turn can reach the slope's side. It does
// not create free cells or alter obstacle/step/side/clearance masks.
inline TerrainSampler makeDirectedTerrainSampler(int width, int height, double resolution,
                                                 double ox, double oy, TerrainSampler physical,
                                                 const DirectedTerrainConfig& cfg,
                                                 std::function<bool(int, int)> observed_free = {}) {
  if (!cfg.enable || cfg.approach_distance <= 0 || !physical || width <= 0 || height <= 0 ||
      !(resolution > 0))
    return physical;
  struct Field {
    std::vector<uint8_t> labels;
    std::vector<float> x, y;
  };
  auto field = std::make_shared<Field>();
  const size_t n = static_cast<size_t>(width) * height;
  field->labels.resize(n);
  field->x.resize(n);
  field->y.resize(n);
  std::vector<size_t> slopes;
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      const size_t k = static_cast<size_t>(y) * width + x;
      const auto s = physical(ox + (x + 0.5) * resolution, oy + (y + 0.5) * resolution);
      field->labels[k] = s.label;
      field->x[k] = s.uphill.x();
      field->y[k] = s.uphill.y();
      if (s.label == 5U && s.uphill.squaredNorm() > 0.5) slopes.push_back(k);
    }
  const auto physical_labels = field->labels;
  for (const size_t k : slopes) {
    const double x = ox + (k % width + 0.5) * resolution, y = oy + (k / width + 0.5) * resolution;
    const Eigen::Vector2d axis(field->x[k], field->y[k]);
    for (int sign : {-1, 1})
      // A ramp cell's center is half a cell inside its physical boundary.
      // Include that half cell so the requested metric runway is not shortened.
      for (double d = 0.5 * resolution;
           d <= cfg.approach_distance + 0.5 * resolution + 1e-9; d += 0.5 * resolution) {
        const int cx = static_cast<int>(std::floor((x + sign * d * axis.x() - ox) / resolution));
        const int cy = static_cast<int>(std::floor((y + sign * d * axis.y() - oy) / resolution));
        if (cx < 0 || cy < 0 || cx >= width || cy >= height) break;
        const size_t j = static_cast<size_t>(cy) * width + cx;
        // The accumulated map stores unclassified free ground as label 255.
        // It can receive an additional direction restriction when occupancy is
        // observed free. Never extend it onto genuinely unknown/blocked cells.
        if (physical_labels[j] != 0U &&
            !(physical_labels[j] == 255U && observed_free && observed_free(cx, cy))) continue;
        if (field->labels[j] == 5U &&
            std::abs(Eigen::Vector2d(field->x[j], field->y[j]).dot(axis)) <
                cfg.min_alignment_cosine) {
          field->x[j] = field->y[j] = 0;
        } else if (field->labels[j] != 5U) {
          field->labels[j] = 5U;
          field->x[j] = axis.x();
          field->y[j] = axis.y();
        }
      }
  }
  return [field, width, height, resolution, ox, oy](double x, double y) {
    return rog_map::sampleTerrainDirection(width, height, resolution, ox, oy, field->labels,
                                           field->x, field->y, x, y);
  };
}

// Both ascent and descent are allowed. A slope's missing/conflicting axis is
// rejected, while absent/unknown semantics retain the existing geometric gates.
// Stationary cell probes are legal; they are not movement edges.
inline bool directed_terrain_edge_allowed(const TerrainSampler& sample, const Eigen::Vector2d& from,
                                          const Eigen::Vector2d& to,
                                          const DirectedTerrainConfig& cfg,
                                          double sampling_distance) {
  if (!cfg.enable || !sample) return true;
  if (!from.allFinite() || !to.allFinite()) return false;
  const Eigen::Vector2d delta = to - from;
  const double length = delta.norm();
  if (length < 1e-9) return true;
  const Eigen::Vector2d movement = delta / length;
  const int count =
      std::max(1, static_cast<int>(std::ceil(length / std::max(0.01, sampling_distance))));
  int sign = 0;
  for (int i = 0; i <= count; ++i) {
    const Eigen::Vector2d p = from + (static_cast<double>(i) / count) * delta;
    const auto terrain = sample(p.x(), p.y());
    if (terrain.label != 5U) continue;
    if (terrain.uphill.squaredNorm() < 0.5) return false;
    const double alignment = movement.dot(terrain.uphill);
    if (std::abs(alignment) + 1e-9 < cfg.min_alignment_cosine) return false;
    const int next_sign = alignment > 0 ? 1 : -1;
    if (sign && sign != next_sign) return false;
    sign = next_sign;
  }
  return true;
}

// Estimate direction over a map-resolved arc window, rather than dividing by
// sub-millimeter time-step chords near a stop. Every dense trajectory position
// is still checked against the same field and cosine; collision/step checks
// retain their independent dense sampling.
inline bool directed_terrain_path_allowed(const TerrainSampler & sample,
    const std::vector<Eigen::Vector2d> & points, const DirectedTerrainConfig & cfg,
    double direction_span, size_t * rejected = nullptr) {
  if (!cfg.enable || !sample || points.empty()) return true;
  std::vector<double> arc(points.size(), 0.0);
  for (size_t i = 0; i < points.size(); ++i) {
    if (!points[i].allFinite()) return false;
    if (i) arc[i] = arc[i-1] + (points[i]-points[i-1]).norm();
  }
  if (arc.back() < 1e-9) return true;
  const double span = std::min(arc.back(), std::max(0.01, direction_span));
  const auto at = [&](double s) -> Eigen::Vector2d {
    const auto it = std::lower_bound(arc.begin(), arc.end(), s);
    if (it == arc.begin()) return points.front();
    if (it == arc.end()) return points.back();
    const size_t j = static_cast<size_t>(it-arc.begin());
    const double u = (s-arc[j-1])/std::max(1e-12,arc[j]-arc[j-1]);
    return points[j-1]+u*(points[j]-points[j-1]);
  };
  for (size_t i = 0; i < points.size(); ++i) {
    const auto terrain = sample(points[i].x(),points[i].y());
    if (terrain.label != 5U) continue;
    const double begin = std::clamp(arc[i]-0.5*span,0.0,arc.back()-span);
    const Eigen::Vector2d a = at(begin), b = at(begin+span);
    Eigen::Vector2d delta = b-a;
    // A stop-and-reverse along a ramp axis is legal too.
    if (delta.norm() < 1e-9) delta = b-points[i];
    if (delta.norm() < 1e-9) delta = points[i]-a;
    if (terrain.uphill.squaredNorm() < 0.5 ||
        (delta.norm() >= 1e-9 && std::abs(delta.normalized().dot(terrain.uphill))+1e-9 <
          cfg.min_alignment_cosine)) {
      if (rejected) *rejected = i;
      return false;
    }
  }
  return true;
}

}  // namespace mas2027_nav_executor
