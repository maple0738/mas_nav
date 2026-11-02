#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace rog_map {

// Label is the containing cell; body and direction are continuous cell-center
// bilinear fields. Their spatial derivatives are used by MINCO's adjoint.
struct TerrainDirectionSample {
  bool known{false};
  uint8_t label{255U};
  double body{0.0};
  Eigen::Vector2d body_gradient{Eigen::Vector2d::Zero()};
  Eigen::Vector2d uphill{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d uphill_jacobian{Eigen::Matrix2d::Zero()};
};

inline TerrainDirectionSample sampleTerrainDirection(int width, int height, double resolution,
                                                     double ox, double oy,
                                                     const std::vector<uint8_t>& labels,
                                                     const std::vector<float>& ux,
                                                     const std::vector<float>& uy, double x,
                                                     double y) {
  TerrainDirectionSample out;
  const size_t count = width > 0 && height > 0 ? static_cast<size_t>(width) * height : 0;
  if (!count || labels.size() != count || ux.size() != count || uy.size() != count ||
      !(resolution > 0.0) || !std::isfinite(resolution) || !std::isfinite(ox) ||
      !std::isfinite(oy) || !std::isfinite(x) || !std::isfinite(y))
    return out;
  const double gx = (x - ox) / resolution, gy = (y - oy) / resolution;
  if (gx < 0 || gy < 0 || gx >= width || gy >= height) return out;
  out.label = labels[static_cast<size_t>(static_cast<int>(gy)) * width + static_cast<int>(gx)];
  out.known = out.label != 255U;
  const int bx = static_cast<int>(std::floor(gx - 0.5));
  const int by = static_cast<int>(std::floor(gy - 0.5));
  const double fx = gx - 0.5 - bx, fy = gy - 0.5 - by;
  Eigen::Vector2d raw = Eigen::Vector2d::Zero();
  Eigen::Matrix2d jac = Eigen::Matrix2d::Zero();
  for (int dy = 0; dy < 2; ++dy)
    for (int dx = 0; dx < 2; ++dx) {
      const int cx = bx + dx, cy = by + dy;
      if (cx < 0 || cy < 0 || cx >= width || cy >= height) continue;
      const size_t k = static_cast<size_t>(cy) * width + cx;
      if (labels[k] != 5U || !std::isfinite(ux[k]) || !std::isfinite(uy[k])) continue;
      Eigen::Vector2d axis(ux[k], uy[k]);
      if (axis.norm() < 0.5) continue;
      axis.normalize();
      const double wx = dx ? fx : 1 - fx, wy = dy ? fy : 1 - fy;
      const double w = wx * wy;
      const Eigen::Vector2d dw((dx ? 1 : -1) * wy / resolution, (dy ? 1 : -1) * wx / resolution);
      out.body += w;
      out.body_gradient += dw;
      raw += w * axis;
      jac += axis * dw.transpose();
    }
  const double norm = raw.norm();
  if (norm > 1e-6) {
    out.uphill = raw / norm;
    out.uphill_jacobian =
        (Eigen::Matrix2d::Identity() - out.uphill * out.uphill.transpose()) * jac / norm;
  } else {
    // Conflicting/missing directions cannot be an optimizer run-up source.
    out.body = 0.0;
    out.body_gradient.setZero();
  }
  return out;
}

}  // namespace rog_map
