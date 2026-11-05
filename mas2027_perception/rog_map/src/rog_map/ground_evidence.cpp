#include <rog_map/ground_evidence.hpp>
#include <rog_map/query_adapter.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/LU>

namespace rog_map {
namespace {

bool enclosesCenter(std::vector<double> & bearings)
{
  if (bearings.size() < 3) return false;
  constexpr double pi = 3.14159265358979323846;
  std::sort(bearings.begin(), bearings.end());
  double gap = bearings.front() + 2 * pi - bearings.back();
  for (size_t i = 1; i < bearings.size(); ++i)
    gap = std::max(gap, bearings[i] - bearings[i - 1]);
  return gap < pi - 1e-6;
}

}  // namespace

GroundEvidenceView::GroundEvidenceView(const MapSnapshot & source) : source_(source)
{
  if (source.width <= 0 || source.height <= 0 || !std::isfinite(source.resolution) ||
      source.resolution <= 0.0 || !std::isfinite(source.max_step_height) ||
      source.max_step_height <= 0.0) return;
  const size_t count = static_cast<size_t>(source.width) * source.height;
  if (source.types.size() != count || source.values.size() != count ||
      source.ground_heights.size() != count ||
      (!source.raw_types.empty() && source.raw_types.size() != count) ||
      (!source.ground_precise.empty() && source.ground_precise.size() != count) ||
      (!source.height_deltas.empty() && source.height_deltas.size() != count) ||
      (!source.height_discontinuities.empty() && source.height_discontinuities.size() != count))
    return;
  valid_ = true;
  measured_support_.resize(count, 0U);
  for (size_t i = 0; i < count; ++i) {
    const uint8_t raw = source.raw_types.empty() ? source.types[i] : source.raw_types[i];
    const bool precise = !source.ground_precise.empty() && source.ground_precise[i] == 1U;
    const bool measured = raw == 2U || (raw == 1U && precise);
    measured_support_[i] = measured && source.types[i] != 3U && source.values[i] < 253U &&
      std::isfinite(source.ground_heights[i]) &&
      (precise || source.height_deltas.empty() || (std::isfinite(source.height_deltas[i]) &&
        source.height_deltas[i] <= source.max_step_height));
  }
}

bool GroundEvidenceView::inside(int x, int y) const
{
  return valid_ && x >= 0 && y >= 0 && x < source_.width && y < source_.height;
}

bool GroundEvidenceView::measuredSupport(int x, int y) const
{
  return inside(x, y) && measured_support_[static_cast<size_t>(y) * source_.width + x] != 0U;
}

bool GroundEvidenceView::continuousSupport(int x, int y, bool allow_sparse) const
{
  if (!inside(x, y)) return false;
  const auto & source = source_;
  if (measuredSupport(x, y)) {
    const double z = source.ground_heights[static_cast<size_t>(y) * source.width + x];
    bool dense = true;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
      dense = dense && measuredSupport(x + dx, y + dy);
      if (dense) dense = std::abs(source.ground_heights[
        static_cast<size_t>(y + dy) * source.width + x + dx] - z) <= source.max_step_height + 1e-6;
    }
    if (dense) return true;
  }
  if (!allow_sparse) return false;

  // A bounded precise patch can disprove a voxel-rounding edge, including on
  // non-planar ground. Semantic labels and one-sided returns are insufficient.
  const size_t center = static_cast<size_t>(y) * source.width + x;
  if (!source.ground_precise.empty() && source.ground_precise[center] == 1U && measuredSupport(x, y)) {
    double low = source.ground_heights[center], high = low;
    int count = 0;
    bool blocked = false;
    std::vector<double> bearings;
    for (int dy = -3; dy <= 3; ++dy) for (int dx = -3; dx <= 3; ++dx) {
      const int nx = x + dx, ny = y + dy;
      if (!inside(nx, ny)) continue;
      const size_t i = static_cast<size_t>(ny) * source.width + nx;
      if (source.values[i] >= 253U && source.types[i] == 3U) blocked = true;
      if (source.ground_precise[i] != 1U || !measuredSupport(nx, ny)) continue;
      low = std::min(low, static_cast<double>(source.ground_heights[i]));
      high = std::max(high, static_cast<double>(source.ground_heights[i]));
      ++count;
      if (dx || dy) bearings.push_back(std::atan2(dy, dx));
    }
    if (!blocked && count >= 6 && high - low <= source.max_step_height + 1e-6 &&
        enclosesCenter(bearings)) return true;
  }

  // Lidar rings and body blind spots need not produce a dense 3x3 return.
  // Require an enclosing plane and reject every sample beyond the residual
  // bound so the two levels of a step cannot masquerade as continuous ground.
  const int radius = static_cast<int>(std::min(
    static_cast<double>(std::max(source.width, source.height)), std::ceil(0.5 / source.resolution)));
  std::vector<std::pair<Eigen::Vector3d, double>> samples;
  std::vector<double> angles;
  Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  for (int dy = -radius; dy <= radius; ++dy) for (int dx = -radius; dx <= radius; ++dx) {
    if (!measuredSupport(x + dx, y + dy)) continue;
    const Eigen::Vector3d row(dx * source.resolution, dy * source.resolution, 1.0);
    const double z = source.ground_heights[static_cast<size_t>(y + dy) * source.width + x + dx];
    normal.noalias() += row * row.transpose();
    rhs += row * z;
    samples.emplace_back(row, z);
    if (dx || dy) angles.push_back(std::atan2(dy, dx));
  }
  if (samples.size() < 6 || !enclosesCenter(angles) || std::abs(normal.determinant()) < 1e-9)
    return false;
  const Eigen::Vector3d plane = normal.fullPivLu().solve(rhs);
  if (!plane.allFinite() || plane.head<2>().norm() > 0.70) return false;
  for (const auto & sample : samples)
    if (std::abs(sample.second - plane.dot(sample.first)) > 0.5 * source.max_step_height)
      return false;
  return true;
}

bool GroundEvidenceView::smoothSupport(int sx, int sy) const
{
  if (!valid_) return false;
  const auto & source = source_;
  constexpr int radius = 4;
  constexpr double max_gap = 0.030, max_range = 0.090, max_slope = 0.70;
  Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  std::vector<double> heights, bearings;
  double low = std::numeric_limits<double>::infinity(), high = -low;
  for (int dy = -radius; dy <= radius; ++dy) for (int dx = -radius; dx <= radius; ++dx) {
    const int x = sx + dx, y = sy + dy;
    if (!inside(x, y)) continue;
    const size_t i = static_cast<size_t>(y) * source.width + x;
    const uint8_t raw = source.raw_types.empty() ? source.types[i] : source.raw_types[i];
    // Enclosure and plane samples deliberately have different provenance:
    // retained finite ground can enclose, but only fresh precise returns fit.
    if (raw != 3U && std::isfinite(source.ground_heights[i]) && (dx || dy))
      bearings.push_back(std::atan2(dy, dx));
    if (!measuredSupport(x, y)) continue;
    if (!source.height_discontinuities.empty() && source.height_discontinuities[i] != 0U) continue;
    if (source.ground_precise.empty() || source.ground_precise[i] != 1U) continue;
    const double z = source.ground_heights[i];
    const Eigen::Vector3d row(dx * source.resolution, dy * source.resolution, 1.0);
    normal.noalias() += row * row.transpose();
    rhs.noalias() += row * z;
    heights.push_back(z);
    low = std::min(low, z);
    high = std::max(high, z);
  }
  if (heights.size() < 8 || bearings.size() < 8 || !enclosesCenter(bearings)) return false;
  std::sort(heights.begin(), heights.end());
  double largest_gap = 0.0;
  for (size_t i = 1; i < heights.size(); ++i)
    largest_gap = std::max(largest_gap, heights[i] - heights[i - 1]);
  if (largest_gap > max_gap || high - low > max_range || std::abs(normal.determinant()) < 1e-9)
    return false;
  const Eigen::Vector3d plane = normal.fullPivLu().solve(rhs);
  return plane.allFinite() && plane.head<2>().norm() <= max_slope;
}

}  // namespace rog_map
