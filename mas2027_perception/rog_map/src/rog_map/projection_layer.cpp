#include <rog_map/projection_layer.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <Eigen/LU>

namespace rog_map {

namespace {

double elapsedMs(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

CellType classifyCell(
  const ColumnStats & stats, CellData & cell, const ProjectionLayerConfig & config, double resolution)
{
  const int min_observed = std::max(1, config.min_observed_voxels);
  cell.confidence = static_cast<float>(
    std::clamp(static_cast<double>(stats.observed_count) / static_cast<double>(min_observed), 0.0, 1.0));

  if (stats.observed_count < min_observed) {
    cell.raw_reason = ProjectionClassReason::INSUFFICIENT_OBSERVATION;
    cell.traversable = 0U;
    return CellType::UNKNOWN;
  }

  if (stats.occupied_count == 0) {
    cell.raw_reason = ProjectionClassReason::EMPTY_COLUMN;
    cell.traversable = 1U;
    return CellType::FREE;
  }

  const bool valid_occupied_span = stats.occupied_z_index_min <= stats.occupied_z_index_max &&
                                   std::isfinite(stats.occupied_z_min_abs) &&
                                   std::isfinite(stats.occupied_z_max_abs);
  if (!valid_occupied_span) {
    cell.raw_reason = ProjectionClassReason::AMBIGUOUS_OCCUPIED;
    cell.traversable = 0U;
    return CellType::OCCUPIED;
  }

  const int span_steps = stats.occupied_z_index_max - stats.occupied_z_index_min;
  const double height_delta = static_cast<double>(span_steps) * resolution;
  cell.occupied_z_min_abs = static_cast<float>(stats.occupied_z_min_abs);
  cell.occupied_z_max_abs = static_cast<float>(stats.occupied_z_max_abs);
  cell.height_delta = static_cast<float>(height_delta);
  const int interior_steps = std::max(0, span_steps - 1);
  cell.free_gap_ratio = interior_steps > 0 ?
    static_cast<float>(stats.free_between_occupied_count) / interior_steps : 0.0f;

  if (height_delta <= config.surface_height_delta_max) {
    cell.vertical_occupancy_ratio = 1.0F;
    cell.raw_reason = ProjectionClassReason::THIN_SURFACE;
    cell.traversable = 1U;
    return CellType::PASSABLE;
  }

  const double vertical_occupancy_ratio =
    std::clamp((static_cast<double>(stats.occupied_count - 1) * resolution) / height_delta, 0.0, 1.0);
  cell.vertical_occupancy_ratio = static_cast<float>(vertical_occupancy_ratio);

  if (height_delta >= config.wall_height_delta_min &&
      vertical_occupancy_ratio >= config.wall_occupancy_ratio_min) {
    cell.raw_reason = ProjectionClassReason::SOLID_VERTICAL_WALL;
    cell.traversable = 0U;
    return CellType::OCCUPIED;
  }

  if (height_delta >= config.tunnel_height_delta_min && height_delta <= config.tunnel_height_delta_max &&
      vertical_occupancy_ratio <= config.tunnel_occupancy_ratio_max) {
    cell.raw_reason = ProjectionClassReason::HOLLOW_TUNNEL;
    cell.traversable = 1U;
    return CellType::PASSABLE;
  }

  cell.raw_reason = ProjectionClassReason::AMBIGUOUS_OCCUPIED;
  cell.traversable = 0U;
  return CellType::OCCUPIED;
}

}  // namespace

TerrainGeometry ProjectionLayer::terrainGeometry() const
{
  constexpr uint8_t kUnknown = 255U;
  constexpr uint8_t kFlat = 0U;
  constexpr uint8_t kSlope = 5U;
  constexpr uint8_t kTunnel = 6U;
  constexpr int kRadius = 5;
  constexpr double kMinSlope = 0.14;  // tan(8 degrees)
  constexpr double kMaxSlope = 0.70;  // tan(35 degrees)
  constexpr double kMaxPlaneRms = 0.04;
  // A single raised thin surface can still leave the plane RMS below 4 cm
  // because it is one sample among many. Reject it by its residual instead;
  // otherwise a table/box top tilts a flat-ground plane into a false slope.
  // The actual field run had 10--16 cm raised samples in an otherwise flat
  // 0.55 m window, including one confirmed false mode-5 region.
  constexpr double kMaxPlaneResidual = 0.05;
  const size_t count = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  TerrainGeometry geometry;
  auto & labels = geometry.labels;
  labels.assign(count, kUnknown);
  geometry.uphill_x.assign(count, 0.0f);
  geometry.uphill_y.assign(count, 0.0f);
  if (cells_.size() != count || width_ <= 0 || height_ <= 0 || resolution_ <= 0.0) {
    return geometry;
  }

  const auto thinGround = [](const CellData & cell) {
    return cell.type == CellType::PASSABLE &&
      cell.raw_reason == ProjectionClassReason::THIN_SURFACE &&
      std::isfinite(cell.occupied_z_min_abs);
  };
  const auto & tunnel_config = current_config_;
  const auto tunnelCandidate = [&tunnel_config](const CellData & cell) {
    // Keep the semantic label tied to the same passability decision as the
    // projection layer. A tall occupied column is not a tunnel merely because
    // its sparse returns happen to have a large vertical gap.
    const double height_delta = static_cast<double>(cell.height_delta);
    return cell.type == CellType::PASSABLE &&
      cell.raw_reason == ProjectionClassReason::HOLLOW_TUNNEL &&
      std::isfinite(cell.occupied_z_min_abs) &&
      std::isfinite(cell.occupied_z_max_abs) &&
      height_delta >= tunnel_config.tunnel_height_delta_min &&
      height_delta <= tunnel_config.tunnel_height_delta_max &&
      static_cast<double>(cell.vertical_occupancy_ratio) <=
        tunnel_config.tunnel_occupancy_ratio_max &&
      cell.free_gap_ratio >= 0.5f;
  };

  struct PlaneSample
  {
    double x;
    double y;
    double z;
  };
  std::vector<PlaneSample> plane_samples;
  plane_samples.reserve((2 * kRadius + 1) * (2 * kRadius + 1));

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const size_t index = static_cast<size_t>(y) * width_ + x;
      const auto & center = cells_[index];
      if (center.type == CellType::UNKNOWN) {
        continue;
      }
      // Empty columns do not prove flat ground; leave them unknown so the
      // static label can be used as fallback by the executor.
      if (thinGround(center)) labels[index] = kFlat;

      // A gap between two occupied heights is only a tunnel candidate when
      // neighboring columns show the same structure. A single sparse wall hit
      // must not create a semantic region.
      if (tunnelCandidate(center)) {
        int support = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx >= 0 && ny >= 0 && nx < width_ && ny < height_ &&
                tunnelCandidate(cells_[static_cast<size_t>(ny) * width_ + nx])) {
              ++support;
            }
          }
        }
        if (support >= 5) {
          labels[index] = kTunnel;
          continue;
        }
      }
      if (!thinGround(center)) {
        continue;
      }

      // Fit z = ax + by + c to neighboring thin ground observations. The
      // within-column height span is not a slope measurement.
      double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
      double sum_xx = 0.0, sum_xy = 0.0, sum_yy = 0.0;
      double sum_xz = 0.0, sum_yz = 0.0, sum_zz = 0.0;
      plane_samples.clear();
      float lowest = center.occupied_z_min_abs;
      float highest = lowest;
      for (int dy = -kRadius; dy <= kRadius; ++dy) {
        for (int dx = -kRadius; dx <= kRadius; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;
          const auto & neighbor = cells_[static_cast<size_t>(ny) * width_ + nx];
          if (!thinGround(neighbor)) continue;
          const double px = dx * resolution_;
          const double py = dy * resolution_;
          const double z = neighbor.occupied_z_min_abs;
          sum_x += px;
          sum_y += py;
          sum_z += z;
          sum_xx += px * px;
          sum_xy += px * py;
          sum_yy += py * py;
          sum_xz += px * z;
          sum_yz += py * z;
          sum_zz += z * z;
          lowest = std::min(lowest, neighbor.occupied_z_min_abs);
          highest = std::max(highest, neighbor.occupied_z_min_abs);
          plane_samples.push_back({px, py, z});
        }
      }
      const int samples = static_cast<int>(plane_samples.size());
      if (samples < 30 || highest - lowest < 0.06f) continue;
      Eigen::Matrix3d normal;
      normal << sum_xx, sum_xy, sum_x,
                sum_xy, sum_yy, sum_y,
                sum_x, sum_y, samples;
      const Eigen::Vector3d rhs(sum_xz, sum_yz, sum_z);
      if (std::abs(normal.determinant()) < 1e-9) continue;
      const Eigen::Vector3d plane = normal.fullPivLu().solve(rhs);
      const double slope = plane.head<2>().norm();
      if (!plane.allFinite() || slope < kMinSlope || slope > kMaxSlope) continue;
      const double squared_error = std::max(0.0, sum_zz - plane.dot(rhs));
      double max_residual = 0.0;
      for (const auto & sample : plane_samples) {
        max_residual = std::max(max_residual, std::abs(sample.z -
          (plane.x() * sample.x + plane.y() * sample.y + plane.z())));
      }
      if (std::sqrt(squared_error / samples) <= kMaxPlaneRms &&
          max_residual <= kMaxPlaneResidual) {
        labels[index] = kSlope;
        geometry.uphill_x[index] = static_cast<float>(plane.x() / slope);
        geometry.uphill_y[index] = static_cast<float>(plane.y() / slope);
      }
    }
  }
  return geometry;
}

std::vector<uint8_t> ProjectionLayer::terrainLabels() const
{
  return terrainGeometry().labels;
}

ProjectionSlideResult ProjectionLayer::syncSlidingWindow(int width,
  int height,
  double resolution,
  const Eigen::Vector2i & min_id,
  const Eigen::Vector2d & origin,
  const ProjectionLayerConfig & config)
{
  if (width <= 0 || height <= 0 || resolution <= 0.0 || width % 2 == 0 || height % 2 == 0) {
    throw std::invalid_argument(
      "ProjectionLayer sliding window requires positive odd dimensions and resolution");
  }

  ProjectionSlideResult result;
  current_config_ = config;
  const Vec3i half_size(width / 2, height / 2, 0);
  const Vec3i center_id(min_id.x() + half_size.x(), min_id.y() + half_size.y(), 0);
  Vec3f center_pos;

  if (!initialized_) {
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_ = origin;
    cell_buffer_.assign(static_cast<size_t>(width_) * static_cast<size_t>(height_), CellData{});
    cells_.assign(cell_buffer_.size(), CellData{});
    values_.assign(cell_buffer_.size(), 255U);
    mask_.assign(cell_buffer_.size(), config.unknown_as_occupied ? 0U : 1U);
    initSlidingMap(half_size, resolution, true, 0.0, Vec3f::Zero());
    globalIndexToPos(center_id, center_pos);
    updateLocalMapOriginAndBound(center_pos, center_id);
    initialized_ = true;
    view_rebuild_required_ = true;
    resetLocalMap();
    result.full_refresh_required = true;
    result.dirty_columns.resize(cell_buffer_.size());
    for (size_t i = 0; i < result.dirty_columns.size(); ++i) {
      result.dirty_columns[i] = static_cast<int>(i);
    }
    return result;
  }

  if (width_ != width || height_ != height || std::abs(resolution_ - resolution) > 1.0e-9 ||
      (sc_.map_size_i - Vec3i(width, height, 1)).cwiseAbs().maxCoeff() != 0 ||
      cell_buffer_.size() != static_cast<size_t>(width) * static_cast<size_t>(height)) {
    throw std::invalid_argument(
      "ProjectionLayer sliding storage geometry cannot change after initialization");
  }

  const Vec3i shift = center_id - local_map_origin_i_;
  result.window_moved = shift.x() != 0 || shift.y() != 0;
  view_rebuild_required_ = result.window_moved;
  result.full_refresh_required = std::abs(shift.x()) >= width_ || std::abs(shift.y()) >= height_;
  slide_dirty_hash_ids_.clear();
  globalIndexToPos(center_id, center_pos);
  SlidingMap::mapSliding(center_pos);
  origin_ = origin;

  std::vector<uint8_t> dirty_flags(cell_buffer_.size(), 0U);
  for (const int hash_id : slide_dirty_hash_ids_) {
    if (hash_id < 0 || hash_id >= static_cast<int>(cell_buffer_.size())) {
      continue;
    }
    Vec3i global_id;
    hashIdToGlobalIndex(hash_id, global_id);
    const int x = global_id.x() - local_map_bound_min_i_.x();
    const int y = global_id.y() - local_map_bound_min_i_.y();
    if (x < 0 || y < 0 || x >= width_ || y >= height_) {
      continue;
    }
    const int view_id = y * width_ + x;
    if (!dirty_flags[static_cast<size_t>(view_id)]) {
      dirty_flags[static_cast<size_t>(view_id)] = 1U;
      result.dirty_columns.push_back(view_id);
    }
  }
  if (result.full_refresh_required && result.dirty_columns.size() != cell_buffer_.size()) {
    result.dirty_columns.resize(cell_buffer_.size());
    for (size_t i = 0; i < result.dirty_columns.size(); ++i) {
      result.dirty_columns[i] = static_cast<int>(i);
    }
  }
  return result;
}

void ProjectionLayer::update(int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  double now,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner,
  ProjectionUpdateStats * stats)
{
  updateFull(width, height, resolution, origin, now, config, scanner, stats);
}

void ProjectionLayer::updateFull(int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  double now,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner,
  ProjectionUpdateStats * stats)
{
  const auto full_start = std::chrono::steady_clock::now();
  if (width <= 0 || height <= 0 || resolution <= 0.0 || !scanner) {
    width_ = 0;
    height_ = 0;
    resolution_ = 0.0;
    cells_.clear();
    values_.clear();
    mask_.clear();
    if (stats) {
      stats->update_full_time_ms = elapsedMs(full_start);
    }
    return;
  }

  if (!initialized_) {
    const Eigen::Vector2i min_id(static_cast<int>(std::llround(origin.x() / resolution)),
      static_cast<int>(std::llround(origin.y() / resolution)));
    syncSlidingWindow(width, height, resolution, min_id, origin, config);
  }
  if (!matchesGeometry(width, height, resolution, origin)) {
    throw std::invalid_argument("ProjectionLayer::updateFull called with unsynchronized geometry");
  }
  current_config_ = config;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      updateOneCell(x, y, now, config, scanner);
    }
  }

  const auto filter_start = std::chrono::steady_clock::now();
  filterGround(config);
  double view_time_ms = 0.0;
  filterMask(config, nullptr, &view_time_ms);
  view_rebuild_required_ = false;
  if (stats) {
    stats->mask_filter_time_ms += std::max(0.0, elapsedMs(filter_start) - view_time_ms);
    stats->value_mask_time_ms += view_time_ms;
    stats->update_full_time_ms += elapsedMs(full_start);
    collectClassificationStats(cells_, stats);
  }
}

void ProjectionLayer::updateDirty(int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  double now,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner,
  const std::vector<int> & dirty_columns,
  bool force_full_refresh,
  ProjectionUpdateStats * stats)
{
  const auto dirty_start = std::chrono::steady_clock::now();
  const bool geometry_changed = !matchesGeometry(width, height, resolution, origin);
  if (force_full_refresh || geometry_changed || cells_.empty()) {
    updateFull(width, height, resolution, origin, now, config, scanner, stats);
    return;
  }

  current_config_ = config;
  if (dirty_columns.empty()) {
    if (stats) {
      stats->update_dirty_time_ms += elapsedMs(dirty_start);
      collectClassificationStats(cells_, stats);
    }
    return;
  }

  const size_t expected = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  std::vector<int> base_dirty_indices;
  base_dirty_indices.reserve(dirty_columns.size());
  for (const int column_id : dirty_columns) {
    if (column_id < 0 || column_id >= static_cast<int>(expected)) {
      continue;
    }
    base_dirty_indices.push_back(column_id);
  }
  std::sort(base_dirty_indices.begin(), base_dirty_indices.end());
  base_dirty_indices.erase(
    std::unique(base_dirty_indices.begin(), base_dirty_indices.end()), base_dirty_indices.end());

  for (const int idx_int : base_dirty_indices) {
    updateOneCell(idx_int % width_, idx_int / width_, now, config, scanner);
  }

  const auto filter_start = std::chrono::steady_clock::now();
  const auto ground_changed = filterGround(config);
  base_dirty_indices.insert(base_dirty_indices.end(), ground_changed.begin(), ground_changed.end());
  std::sort(base_dirty_indices.begin(), base_dirty_indices.end());
  base_dirty_indices.erase(std::unique(base_dirty_indices.begin(), base_dirty_indices.end()),
    base_dirty_indices.end());
  double view_time_ms = 0.0;
  filterMask(
    config, view_rebuild_required_ ? nullptr : &base_dirty_indices, &view_time_ms);
  view_rebuild_required_ = false;
  if (stats) {
    stats->mask_filter_time_ms += std::max(0.0, elapsedMs(filter_start) - view_time_ms);
    stats->value_mask_time_ms += view_time_ms;
    stats->update_dirty_time_ms += elapsedMs(dirty_start);
    collectClassificationStats(cells_, stats);
  }
}

std::vector<int> ProjectionLayer::filterGround(const ProjectionLayerConfig & config)
{
  std::vector<int> changed;
  if (!config.ground_filter_en) return changed;
  constexpr int radius = 3;
  constexpr int min_support = 8;
  constexpr double max_rms = 0.025;
  constexpr double max_slope = 0.7;
  // Only original thin observations may support a fit. Corrected thick cells
  // never become seeds, so correction cannot spread across an obstacle.
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      auto & center = cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
      const bool corrected = center.raw_reason == ProjectionClassReason::THIN_SURFACE &&
        center.height_delta > config.surface_height_delta_max + 1e-6;
      if ((!corrected && center.raw_reason != ProjectionClassReason::AMBIGUOUS_OCCUPIED) ||
        !std::isfinite(center.occupied_z_min_abs) || !std::isfinite(center.occupied_z_max_abs) ||
        center.height_delta > config.ground_filter_max_span + 1e-6) continue;
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      double sum_zz = 0.0;
      int samples = 0;
      bool left = false, right = false, below = false, above = false;
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const int nx = x + dx, ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;
          const auto & neighbor = cell_buffer_[static_cast<size_t>(hashIndexFromLocal(nx, ny))];
          if (neighbor.raw_type != CellType::PASSABLE ||
            neighbor.raw_reason != ProjectionClassReason::THIN_SURFACE ||
            neighbor.height_delta > config.surface_height_delta_max + 1e-6 ||
            !std::isfinite(neighbor.occupied_z_min_abs) ||
            !std::isfinite(neighbor.occupied_z_max_abs)) continue;
          const Eigen::Vector3d row(dx * resolution_, dy * resolution_, 1.0);
          const double z = 0.5 * (neighbor.occupied_z_min_abs + neighbor.occupied_z_max_abs);
          normal.noalias() += row * row.transpose();
          rhs.noalias() += row * z;
          sum_zz += z * z;
          ++samples;
          left |= dx < 0; right |= dx > 0; below |= dy < 0; above |= dy > 0;
        }
      }
      bool ground = false;
      if (samples >= min_support && left && right && below && above &&
          std::abs(normal.determinant()) > 1e-9) {
        const Eigen::Vector3d plane = normal.fullPivLu().solve(rhs);
        const double rms = std::sqrt(std::max(0.0, sum_zz - plane.dot(rhs)) / samples);
        const double tolerance = config.ground_filter_plane_tolerance + 1e-6;
        ground = plane.allFinite() && plane.head<2>().norm() <= max_slope && rms <= max_rms &&
          std::abs(center.occupied_z_min_abs - plane.z()) <= tolerance &&
          std::abs(center.occupied_z_max_abs - plane.z()) <= tolerance;
      }
      const auto type = ground ? CellType::PASSABLE : CellType::OCCUPIED;
      const auto reason = ground ? ProjectionClassReason::THIN_SURFACE :
        ProjectionClassReason::AMBIGUOUS_OCCUPIED;
      if (center.raw_type == type && center.base_type == type && center.raw_reason == reason) continue;
      center.raw_type = center.base_type = center.type = center.pending_type = type;
      center.raw_reason = reason;
      center.pending_count = 0U;
      center.occupied_clear_deadline = 0.0;
      center.traversable = ground ? 1U : 0U;
      changed.push_back(y * width_ + x);
    }
  }
  return changed;
}

bool ProjectionLayer::matchesGeometry(
  int width, int height, double resolution, const Eigen::Vector2d & origin) const
{
  return width_ == width && height_ == height &&
         cells_.size() == static_cast<size_t>(width) * static_cast<size_t>(height) &&
         std::abs(resolution_ - resolution) <= 1.0e-9 && (origin_ - origin).norm() <= 1.0e-9;
}

void ProjectionLayer::applyValueAndMask(CellData & cell, const ProjectionLayerConfig & config)
{
  // value is a cost/debug layer; mask is the 2D ESDF source, where 0 is obstacle and 1 is free.
  switch (cell.type) {
  case CellType::UNKNOWN:
    cell.value = config.unknown_as_occupied ? 254U : 255U;
    cell.mask = config.unknown_as_occupied ? 0U : 1U;
    cell.traversable = config.unknown_as_occupied ? 0U : 1U;
    break;
  case CellType::FREE:
    cell.value = 0U;
    cell.mask = 1U;
    cell.traversable = 1U;
    break;
  case CellType::PASSABLE:
    cell.value = config.passable_as_free ? 0U : config.passable_cost;
    cell.mask = 1U;
    cell.traversable = 1U;
    break;
  case CellType::OCCUPIED:
    cell.value = 254U;
    cell.mask = 0U;
    cell.traversable = 0U;
    break;
  }
}

void ProjectionLayer::collectClassificationStats(
  const std::vector<CellData> & cells, ProjectionUpdateStats * stats)
{
  if (!stats) {
    return;
  }
  stats->thin_surface_count = 0.0;
  stats->vertical_wall_count = 0.0;
  stats->hollow_tunnel_count = 0.0;
  stats->ambiguous_occupied_count = 0.0;
  stats->empty_column_count = 0.0;
  stats->insufficient_observation_count = 0.0;
  for (const auto & cell : cells) {
    switch (cell.raw_reason) {
    case ProjectionClassReason::INSUFFICIENT_OBSERVATION:
      stats->insufficient_observation_count += 1.0;
      break;
    case ProjectionClassReason::EMPTY_COLUMN:
      stats->empty_column_count += 1.0;
      break;
    case ProjectionClassReason::THIN_SURFACE:
      stats->thin_surface_count += 1.0;
      break;
    case ProjectionClassReason::SOLID_VERTICAL_WALL:
      stats->vertical_wall_count += 1.0;
      break;
    case ProjectionClassReason::HOLLOW_TUNNEL:
      stats->hollow_tunnel_count += 1.0;
      break;
    case ProjectionClassReason::AMBIGUOUS_OCCUPIED:
      stats->ambiguous_occupied_count += 1.0;
      break;
    }
  }
}

void ProjectionLayer::updateOneCell(
  int x, int y, double now, const ProjectionLayerConfig & config, const ColumnScanner & scanner)
{
  const ColumnStats stats = scanner(x, y);
  const int hash_id = hashIndexFromLocal(x, y);
  const CellData previous = cell_buffer_[static_cast<size_t>(hash_id)];

  CellData cell;
  cell.last_hit_time = static_cast<float>(stats.last_hit_time);
  cell.last_update_time = static_cast<float>(stats.last_update_time);
  const CellType raw_type = classifyCell(stats, cell, config, resolution_);
  cell.raw_type = raw_type;
  cell.base_type = previous.base_type;
  cell.pending_type = previous.pending_type;
  cell.pending_count = previous.pending_count;
  cell.occupied_clear_deadline = previous.occupied_clear_deadline;

  const bool hold_active = previous.occupied_clear_deadline > 0.0;
  if (raw_type == CellType::OCCUPIED) {
    cell.base_type = CellType::OCCUPIED;
    cell.occupied_clear_deadline = 0.0;
    cell.pending_type = CellType::OCCUPIED;
    cell.pending_count = 0U;
  } else if (config.obstacle_hold_time > 0.0 &&
             (previous.raw_type == CellType::OCCUPIED || hold_active)) {
    if (!hold_active) {
      cell.occupied_clear_deadline = now + config.obstacle_hold_time;
    }
    if (now < cell.occupied_clear_deadline) {
      cell.base_type = CellType::OCCUPIED;
      cell.pending_type = raw_type;
      cell.pending_count = 0U;
    } else {
      cell.base_type = raw_type;
      cell.occupied_clear_deadline = 0.0;
      cell.pending_type = raw_type;
      cell.pending_count = 0U;
    }
  } else {
    cell.occupied_clear_deadline = 0.0;
    cell.base_type = applyHysteresis(cell, raw_type, config);
  }
  cell.type = cell.base_type;
  cell.hole_filled = false;
  cell_buffer_[static_cast<size_t>(hash_id)] = cell;
}

bool ProjectionLayer::advanceObstacleClearance(
  double now, const ProjectionLayerConfig & config)
{
  std::vector<int> changed_indices;
  for (size_t hash_id = 0; hash_id < cell_buffer_.size(); ++hash_id) {
    auto & cell = cell_buffer_[hash_id];
    if (cell.occupied_clear_deadline <= 0.0 || now < cell.occupied_clear_deadline) {
      continue;
    }
    cell.occupied_clear_deadline = 0.0;
    if (cell.raw_type == CellType::OCCUPIED || cell.base_type != CellType::OCCUPIED) {
      continue;
    }
    cell.base_type = cell.raw_type;
    cell.pending_type = cell.raw_type;
    cell.pending_count = 0U;
    Vec3i global_id;
    hashIdToGlobalIndex(static_cast<int>(hash_id), global_id);
    const int x = global_id.x() - local_map_bound_min_i_.x();
    const int y = global_id.y() - local_map_bound_min_i_.y();
    if (x >= 0 && y >= 0 && x < width_ && y < height_) {
      changed_indices.push_back(y * width_ + x);
    }
  }
  if (!changed_indices.empty()) {
    filterMask(config, &changed_indices);
  }
  return !changed_indices.empty();
}

CellType ProjectionLayer::applyHysteresis(
  CellData & cell, CellType raw_type, const ProjectionLayerConfig & config)
{
  // OCCUPIED enters immediately; clearing to FREE/PASSABLE waits for repeated confirmation.
  if (!config.hysteresis_en || config.hysteresis_count <= 0 ||
      cell.base_type == CellType::UNKNOWN ||
      raw_type == CellType::OCCUPIED) {
    cell.base_type = raw_type;
    cell.pending_type = raw_type;
    cell.pending_count = 0U;
    return cell.base_type;
  }

  if (raw_type == cell.base_type) {
    cell.pending_type = raw_type;
    cell.pending_count = 0U;
    return cell.base_type;
  }

  if (cell.base_type == CellType::OCCUPIED && raw_type != CellType::OCCUPIED) {
    if (cell.pending_type != raw_type) {
      cell.pending_type = raw_type;
      cell.pending_count = 1U;
    } else {
      cell.pending_count = static_cast<uint8_t>(std::min<int>(255, cell.pending_count + 1));
    }
    if (cell.pending_count >= static_cast<uint8_t>(config.hysteresis_count)) {
      cell.base_type = raw_type;
      cell.pending_count = 0U;
    }
    return cell.base_type;
  }

  cell.base_type = raw_type;
  cell.pending_type = raw_type;
  cell.pending_count = 0U;
  return cell.base_type;
}

void ProjectionLayer::filterMask(
  const ProjectionLayerConfig & config,
  const std::vector<int> * base_dirty_indices,
  double * view_time_ms)
{
  if (width_ <= 0 || height_ <= 0 || cell_buffer_.empty()) {
    return;
  }

  std::vector<int> affected;
  if (base_dirty_indices) {
    constexpr int kFilterInfluenceRadius = 2;
    const size_t expected = static_cast<size_t>(width_) * static_cast<size_t>(height_);
    std::vector<uint8_t> update_mask(expected, 0U);
    affected.reserve(base_dirty_indices->size() * 25U);
    for (const int view_id : *base_dirty_indices) {
      if (view_id < 0 || view_id >= static_cast<int>(expected)) {
        continue;
      }
      const int cx = view_id % width_;
      const int cy = view_id / width_;
      for (int dy = -kFilterInfluenceRadius; dy <= kFilterInfluenceRadius; ++dy) {
        for (int dx = -kFilterInfluenceRadius; dx <= kFilterInfluenceRadius; ++dx) {
          const int nx = cx + dx;
          const int ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
            continue;
          }
          const int nidx = ny * width_ + nx;
          if (update_mask[static_cast<size_t>(nidx)] == 0U) {
            update_mask[static_cast<size_t>(nidx)] = 1U;
            affected.push_back(nidx);
          }
        }
      }
    }
  } else {
    const int expected = width_ * height_;
    affected.resize(static_cast<size_t>(expected));
    for (int i = 0; i < expected; ++i) {
      affected[static_cast<size_t>(i)] = i;
    }
  }

  if (config.mask_filter_en) {
    fillMask(config, affected);
    denoiseMask(config, affected);
  } else {
    for (const int view_id : affected) {
      const int x = view_id % width_;
      const int y = view_id / width_;
      CellData & cell =
        cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
      cell.hole_filled = false;
      cell.type = cell.base_type;
    }
  }

  const auto view_start = std::chrono::steady_clock::now();
  if (base_dirty_indices) {
    for (const int view_id : affected) {
      updateView(view_id, config);
    }
  } else {
    rebuildViews(config);
  }
  if (view_time_ms) {
    *view_time_ms += elapsedMs(view_start);
  }
}

void ProjectionLayer::fillMask(
  const ProjectionLayerConfig & config, const std::vector<int> & candidate_indices)
{
  for (const int view_id : candidate_indices) {
    const int x = view_id % width_;
    const int y = view_id / width_;
    CellData & cell = cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
    cell.hole_filled = false;
    if (x <= 0 || y <= 0 || x >= width_ - 1 || y >= height_ - 1 ||
        (cell.base_type != CellType::UNKNOWN && cell.base_type != CellType::FREE)) {
      continue;
    }

    int occupied_neighbors = 0;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const CellData & neighbor = cell_buffer_[static_cast<size_t>(
          hashIndexFromLocal(x + dx, y + dy))];
        if (neighbor.base_type == CellType::OCCUPIED) {
          ++occupied_neighbors;
        }
      }
    }
    cell.hole_filled = occupied_neighbors >= config.fill_occ_min;
  }
}

void ProjectionLayer::denoiseMask(
  const ProjectionLayerConfig & config, const std::vector<int> & candidate_indices)
{
  for (const int view_id : candidate_indices) {
    const int x = view_id % width_;
    const int y = view_id / width_;
    CellData & cell = cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
    const CellType filled_type =
      cell.hole_filled ? CellType::OCCUPIED : cell.base_type;
    cell.type = filled_type;
    if (filled_type != CellType::OCCUPIED ||
        x <= 0 || y <= 0 || x >= width_ - 1 || y >= height_ - 1) {
      continue;
    }

    int occupied_neighbors = 0;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const CellData & neighbor = cell_buffer_[static_cast<size_t>(
          hashIndexFromLocal(x + dx, y + dy))];
        if (neighbor.base_type == CellType::OCCUPIED || neighbor.hole_filled) {
          ++occupied_neighbors;
        }
      }
    }
    if (occupied_neighbors <= config.denoise_occ_max) {
      cell.type = CellType::UNKNOWN;
    }
  }
}

void ProjectionLayer::updateView(int view_id, const ProjectionLayerConfig & config)
{
  const int x = view_id % width_;
  const int y = view_id / width_;
  const size_t view_index = static_cast<size_t>(view_id);
  CellData & stored = cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
  applyValueAndMask(stored, config);
  cells_[view_index] = stored;
  values_[view_index] = stored.value;
  mask_[view_index] = stored.mask;
}

void ProjectionLayer::rebuildViews(const ProjectionLayerConfig & config)
{
  const size_t expected =
    static_cast<size_t>(std::max(0, width_)) * static_cast<size_t>(std::max(0, height_));
  cells_.resize(expected);
  values_.resize(expected);
  mask_.resize(expected);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const size_t view_id = static_cast<size_t>(y) * static_cast<size_t>(width_) + static_cast<size_t>(x);
      updateView(static_cast<int>(view_id), config);
    }
  }
}

int ProjectionLayer::hashIndexFromLocal(int x, int y) const
{
  const Vec3i global_id(local_map_bound_min_i_.x() + x, local_map_bound_min_i_.y() + y, 0);
  return SlidingMap::getHashIndexFromGlobalIndex(global_id);
}

void ProjectionLayer::resetLocalMap()
{
  slide_dirty_hash_ids_.clear();
  for (size_t i = 0; i < cell_buffer_.size(); ++i) {
    CellData cell;
    applyValueAndMask(cell, current_config_);
    cell_buffer_[i] = cell;
    slide_dirty_hash_ids_.push_back(static_cast<int>(i));
  }
}

void ProjectionLayer::resetCell(const int & hash_id)
{
  if (hash_id < 0 || hash_id >= static_cast<int>(cell_buffer_.size())) {
    return;
  }
  CellData cell;
  applyValueAndMask(cell, current_config_);
  cell_buffer_[static_cast<size_t>(hash_id)] = cell;
  slide_dirty_hash_ids_.push_back(hash_id);
}

}  // namespace rog_map
