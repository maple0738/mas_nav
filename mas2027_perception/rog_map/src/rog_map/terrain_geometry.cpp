#include <rog_map/projection_layer.hpp>
#include <rog_map/undulating_profile.hpp>

#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>

namespace rog_map {
namespace {
bool coherentWaveProfiles(const std::array<double, 17> &a,
                          const std::array<double, 17> &b,
                          const UndulatingRecognitionConfig &config) {
  // Compare only jointly measured heights. Similar wavelengths alone do not
  // prove a surface: neighboring strips must put crests at the same position.
  int count = 0;
  double sum_a = 0.0, sum_b = 0.0, aa = 0.0, bb = 0.0, ab = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    if (!std::isfinite(a[i]) || !std::isfinite(b[i]))
      continue;
    ++count;
    sum_a += a[i];
    sum_b += b[i];
    aa += a[i] * a[i];
    bb += b[i] * b[i];
    ab += a[i] * b[i];
  }
  if (count < config.min_common_samples)
    return false;
  aa -= sum_a * sum_a / count;
  bb -= sum_b * sum_b / count;
  ab -= sum_a * sum_b / count;
  return aa > 1e-8 && bb > 1e-8 && ab > 0.0 &&
         ab * ab >= config.min_neighbor_correlation *
                        config.min_neighbor_correlation * aa * bb;
}

} // namespace

TerrainGeometry ProjectionLayer::terrainGeometry() const {
  constexpr uint8_t kUnknown = 255U;
  constexpr uint8_t kFlat = 0U;
  constexpr uint8_t kSlope = 5U;
  constexpr uint8_t kTunnel = 6U;
  const auto &slope_config = current_config_.terrain.slope;
  const auto &wave_config = current_config_.terrain.undulating;
  const int kRadius = slope_config.fit_radius_cells;
  // A single raised thin surface can still leave the plane RMS below 4 cm
  // because it is one sample among many. Reject it by its residual instead;
  // otherwise a table/box top tilts a flat-ground plane into a false slope.
  // The actual field run had 10--16 cm raised samples in an otherwise flat
  // 0.55 m window, including one confirmed false mode-5 region.
  const size_t count =
      static_cast<size_t>(width_) * static_cast<size_t>(height_);
  TerrainGeometry geometry;
  auto &labels = geometry.labels;
  labels.assign(count, kUnknown);
  geometry.uphill_x.assign(count, 0.0f);
  geometry.uphill_y.assign(count, 0.0f);
  if (cells_.size() != count || width_ <= 0 || height_ <= 0 ||
      resolution_ <= 0.0) {
    return geometry;
  }

  const auto thinGround = [](const CellData &cell) {
    return (cell.type == CellType::PASSABLE || cell.type == CellType::FREE) &&
           std::isfinite(cell.ground_reference_z);
  };
  const auto measuredWaveGround = [&thinGround](const CellData &cell) {
    // A 50 mm voxel boundary can turn millimetre floor noise into alternating
    // height levels. Only validated raw-point surface heights prove waves.
    return thinGround(cell) && std::isfinite(cell.precise_ground_z) &&
           !cell.height_discontinuity;
  };
  const auto &tunnel_config = current_config_;
  const auto tunnelCandidate = [&tunnel_config](const CellData &cell) {
    // Keep the semantic label tied to the same passability decision as the
    // projection layer. A tall occupied column is not a tunnel merely because
    // its sparse returns happen to have a large vertical gap.
    const double height_delta = static_cast<double>(cell.height_delta);
    return tunnel_config.terrain.tunnel.enable &&
           cell.type == CellType::PASSABLE &&
           cell.raw_reason == ProjectionClassReason::HOLLOW_TUNNEL &&
           std::isfinite(cell.occupied_z_min_abs) &&
           std::isfinite(cell.occupied_z_max_abs) &&
           height_delta >= tunnel_config.tunnel_height_delta_min &&
           height_delta <= tunnel_config.tunnel_height_delta_max &&
           static_cast<double>(cell.vertical_occupancy_ratio) <=
               tunnel_config.tunnel_occupancy_ratio_max &&
           cell.free_gap_ratio >=
               static_cast<float>(
                   tunnel_config.terrain.tunnel.min_free_gap_ratio);
  };

  // Neighbor windows need only heights and a few evidence bits. Extract them
  // once so repeated scans do not walk the much larger CellData records.
  constexpr uint8_t kGround = 1U, kWaveGround = 2U, kBarrier = 4U,
                    kTunnelCandidate = 8U, kObserved = 16U;
  std::vector<uint8_t> evidence(count, 0U);
  std::vector<float> ground_heights(count);
  std::vector<float> wave_heights(count);
  // A union of intersecting tiles encloses every 11x11 fit window. Its height
  // range is an upper bound on the exact range, so rejecting a range below
  // 6 cm cannot discard any slope accepted by the original fit. This avoids
  // scanning 121 CellData records per flat cell; uncertain tiles still take
  // the original fitting path, including all residual and support checks.
  constexpr int kTileSize = 8;
  struct HeightRange {
    float low{std::numeric_limits<float>::infinity()};
    float high{-std::numeric_limits<float>::infinity()};
  };
  const int tile_width = (width_ + kTileSize - 1) / kTileSize;
  const int tile_height = (height_ + kTileSize - 1) / kTileSize;
  std::vector<HeightRange> height_ranges(static_cast<size_t>(tile_width) *
                                         tile_height);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const auto &cell = cells_[static_cast<size_t>(y) * width_ + x];
      const size_t id = static_cast<size_t>(y) * width_ + x;
      if (cell.type != CellType::UNKNOWN)
        evidence[id] |= kObserved;
      ground_heights[id] = cell.ground_reference_z;
      wave_heights[id] = cell.precise_ground_z;
      if (cell.type == CellType::OCCUPIED || cell.height_discontinuity)
        evidence[id] |= kBarrier;
      if (tunnelCandidate(cell))
        evidence[id] |= kTunnelCandidate;
      if (measuredWaveGround(cell))
        evidence[id] |= kWaveGround;
      if (!thinGround(cell))
        continue;
      evidence[id] |= kGround;
      auto &range =
          height_ranges[static_cast<size_t>(y / kTileSize) * tile_width +
                        x / kTileSize];
      range.low = std::min(range.low, cell.ground_reference_z);
      range.high = std::max(range.high, cell.ground_reference_z);
    }
  }
  // Reuse the bound for every center in a tile. The extra border is
  // deliberately conservative; avoid a second neighborhood walk per cell.
  std::vector<uint8_t> flat_tiles(height_ranges.size(), 0U);
  bool has_flat_tiles = false;
  const int kTileRadius = (kRadius + kTileSize - 1) / kTileSize;
  for (int ty = 0; ty < tile_height; ++ty) {
    for (int tx = 0; tx < tile_width; ++tx) {
      HeightRange bound;
      for (int ny = std::max(0, ty - kTileRadius);
           ny <= std::min(tile_height - 1, ty + kTileRadius); ++ny) {
        for (int nx = std::max(0, tx - kTileRadius);
             nx <= std::min(tile_width - 1, tx + kTileRadius); ++nx) {
          const auto &range =
              height_ranges[static_cast<size_t>(ny) * tile_width + nx];
          bound.low = std::min(bound.low, range.low);
          bound.high = std::max(bound.high, range.high);
        }
      }
      flat_tiles[static_cast<size_t>(ty) * tile_width + tx] =
          bound.high - bound.low <
          static_cast<float>(slope_config.min_height_change_m);
      has_flat_tiles = has_flat_tiles ||
                       flat_tiles[static_cast<size_t>(ty) * tile_width + tx];
    }
  }

  struct PlaneSample {
    double x;
    double y;
    double z;
  };
  // The fit window bounds sample count. Fill reusable slots directly rather
  // than checking vector capacity on every sample in every neighboring fit.
  std::vector<PlaneSample> plane_samples((2 * kRadius + 1) * (2 * kRadius + 1));

  // Lazily allocated per direction; discarded after this snapshot, so height,
  // collision and parameter changes never reuse stale detection results.
  std::array<std::vector<int8_t>, 4> waveform_cache;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const size_t index = static_cast<size_t>(y) * width_ + x;
      if (!(evidence[index] & kObserved)) {
        continue;
      }
      // Empty columns do not prove flat ground; leave them unknown so the
      // static label can be used as fallback by the executor.
      if (evidence[index] & kGround)
        labels[index] = kFlat;

      // A gap between two occupied heights is only a tunnel candidate when
      // neighboring columns show the same structure. A single sparse wall hit
      // must not create a semantic region.
      if (evidence[index] & kTunnelCandidate) {
        int support = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx >= 0 && ny >= 0 && nx < width_ && ny < height_ &&
                (evidence[static_cast<size_t>(ny) * width_ + nx] &
                 kTunnelCandidate)) {
              ++support;
            }
          }
        }
        if (support >= tunnel_config.terrain.tunnel.min_neighbor_support) {
          labels[index] = kTunnel;
          continue;
        }
      }
      if (!(evidence[index] & kGround)) {
        continue;
      }

      // Require repeated waves on three neighboring measured profiles. Unknown
      // columns and collision barriers never supply support evidence. Only the
      // semantic label changes; collision masks retain their existing
      // authority.
      bool undulating = false;
      constexpr int axes[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
      for (size_t axis_id = 0; axis_id < 4; ++axis_id) {
        const auto &axis = axes[axis_id];
        if (!wave_config.enable || !(evidence[index] & kWaveGround))
          break;
        // Cheap local reversal gate keeps flat ground and monotonic ramps out
        // of the longer three-profile check.
        double low = wave_heights[index], high = low, previous = low;
        int last_sign = 0, measured = 0;
        bool reversal = false, valid = true;
        for (int i = -2; i <= 2; ++i) {
          const int nx = x + i * axis[0], ny = y + i * axis[1];
          if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
            valid = false;
            break;
          }
          const size_t sample_id = static_cast<size_t>(ny) * width_ + nx;
          if (evidence[sample_id] & kBarrier) {
            valid = false;
            break;
          }
          if (!(evidence[sample_id] & kWaveGround))
            continue;
          const double z = wave_heights[sample_id];
          low = std::min(low, z);
          high = std::max(high, z);
          if (measured > 0) {
            const double delta = z - previous;
            const int sign = delta > wave_config.min_delta_m    ? 1
                             : delta < -wave_config.min_delta_m ? -1
                                                                : 0;
            if (sign && last_sign && sign != last_sign)
              reversal = true;
            if (sign)
              last_sign = sign;
          }
          previous = z;
          ++measured;
        }
        if (!valid || measured < 3 || !reversal ||
            high - low < wave_config.min_peak_to_trough_m)
          continue;
        ++geometry.wave_candidate_count;
        bool supported = true;
        auto &cache = waveform_cache[axis_id];
        if (cache.empty())
          cache.assign(count, -1);
        std::array<double, 17> previous_profile{};
        for (int strip = -1; strip <= 1 && supported; ++strip) {
          const int cx = x - strip * axis[1], cy = y + strip * axis[0];
          if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) {
            supported = false;
            break;
          }
          auto &cached = cache[static_cast<size_t>(cy) * width_ + cx];
          if (cached == 0) {
            ++geometry.wave_profile_cache_hits;
            supported = false;
            break;
          }
          std::array<double, 17> profile;
          profile.fill(std::numeric_limits<double>::quiet_NaN());
          for (int i = -8; i <= 8; ++i) {
            const int nx = x + i * axis[0] - strip * axis[1];
            const int ny = y + i * axis[1] + strip * axis[0];
            if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
              supported = false;
              break;
            }
            const size_t sample_id = static_cast<size_t>(ny) * width_ + nx;
            if (evidence[sample_id] & kBarrier) {
              supported = false;
              break;
            }
            if (evidence[sample_id] & kWaveGround)
              profile[i + 8] = wave_heights[sample_id];
          }
          if (supported) {
            bool wave = cached == 1;
            if (wave) {
              ++geometry.wave_profile_cache_hits;
            } else {
              ++geometry.wave_profile_check_count;
              const double spacing = resolution_ * std::hypot(axis[0], axis[1]);
              wave = isUndulatingProfile(profile, spacing, wave_config);
              for (int offset : {0, 4}) {
                if (!wave_config.entry_window_enable || wave)
                  break;
                std::array<double, 13> near;
                std::copy_n(profile.begin() + offset, 13, near.begin());
                wave = isUndulatingProfile(near, spacing, wave_config);
              }
              cached = wave ? 1 : 0;
            }
            supported = wave && (strip == -1 ||
                                 coherentWaveProfiles(previous_profile, profile,
                                                      wave_config));
            previous_profile = profile;
          } else {
            cached = 0;
          }
        }
        if (supported) {
          undulating = true;
          break;
        }
      }
      if (undulating) {
        labels[index] = 7U;
        continue;
      }

      if (!slope_config.enable)
        continue;
      if (has_flat_tiles &&
          flat_tiles[static_cast<size_t>(y / kTileSize) * tile_width +
                     x / kTileSize]) {
        ++geometry.slope_prefilter_rejections;
        continue;
      }
      // Fit z = ax + by + c to neighboring thin ground observations. The
      // within-column height span is not a slope measurement.
      double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
      double sum_xx = 0.0, sum_xy = 0.0, sum_yy = 0.0;
      double sum_xz = 0.0, sum_yz = 0.0, sum_zz = 0.0;
      int samples = 0;
      float lowest = ground_heights[index];
      float highest = lowest;
      for (int dy = -kRadius; dy <= kRadius; ++dy) {
        for (int dx = -kRadius; dx <= kRadius; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_)
            continue;
          const size_t sample_id = static_cast<size_t>(ny) * width_ + nx;
          if (!(evidence[sample_id] & kGround))
            continue;
          const double px = dx * resolution_;
          const double py = dy * resolution_;
          const double z = ground_heights[sample_id];
          sum_x += px;
          sum_y += py;
          sum_z += z;
          sum_xx += px * px;
          sum_xy += px * py;
          sum_yy += py * py;
          sum_xz += px * z;
          sum_yz += py * z;
          sum_zz += z * z;
          lowest = std::min(lowest, ground_heights[sample_id]);
          highest = std::max(highest, ground_heights[sample_id]);
          plane_samples[samples++] = {px, py, z};
        }
      }
      if (samples < slope_config.min_samples ||
          highest - lowest <
              static_cast<float>(slope_config.min_height_change_m))
        continue;
      Eigen::Matrix3d normal;
      normal << sum_xx, sum_xy, sum_x, sum_xy, sum_yy, sum_y, sum_x, sum_y,
          samples;
      const Eigen::Vector3d rhs(sum_xz, sum_yz, sum_z);
      if (std::abs(normal.determinant()) < 1e-9)
        continue;
      ++geometry.slope_fit_count;
      const Eigen::Vector3d plane = normal.fullPivLu().solve(rhs);
      const double slope = plane.head<2>().norm();
      if (!plane.allFinite() || slope < slope_config.min_gradient ||
          slope > slope_config.max_gradient)
        continue;
      const double squared_error = std::max(0.0, sum_zz - plane.dot(rhs));
      double max_residual = 0.0;
      for (int i = 0; i < samples; ++i) {
        const auto &sample = plane_samples[i];
        max_residual =
            std::max(max_residual,
                     std::abs(sample.z - (plane.x() * sample.x +
                                          plane.y() * sample.y + plane.z())));
      }
      if (std::sqrt(squared_error / samples) <= slope_config.max_plane_rms_m &&
          max_residual <= slope_config.max_plane_residual_m) {
        labels[index] = kSlope;
        geometry.uphill_x[index] = static_cast<float>(plane.x() / slope);
        geometry.uphill_y[index] = static_cast<float>(plane.y() / slope);
      }
    }
  }

  // Head-on entry signature. From the low sensor side, only the first rising
  // face is measured: the crest hides the trough, and the next crest top
  // reappears at the same height. There is no height reversal to feed the
  // profile detector, so recognize the face + hidden run + same-height crest
  // patch instead. This only changes the semantic label; masks and step
  // barriers keep their existing authority.
  if (wave_config.enable && wave_config.front_signature_enable) {
    const int max_face = wave_config.front_max_face_cells;
    const int max_gap = wave_config.front_max_hidden_cells;
    const int max_crest = wave_config.front_max_crest_cells;
    const double crest_tolerance =
        wave_config.front_crest_height_tolerance_m;
    std::vector<int> center_cells, side_cells, supported_cells;
    center_cells.reserve(static_cast<size_t>(max_face + max_crest));
    side_cells.reserve(static_cast<size_t>(max_face + max_crest));
    supported_cells.reserve(
        static_cast<size_t>(3 * (max_face + max_crest)));
    const auto rayCell = [&](int sx, int sy, double ux, double uy, int step,
                             int & cx, int & cy) {
      cx = sx + static_cast<int>(std::lround(ux * step));
      cy = sy + static_cast<int>(std::lround(uy * step));
      return cx >= 0 && cy >= 0 && cx < width_ && cy < height_;
    };
    const auto signature = [&](int sx, int sy, double ux, double uy,
                               double & face_top,
                               std::vector<int> & cells) {
      cells.clear();
      int last = -1;
      int face_cells = 0;
      double bottom = 0.0;
      face_top = -std::numeric_limits<double>::infinity();
      int step = 0;
      for (; step <= max_face; ++step) {
        int cx, cy;
        if (!rayCell(sx, sy, ux, uy, step, cx, cy)) return false;
        const int id = cy * width_ + cx;
        if (id == last) continue;
        last = id;
        const size_t sample = static_cast<size_t>(id);
        if (!(evidence[sample] & kGround)) break;
        const double z = ground_heights[sample];
        if (face_cells == 0) bottom = z;
        face_top = std::max(face_top, z);
        cells.push_back(id);
        ++face_cells;
      }
      if (face_cells < wave_config.front_min_face_cells ||
          face_top - bottom < wave_config.front_min_face_height_m)
        return false;
      // A wave starts from near-level ground. On a long grade the cells
      // behind the start keep rising with the same trend, so a hidden far
      // side of a ramp crest must not pass as a wave.
      for (int i = 1; i <= wave_config.front_approach_cells; ++i) {
        int cx, cy;
        if (!rayCell(sx, sy, ux, uy, -i, cx, cy)) return false;
        const size_t sample =
            static_cast<size_t>(cy) * width_ + static_cast<size_t>(cx);
        if (!(evidence[sample] & kGround) ||
            std::abs(ground_heights[sample] - bottom) >
                wave_config.front_approach_tolerance_m)
          return false;
      }
      // The trough behind the crest has no ground return because the crest
      // blocks the line of sight. A wall or raised object is not a shadow.
      int gap = 0;
      bool crest_found = false;
      last = -1;
      for (; step <= max_face + max_gap; ++step) {
        int cx, cy;
        if (!rayCell(sx, sy, ux, uy, step, cx, cy)) return false;
        const int id = cy * width_ + cx;
        if (id == last) continue;
        last = id;
        const size_t sample = static_cast<size_t>(id);
        if (evidence[sample] & kGround) {
          crest_found = true;
          break;
        }
        if (cells_[sample].type == CellType::OCCUPIED) return false;
        ++gap;
      }
      if (!crest_found || gap < wave_config.front_min_hidden_cells ||
          gap > max_gap)
        return false;
      // The next crest top is visible again at the first crest's height, and
      // it is a short patch: a plateau that continues beyond the scan limit is
      // a ramp/step top, not a wave.
      int crest_cells = 0;
      bool crest_ended = false;
      last = -1;
      for (int i = 0; i < max_crest; ++i, ++step) {
        int cx, cy;
        if (!rayCell(sx, sy, ux, uy, step, cx, cy)) break;
        const int id = cy * width_ + cx;
        if (id == last) continue;
        last = id;
        const size_t sample = static_cast<size_t>(id);
        if (!(evidence[sample] & kGround)) {
          crest_ended = true;
          break;
        }
        if (std::abs(ground_heights[sample] - face_top) > crest_tolerance) {
          crest_ended = true;
          break;
        }
        cells.push_back(id);
        ++crest_cells;
      }
      return crest_ended &&
             crest_cells >= wave_config.front_min_crest_cells;
    };

    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const size_t index = static_cast<size_t>(y) * width_ + x;
        if (labels[index] != kSlope) continue;
        const double ux = geometry.uphill_x[index];
        const double uy = geometry.uphill_y[index];
        const double norm = std::hypot(ux, uy);
        if (!(norm > 1e-6) || !std::isfinite(norm)) continue;
        const double ux_unit = ux / norm;
        const double uy_unit = uy / norm;
        double center_top = 0.0;
        if (!signature(x, y, ux_unit, uy_unit, center_top, center_cells))
          continue;
        const int perp_x = static_cast<int>(std::lround(-uy_unit));
        const int perp_y = static_cast<int>(std::lround(ux_unit));
        int support = 1;
        supported_cells.assign(center_cells.begin(), center_cells.end());
        for (const int offset : {-1, 1}) {
          const int sx = x + offset * perp_x;
          const int sy = y + offset * perp_y;
          if (sx < 0 || sy < 0 || sx >= width_ || sy >= height_) continue;
          double side_top = 0.0;
          if (!signature(sx, sy, ux_unit, uy_unit, side_top, side_cells))
            continue;
          if (std::abs(side_top - center_top) > crest_tolerance) continue;
          ++support;
          supported_cells.insert(supported_cells.end(), side_cells.begin(),
                                 side_cells.end());
        }
        if (support < wave_config.front_min_lateral_support) continue;
        for (const int id : supported_cells) {
          const size_t sample = static_cast<size_t>(id);
          const uint8_t label = labels[sample];
          if (label == kUnknown || label == kFlat || label == kSlope)
            labels[sample] = 7U;
        }
      }
    }
  }
  return geometry;
}

std::vector<uint8_t> ProjectionLayer::terrainLabels() const {
  return terrainGeometry().labels;
}

} // namespace rog_map
