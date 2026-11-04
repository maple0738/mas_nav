#include "mas2027_nav_executor/common/environment/online_global_map.hpp"

#include <algorithm>
#include <cstring>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>
#include "rclcpp/time.hpp"
#include "rog_map/query_adapter.hpp"
#include "mas2027_nav_executor/common/environment/terrain_semantics.hpp"
#include "mas2027_nav_executor/common/environment/clearance_gate.hpp"

namespace mas2027_nav_executor {
namespace {

constexpr double kMaxDistance = 3.0;
constexpr double kMinDistance = -1.0;
constexpr int kPadding = 2;

bool latticeIndex(double coordinate, double resolution, int & index)
{
  const double scaled = std::floor(coordinate / resolution);
  // Leave room for padding and checked integer arithmetic during growth.
  if (!std::isfinite(scaled) || std::abs(scaled) > 1000000000.0) return false;
  index = static_cast<int>(scaled);
  return true;
}

std::shared_ptr<const std::vector<double>> buildField(const nav_msgs::msg::OccupancyGrid & grid,
  double resolution)
{
  cv::Mat free_mask(static_cast<int>(grid.info.height), static_cast<int>(grid.info.width), CV_8UC1);
  for (size_t i = 0; i < grid.data.size(); ++i) free_mask.data[i] = grid.data[i] == 0 ? 255U : 0U;
  cv::Mat occupied_mask, positive, negative;
  cv::bitwise_not(free_mask, occupied_mask);
  cv::distanceTransform(free_mask, positive, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  cv::distanceTransform(occupied_mask, negative, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  auto distances = std::make_shared<std::vector<double>>(grid.data.size());
  for (size_t i = 0; i < distances->size(); ++i) {
    const double distance = grid.data[i] == 0 ?
      positive.ptr<float>()[i] * resolution : -negative.ptr<float>()[i] * resolution;
    (*distances)[i] = std::clamp(distance, kMinDistance, kMaxDistance);
  }
  return distances;
}

// A quantized 50 mm column can sit one voxel below its true surface, so two
// neighboring columns may differ by more than the step threshold even on a
// continuous grade. Around a direct height edge, test the independently
// measured precise ground on BOTH sides with a local plane. Repeated waves
// fit a plane within a few centimetres; a real step or one-sided return does
// not. The edge cell itself is excluded: its height is exactly what is under
// suspicion. This never uses ray-only support or the semantic terrain label.
bool smoothMeasuredGround(const rog_map::MapSnapshot & source, int sx, int sy,
  const std::function<bool(int, int)> & fresh_ground)
{
  if (source.ground_heights.empty()) return false;
  constexpr int radius = 4;  // 0.2 m at the 50 mm input resolution.
  constexpr double max_gap = 0.030;
  constexpr double max_range = 0.090;
  constexpr double max_slope = 0.70;
  Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  std::vector<double> heights;
  std::vector<double> bearings;
  double low = std::numeric_limits<double>::infinity();
  double high = -low;
  int count = 0;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int x = sx + dx, y = sy + dy;
      if (x < 0 || y < 0 || x >= source.width || y >= source.height) continue;
      const size_t i = static_cast<size_t>(y) * source.width + x;
      const uint8_t raw = source.raw_types.empty() ? source.types[i] : source.raw_types[i];
      // Enclosure may use rounded free/ground columns, but never a real
      // obstacle column or a ray-only unknown cell.
      if (raw != 3U && std::isfinite(source.ground_heights[i]) && (dx || dy))
        bearings.push_back(std::atan2(dy, dx));
      if (!fresh_ground(x, y)) continue;
      if (!source.height_discontinuities.empty() && source.height_discontinuities[i] != 0U) continue;
      if (source.ground_precise.empty() || source.ground_precise[i] != 1U) continue;
      const double z = source.ground_heights[i];
      if (!std::isfinite(z)) continue;
      const Eigen::Vector3d row(dx * source.resolution, dy * source.resolution, 1.0);
      normal.noalias() += row * row.transpose();
      rhs.noalias() += row * z;
      heights.push_back(z);
      low = std::min(low, z); high = std::max(high, z);
      ++count;
    }
  }
  if (count < 8 || bearings.size() < 8) return false;
  // Reject a one-sided scan: support must surround the cell, not just form a
  // line or half-plane whose two levels could masquerade as a grade.
  std::sort(bearings.begin(), bearings.end());
  constexpr double pi = 3.14159265358979323846;
  double angular_gap = bearings.front() + 2 * pi - bearings.back();
  for (size_t i = 1; i < bearings.size(); ++i)
    angular_gap = std::max(angular_gap, bearings[i] - bearings[i - 1]);
  if (angular_gap >= pi - 1e-6) return false;
  // A continuous grade spans its heights densely. A real step leaves a wide
  // empty interval between the two levels; excluding the suspect edge column
  // prevents its stale rounded value from filling that interval.
  std::sort(heights.begin(), heights.end());
  double largest_gap = 0.0;
  for (size_t i = 1; i < heights.size(); ++i)
    largest_gap = std::max(largest_gap, heights[i] - heights[i - 1]);
  if (largest_gap > max_gap || high - low > max_range) return false;
  if (std::abs(normal.determinant()) < 1e-9) return false;
  const Eigen::Vector3d plane = normal.fullPivLu().solve(rhs);
  return plane.allFinite() && plane.head<2>().norm() <= max_slope;
}

}  // namespace

bool OnlineGlobalMapSnapshot::worldToMap(double x, double y, unsigned int & mx,
  unsigned int & my) const
{
  const double fx = (x - originX()) / resolution_, fy = (y - originY()) / resolution_;
  if (!std::isfinite(fx) || !std::isfinite(fy) || fx < 0 || fy < 0 ||
    fx >= sizeX() || fy >= sizeY()) return false;
  mx = static_cast<unsigned int>(std::floor(fx));
  my = static_cast<unsigned int>(std::floor(fy));
  return true;
}

nav_msgs::msg::OccupancyGrid OnlineGlobalMapSnapshot::terrainGrid() const
{
  auto result = grid_;
  result.data.resize(terrain_labels_.size());
  for (size_t i = 0; i < terrain_labels_.size(); ++i) {
    result.data[i] = terrain_labels_[i] == 255U ? -1 :
      static_cast<int8_t>(terrain_labels_[i]);
  }
  return result;
}

sensor_msgs::msg::Image OnlineGlobalMapSnapshot::uphillImage() const
{
  sensor_msgs::msg::Image image;
  image.header = grid_.header;
  image.width = grid_.info.width;
  image.height = grid_.info.height;
  image.encoding = "32FC2";
  image.is_bigendian = false;
  image.step = image.width * 2U * sizeof(float);
  image.data.resize(static_cast<size_t>(image.step) * image.height);
  for (size_t i = 0; i < uphill_x_.size(); ++i) {
    std::memcpy(image.data.data() + i * 2 * sizeof(float), &uphill_x_[i], sizeof(float));
    std::memcpy(image.data.data() + (i * 2 + 1) * sizeof(float), &uphill_y_[i], sizeof(float));
  }
  return image;
}

nav_msgs::msg::OccupancyGrid OnlineGlobalMapSnapshot::heightGrid() const
{
  auto result = grid_;
  for (size_t i = 0; i < result.data.size(); ++i)
    result.data[i] = height_discontinuity_mask_[i] ? 100 : 0;
  return result;
}

bool OnlineGlobalMapSnapshot::nearWave(double x, double y, double radius) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(radius) || radius < 0.0)
    return false;
  // A confirmed wave fixes a corridor for the rest of its known length. This
  // is the map-side counterpart of the region controller's alignment latch:
  // later sparse or missing labels must not re-open height barriers.
  if (wave_latch_ && wave_latch_length_ > 0.0 && wave_latch_half_width_ > 0.0) {
    const Eigen::Vector2d delta(x - wave_latch_start_.x(), y - wave_latch_start_.y());
    const double along = delta.dot(wave_latch_dir_);
    const double lateral = std::abs(delta.x() * wave_latch_dir_.y() -
      delta.y() * wave_latch_dir_.x());
    if (along >= -0.2 && along <= wave_latch_length_ && lateral <= wave_latch_half_width_)
      return true;
  }
  if (terrain_labels_.empty()) return false;
  unsigned int mx = 0, my = 0;
  if (!worldToMap(x, y, mx, my)) return false;
  const int cells = std::max(1, static_cast<int>(std::ceil(radius / resolution_)));
  for (int dy = -cells; dy <= cells; ++dy) for (int dx = -cells; dx <= cells; ++dx) {
    const int nx = static_cast<int>(mx) + dx, ny = static_cast<int>(my) + dy;
    if (nx < 0 || ny < 0 || nx >= static_cast<int>(sizeX()) || ny >= static_cast<int>(sizeY()))
      continue;
    if (terrain_labels_[static_cast<size_t>(ny) * sizeX() + nx] == 7U) return true;
  }
  return false;
}

bool OnlineGlobalMapSnapshot::terrainTransition(double from_x, double from_y,
  double to_x, double to_y, double height_clearance,
  std::optional<Eigen::Vector2d> clearance_origin) const
{
  if (!terrainSideTransitionAllowed(static_cast<int>(sizeX()),
    static_cast<int>(sizeY()), resolution_, originX(), originY(), slope_side_mask_,
    from_x, from_y, to_x, to_y, false)) return false;
  // A confirmed repeated-wave patch carries continuous thin ground. Its
  // quantized height jumps are not steps; keep the occupancy and slope-side
  // masks authoritative and skip only height-discontinuity evidence here.
  const bool wave = nearWave(from_x, from_y, 0.80) && nearWave(to_x, to_y, 0.80);
  if (!wave && !terrainSideTransitionAllowed(static_cast<int>(sizeX()),
      static_cast<int>(sizeY()), resolution_, originX(), originY(), height_discontinuity_mask_,
      from_x, from_y, to_x, to_y, false)) return false;
  if (wave || height_clearance <= 0.0) return true;
  const auto initial = clearance_origin ? queryHeight(
    {clearance_origin->x(), clearance_origin->y(), 0.0}) : rog_map::QueryResult{};
  // Reuse the existing near-field rule when new observations find the robot
  // already closer to an edge: move out without worsening clearance, then
  // restore the full threshold outside one safety radius. Cell crossings
  // remain forbidden, including within this recovery interval.
  const auto gate = makeClearanceRequirement(height_clearance, height_clearance,
    initial.distance, initial.ok, 0.0);
  const int steps = std::max(1, static_cast<int>(std::ceil(
    std::hypot(to_x-from_x, to_y-from_y) / (0.5 * resolution_))));
  for (int i = 0; i <= steps; ++i) {
    const double u = static_cast<double>(i) / steps;
    const double x = from_x + u*(to_x-from_x), y = from_y + u*(to_y-from_y);
    const auto q = queryHeight({x, y, 0.0});
    if (q.status == rog_map::QueryStatus::OUT_OF_MAP) continue;
    const double required = clearance_origin ? gate.requiredAt(
      std::hypot(x-clearance_origin->x(), y-clearance_origin->y())) : height_clearance;
    if (!q.ok || q.distance + kClearanceComparisonTolerance < required) return false;
  }
  return true;
}

void OnlineGlobalMapSnapshot::mapToWorld(unsigned int mx, unsigned int my, double & x,
  double & y) const
{
  x = originX() + (mx + 0.5) * resolution_;
  y = originY() + (my + 0.5) * resolution_;
}

bool OnlineGlobalMapSnapshot::isValid(unsigned int mx, unsigned int my) const
{ return mx < sizeX() && my < sizeY(); }

uint8_t OnlineGlobalMapSnapshot::value(unsigned int mx, unsigned int my) const
{
  if (!isValid(mx, my)) return 254U;
  const int8_t occupancy = grid_.data[static_cast<size_t>(my) * sizeX() + mx];
  return occupancy < 0 ? 255U : (occupancy == 0 ? 0U : 254U);
}

bool OnlineGlobalMapSnapshot::isFree(unsigned int mx, unsigned int my) const
{ return isValid(mx, my) && value(mx, my) == 0U; }

bool OnlineGlobalMapSnapshot::copyValues(std::vector<unsigned char> & out) const
{
  out.resize(grid_.data.size());
  for (size_t i = 0; i < out.size(); ++i) {
    const int8_t occupancy = grid_.data[i];
    out[i] = occupancy < 0 ? 255U : (occupancy == 0 ? 0U : 254U);
  }
  return !out.empty();
}

rog_map::QueryResult OnlineGlobalMapSnapshot::query(const Eigen::Vector3d & position) const
{
  return queryField(position, distances_);
}

rog_map::QueryResult OnlineGlobalMapSnapshot::queryHeight(const Eigen::Vector3d & position) const
{
  auto result = queryField(position, height_distances_);
  if (result.ok) result.distance -= 0.5 * resolution_; // Cell extent, not only its center.
  return result;
}

rog_map::QueryResult OnlineGlobalMapSnapshot::queryField(const Eigen::Vector3d & position,
  const std::shared_ptr<const std::vector<double>> & field) const
{
  rog_map::QueryResult result;
  if (!position.allFinite()) {
    result.status = rog_map::QueryStatus::NONFINITE_INPUT;
    return result;
  }
  if (!field || field->size() != grid_.data.size()) return result;
  // Distance samples lie at cell centers, as in TerrainMapQuery.
  const double px = (position.x() - originX()) / resolution_ - 0.5;
  const double py = (position.y() - originY()) / resolution_ - 0.5;
  if (!std::isfinite(px) || !std::isfinite(py) || px < 0 || py < 0 ||
    px >= static_cast<double>(sizeX()) - 1 || py >= static_cast<double>(sizeY()) - 1) {
    result.status = rog_map::QueryStatus::OUT_OF_MAP;
    return result;
  }
  const auto x = static_cast<unsigned int>(std::floor(px));
  const auto y = static_cast<unsigned int>(std::floor(py));
  const size_t i = static_cast<size_t>(y) * sizeX() + x;
  const double fx = px - x, fy = py - y;
  const auto & d = *field;
  const double a = d[i], b = d[i + 1], c = d[i + sizeX()], e = d[i + sizeX() + 1];
  result.distance = (1 - fy) * ((1 - fx) * a + fx * b) + fy * ((1 - fx) * c + fx * e);
  result.gradient = {((1 - fy) * (b - a) + fy * (e - c)) / resolution_,
    ((1 - fx) * (c - a) + fx * (e - b)) / resolution_, 0.0};
  result.snapshot_sequence = revision_;
  result.field_sequence = revision_;
  result.ok = std::isfinite(result.distance) && result.gradient.allFinite();
  result.status = result.ok ? rog_map::QueryStatus::OK : rog_map::QueryStatus::NONFINITE_OUTPUT;
  return result;
}

bool OnlineGlobalMapSnapshot::evaluate(const Eigen::Vector3d & position, double & distance,
  Eigen::Vector3d & gradient) const
{
  const auto result = query(position);
  distance = result.distance;
  gradient = result.gradient;
  return result.ok;
}

OnlineGlobalMap::OnlineGlobalMap(const OnlineGlobalMapConfig & config, std::string frame)
: config_(config), frame_(std::move(frame))
{
  if (!std::isfinite(config.resolution) || config.resolution <= 0 || config.max_cells < 25 ||
    config.max_cells > 20000000 || config.occupied_increment <= 0 || config.free_decrement <= 0 ||
    config.evidence_limit <= 0 || config.evidence_limit > 127 ||
    config.occupied_increment > config.evidence_limit || config.free_decrement > config.evidence_limit ||
    config.occupied_confirmations < 1 || config.occupied_confirmations > 255 ||
    config.terrain_confirmations < 1 || config.terrain_confirmations > 255 ||
    config.terrain_clear_confirmations < 1 || config.terrain_clear_confirmations > 255 ||
    !std::isfinite(config.confirmation_max_gap_s) || config.confirmation_max_gap_s <= 0 ||
    frame_.empty()) throw std::invalid_argument("invalid online global map configuration");
}

std::shared_ptr<OnlineGlobalMapSnapshot> OnlineGlobalMap::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

void OnlineGlobalMap::setWaveTravelDirection(const Eigen::Vector2d & direction)
{
  if (!direction.allFinite() || direction.norm() < 1e-9) return;
  std::lock_guard<std::mutex> lock(wave_direction_mutex_);
  wave_travel_direction_ = direction.normalized();
  wave_travel_direction_valid_ = true;
}

OnlineGlobalMap::UpdateResult OnlineGlobalMap::update(const rog_map::MapSnapshot & source)
{
  if (source.width <= 0 || source.height <= 0 || !std::isfinite(source.resolution) ||
    source.resolution <= 0 || !std::isfinite(source.stamp) || source.stamp <= 0 ||
    source.stamp >= std::numeric_limits<int32_t>::max() ||
    source.sequence == 0 || !std::isfinite(source.origin_x) || !std::isfinite(source.origin_y)) {
    return UpdateResult::INVALID_INPUT;
  }
  const size_t source_count = static_cast<size_t>(source.width) * source.height;
  if (source_count > config_.max_cells || source.values.size() != source_count ||
    source.types.size() != source_count ||
    (!source.raw_types.empty() && source.raw_types.size() != source_count) ||
    (!source.height_deltas.empty() && source.height_deltas.size() != source_count) ||
    (!source.ground_heights.empty() && source.ground_heights.size() != source_count) ||
    (!source.ground_precise.empty() &&
      (source.ground_precise.size() != source_count || source.ground_heights.empty())) ||
    (!source.height_discontinuities.empty() && source.height_discontinuities.size() != source_count) ||
    ((!source.ground_heights.empty() || !source.height_discontinuities.empty()) &&
      (!std::isfinite(source.max_step_height) || source.max_step_height <= 0.0)) ||
    (!source.confidence.empty() && source.confidence.size() != source_count) ||
    ((!source.terrain_labels.empty() || !source.uphill_x.empty() ||
       !source.uphill_y.empty()) &&
      (source.terrain_labels.size() != source_count ||
       source.uphill_x.size() != source_count || source.uphill_y.size() != source_count)))
    return UpdateResult::INVALID_INPUT;
  const double ratio = config_.resolution / source.resolution;
  // Conservative downsampling requires aligned, integer-sized source cells.
  if (ratio < 1 - 1e-6 || ratio > 32 || std::abs(ratio - std::round(ratio)) > 1e-6 ||
    std::abs(source.origin_x / source.resolution - std::round(source.origin_x / source.resolution)) > 1e-6 ||
    std::abs(source.origin_y / source.resolution - std::round(source.origin_y / source.resolution)) > 1e-6) {
    return UpdateResult::INVALID_INPUT;
  }
  int bx, by, ex, ey;
  if (!latticeIndex(source.origin_x + 0.5 * source.resolution, config_.resolution, bx) ||
    !latticeIndex(source.origin_y + 0.5 * source.resolution, config_.resolution, by) ||
    !latticeIndex(source.origin_x + (source.width - 0.5) * source.resolution, config_.resolution, ex) ||
    !latticeIndex(source.origin_y + (source.height - 0.5) * source.resolution, config_.resolution, ey)) {
    return UpdateResult::INVALID_INPUT;
  }
  const int ow = ex - bx + 1, oh = ey - by + 1;
  if (ow <= 0 || oh <= 0 || static_cast<uint64_t>(ow) * oh > config_.max_cells) {
    return UpdateResult::INVALID_INPUT;
  }
  struct Observation {
    unsigned int free_count{0};
    bool occupied{false};
    unsigned int ordinary_count{0};
    bool terrain_conflict{false};
    uint8_t terrain_label{255U};
    float uphill_x{0.0f};
    float uphill_y{0.0f};
    bool height_discontinuity{false};
    bool height_measured{false};
    // Occupied for a reason other than a gap-inferred height barrier.
    bool occupied_not_inferred{false};
    unsigned int continuous_ground_count{0};
    unsigned int measured_edges{0};
    unsigned int inferred_edges{0};
    unsigned int thick_edges{0};
    unsigned int real_occupied{0};
    unsigned int soft_occupied{0};
    bool clear_wave_barrier{false};
  };
  std::vector<Observation> observations(static_cast<size_t>(ow) * oh);
  const auto ground_history = snapshot();
  const auto fresh_ground = [&source](int x, int y) {
    if (source.ground_heights.empty() || x < 0 || y < 0 ||
      x >= source.width || y >= source.height) return false;
    const size_t i = static_cast<size_t>(y) * source.width + x;
    const uint8_t raw = source.raw_types.empty() ? source.types[i] : source.raw_types[i];
    const bool measured_support = raw == 2U ||
      (raw == 1U && !source.ground_precise.empty() && source.ground_precise[i] == 1U);
    return measured_support && source.types[i] != 3U && source.values[i] < 253U &&
      std::isfinite(source.ground_heights[i]) &&
      ((!source.ground_precise.empty() && source.ground_precise[i] == 1U) ||
       source.height_deltas.empty() || (std::isfinite(source.height_deltas[i]) &&
        source.height_deltas[i] <= source.max_step_height));
  };
  const auto continuous_ground = [&source, &fresh_ground, &ground_history](int x, int y) {
    if (fresh_ground(x, y)) {
      const double z = source.ground_heights[static_cast<size_t>(y) * source.width + x];
      bool dense = true;
      for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        dense = dense && fresh_ground(x + dx, y + dy);
        if (dense) dense = std::abs(source.ground_heights[
          static_cast<size_t>(y + dy) * source.width + x + dx] - z) <= source.max_step_height + 1e-6;
      }
      if (dense) return true;
    }
    if (!ground_history || source.ground_heights.empty()) return false;
    unsigned int hx, hy;
    if (!ground_history->worldToMap(source.origin_x + (x + 0.5) * source.resolution,
        source.origin_y + (y + 0.5) * source.resolution, hx, hy) ||
      !ground_history->heightDiscontinuityMask()[static_cast<size_t>(hy) * ground_history->sizeX() + hx])
      return false;
    // A bounded, measured non-planar patch can disprove a quantization edge.
    // Require a verified center and enclosing verified returns; semantic label
    // 7 alone never clears a barrier, and one-sided/unknown support cannot do so.
    const size_t center=static_cast<size_t>(y)*source.width+x;
    if (!source.ground_precise.empty() && source.ground_precise[center]==1U && fresh_ground(x,y)) {
      double low=source.ground_heights[center],high=low;
      int count=0;bool blocked=false;std::vector<double> bearings;
      for(int dy=-3;dy<=3;++dy) for(int dx=-3;dx<=3;++dx) {
        const int nx=x+dx,ny=y+dy;
        if(nx<0||ny<0||nx>=source.width||ny>=source.height) continue;
        const size_t i=static_cast<size_t>(ny)*source.width+nx;
        if(source.values[i]>=253U && source.types[i]==3U) blocked=true;
        if(source.ground_precise[i]!=1U || !fresh_ground(nx,ny)) continue;
        low=std::min(low,static_cast<double>(source.ground_heights[i]));
        high=std::max(high,static_cast<double>(source.ground_heights[i]));
        ++count;if(dx||dy) bearings.push_back(std::atan2(dy,dx));
      }
      if(!blocked && count>=6 && bearings.size()>=3 && high-low<=source.max_step_height+1e-6) {
        std::sort(bearings.begin(),bearings.end());
        constexpr double pi=3.14159265358979323846;
        double gap=bearings.front()+2*pi-bearings.back();
        for(size_t i=1;i<bearings.size();++i) gap=std::max(gap,bearings[i]-bearings[i-1]);
        if(gap<pi-1e-6) return true;
      }
    }
    // Lidar rings and the body blind spot need not produce a dense 3x3
    // return. Correct a remembered gap only with an enclosing, measured local
    // plane. One-sided support or the two levels of a step cannot clear it.
    const int radius = static_cast<int>(std::ceil(0.5 / source.resolution));
    std::vector<std::pair<Eigen::Vector3d, double>> samples;
    std::vector<double> angles;
    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (int dy = -radius; dy <= radius; ++dy) for (int dx = -radius; dx <= radius; ++dx) {
      if (!fresh_ground(x + dx, y + dy)) continue;
      const Eigen::Vector3d row(dx * source.resolution, dy * source.resolution, 1.0);
      const double z = source.ground_heights[
        static_cast<size_t>(y + dy) * source.width + x + dx];
      normal.noalias() += row * row.transpose(); rhs += row * z;
      samples.emplace_back(row, z);
      if (dx || dy) angles.push_back(std::atan2(dy, dx));
    }
    if (samples.size() < 6 || angles.size() < 3 || std::abs(normal.determinant()) < 1e-9) return false;
    std::sort(angles.begin(), angles.end());
    constexpr double pi = 3.14159265358979323846;
    double gap = angles.front() + 2 * pi - angles.back();
    for (size_t i = 1; i < angles.size(); ++i) gap = std::max(gap, angles[i] - angles[i - 1]);
    if (gap >= pi - 1e-6) return false;
    const Eigen::Vector3d plane = normal.fullPivLu().solve(rhs);
    if (!plane.allFinite() || plane.head<2>().norm() > 0.70) return false;
    for (const auto & sample : samples)
      if (std::abs(sample.second - plane.dot(sample.first)) > 0.5 * source.max_step_height)
        return false;
    return true;
  };
  for (int y = 0; y < source.height; ++y) {
    for (int x = 0; x < source.width; ++x) {
      const size_t i = static_cast<size_t>(y) * source.width + x;
      const uint8_t type = source.types[i];
      if (!source.ground_precise.empty() && source.ground_precise[i]>1U) return UpdateResult::INVALID_INPUT;
      if (type > 3U) return UpdateResult::INVALID_INPUT;
      if (!source.height_discontinuities.empty() && source.height_discontinuities[i] > 2U)
        return UpdateResult::INVALID_INPUT;
      const bool height_edge = !source.height_discontinuities.empty() &&
        source.height_discontinuities[i] != 0U && type == 3U && source.values[i] >= 253U;
      if (type == 0U) continue;  // Unknown != observed obstacle, even if its cost is 254.
      const uint8_t raw_type = source.raw_types.empty() ? type : source.raw_types[i];
      if (raw_type > 3U) return UpdateResult::INVALID_INPUT;
      // Filter-filled and held obstacles belong to current local safety, not
      // persistent sensor evidence. Ignore rather than reinterpret as free.
      if (!height_edge && (raw_type == 0U || (type == 3U && raw_type != 3U))) continue;
      // Projection morphology can fill an unobserved hole with an occupied
      // type. Its observation confidence remains zero: do not memorize it.
      if (!height_edge && !source.confidence.empty() && (!std::isfinite(source.confidence[i]) ||
        source.confidence[i] <= 0.0F)) continue;
      int gx, gy;
      if (!latticeIndex(source.origin_x + (x + 0.5) * source.resolution, config_.resolution, gx) ||
        !latticeIndex(source.origin_y + (y + 0.5) * source.resolution, config_.resolution, gy)) {
        return UpdateResult::INVALID_INPUT;
      }
      auto & observation = observations[static_cast<size_t>(gy - by) * ow + gx - bx];
      observation.height_discontinuity = observation.height_discontinuity || height_edge;
      observation.height_measured = observation.height_measured ||
        (height_edge && source.height_discontinuities[i] == 1U);
      if (height_edge) {
        if (source.height_discontinuities[i] == 1U) ++observation.measured_edges;
        else ++observation.inferred_edges;
        if (raw_type == 3U) ++observation.thick_edges;
      }
      if (!height_edge && continuous_ground(x, y)) ++observation.continuous_ground_count;
      if (!source.terrain_labels.empty() &&
          (source.terrain_labels[i] == 5U || source.terrain_labels[i] == 6U ||
           source.terrain_labels[i] == 7U)) {
        const uint8_t label = source.terrain_labels[i];
        if (observation.terrain_label != 255U && observation.terrain_label != label) {
          observation.terrain_conflict = true;
        }
        if (!observation.terrain_conflict &&
            (observation.terrain_label == 255U || observation.terrain_label == label)) {
          observation.terrain_label = label;
          if (label == 5U && std::isfinite(source.uphill_x[i]) &&
              std::isfinite(source.uphill_y[i])) {
            observation.uphill_x += source.uphill_x[i];
            observation.uphill_y += source.uphill_y[i];
          }
        }
      } else if (!source.terrain_labels.empty() && source.terrain_labels[i] == 0U &&
                 type != 3U && source.values[i] < 253U) {
        ++observation.ordinary_count;
      }
      if (type == 3U || source.values[i] >= 253U) {
        observation.occupied = true;
        if (!height_edge) {
          if (raw_type == 3U) ++observation.real_occupied;
          else ++observation.soft_occupied;
        }
        const bool inferred_only = height_edge && source.height_discontinuities[i] == 2U &&
          raw_type != 3U;
        observation.occupied_not_inferred = observation.occupied_not_inferred || !inferred_only;
      }
      else ++observation.free_count;
    }
  }
  for (auto & observation : observations) {
    if (observation.terrain_conflict) {
      observation.terrain_label = 255U;
      observation.uphill_x = 0.0F;
      observation.uphill_y = 0.0F;
    }
  }
  const auto free_coverage = static_cast<unsigned int>(std::round(ratio) * std::round(ratio));
  // A direct edge between two rounded columns, or a barrier left by one, is
  // only kept after independently measured precise ground around it has been
  // checked. Repeated waves are smooth; a step, wall, or one-sided scan is not.
  // Collision occupancy stays authoritative: thick/real obstacles and inferred
  // gap edges are never eligible, and only thin measured ground is considered.
  for (int y = 0; y < oh; ++y) {
    for (int x = 0; x < ow; ++x) {
      auto & o = observations[static_cast<size_t>(y) * ow + x];
      if (o.thick_edges != 0 || o.real_occupied != 0 || o.inferred_edges != 0) continue;
      if (!o.height_discontinuity && o.soft_occupied == 0 &&
        o.free_count + o.measured_edges < free_coverage) continue;
      const double wx = (bx + x + 0.5) * config_.resolution;
      const double wy = (by + y + 0.5) * config_.resolution;
      bool remembered = false;
      if (ground_history) {
        unsigned int hx = 0, hy = 0;
        remembered = ground_history->worldToMap(wx, wy, hx, hy) &&
          ground_history->heightDiscontinuityMask()[
            static_cast<size_t>(hy) * ground_history->sizeX() + hx] != 0U;
      }
      if (!o.height_discontinuity && !remembered) continue;
      const int sx = static_cast<int>(std::floor((wx - source.origin_x) / source.resolution));
      const int sy = static_cast<int>(std::floor((wy - source.origin_y) / source.resolution));
      // The repeated-wave detector is a second, independent witness. It only
      // covers verified alternating profiles, so a single step or wall cannot
      // acquire it. Check the current scan and the confirmed history labels.
      bool wave_support = false;
      if (!source.terrain_labels.empty()) {
        for (int dy = -8; dy <= 8 && !wave_support; ++dy) {
          for (int dx = -8; dx <= 8; ++dx) {
            const int nx = sx + dx, ny = sy + dy;
            if (nx < 0 || ny < 0 || nx >= source.width || ny >= source.height) continue;
            if (source.terrain_labels[static_cast<size_t>(ny) * source.width + nx] == 7U) {
              wave_support = true;
              break;
            }
          }
        }
      }
      if (!wave_support && ground_history) {
        for (int dy = -4; dy <= 4 && !wave_support; ++dy) {
          for (int dx = -4; dx <= 4; ++dx) {
            unsigned int hx = 0, hy = 0;
            if (!ground_history->worldToMap(wx + dx * config_.resolution,
                wy + dy * config_.resolution, hx, hy)) continue;
            if (ground_history->terrainLabel(hx, hy) == 7U) {
              wave_support = true;
              break;
            }
          }
        }
      }
      if (!wave_support && !smoothMeasuredGround(source, sx, sy, fresh_ground)) continue;
      o.clear_wave_barrier = wave_support;
      if (o.height_discontinuity) {
        o.height_discontinuity = false;
        o.height_measured = false;
        o.measured_edges = 0;
        o.occupied = false;
      }
      // A rounded type-3 cell without a measured height edge is likely mask
      // inflation from a neighboring obstacle. Clear only its stale height
      // barrier; keep its occupancy so collision safety is unchanged.
      if (!o.occupied) {
        o.continuous_ground_count = std::max(o.continuous_ground_count, free_coverage);
        o.free_count = std::max(o.free_count, free_coverage);
      }
    }
  }
  // Confirm once, latch a world-fixed corridor for the known segment length.
  // The field's undulating strip is fixed length; requiring a label on every
  // cell made the last metres depend on sensor view and reopened old barriers.
  if (!source.terrain_labels.empty()) {
    std::vector<Eigen::Vector2d> wave_points;
    for (int y = 0; y < source.height; ++y) for (int x = 0; x < source.width; ++x) {
      if (source.terrain_labels[static_cast<size_t>(y) * source.width + x] != 7U) continue;
      wave_points.emplace_back(source.origin_x + (x + 0.5) * source.resolution,
        source.origin_y + (y + 0.5) * source.resolution);
    }
    if (wave_points.size() >= 3) {
      Eigen::Vector2d mean = Eigen::Vector2d::Zero();
      for (const auto & point : wave_points) mean += point;
      mean /= static_cast<double>(wave_points.size());
      Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
      bool directed = false;
      {
        std::lock_guard<std::mutex> lock(wave_direction_mutex_);
        if (wave_travel_direction_valid_) {
          direction = wave_travel_direction_;
          directed = true;
        }
      }
      bool direction_ready = directed;
      if (!direction_ready) {
        Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
        for (const auto & point : wave_points) {
          const Eigen::Vector2d delta = point - mean;
          covariance.noalias() += delta * delta.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
        if (solver.info() == Eigen::Success) {
          direction = solver.eigenvectors().col(1);
          if (direction.allFinite() && direction.norm() > 1e-9) {
            direction.normalize();
            direction_ready = true;
          }
        }
      }
      if (direction_ready) {
        double low = std::numeric_limits<double>::infinity();
        double high = -low, half_width = 0.0;
        for (const auto & point : wave_points) {
          const Eigen::Vector2d delta = point - mean;
          const double along = delta.dot(direction);
          const double lateral = std::abs(delta.x() * direction.y() -
            delta.y() * direction.x());
          low = std::min(low, along); high = std::max(high, along);
          half_width = std::max(half_width, lateral);
        }
        if (directed || high - low >= 0.40) {
          // The field's strip has a known length. Once any part is confirmed,
          // extend forward from its entry by the traversal allowance rather
          // than waiting for labels at every metre.
          constexpr double kBackwardExtension = 1.0;
          constexpr double kForwardExtension = 3.8;
          wave_latch_active_ = true;
          wave_latch_start_ = mean + (low - kBackwardExtension) * direction;
          wave_latch_dir_ = direction;
          wave_latch_length_ = (high - low) + kBackwardExtension + kForwardExtension;
          wave_latch_half_width_ = std::min(0.75, std::max(0.55, half_width + 0.15));
          wave_latch_stamp_ = source.stamp;
        }
      }
    } else if (wave_latch_active_ && source.stamp - wave_latch_stamp_ > 30.0) {
      wave_latch_active_ = false;
    }
  } else if (wave_latch_active_ && source.stamp - wave_latch_stamp_ > 30.0) {
    wave_latch_active_ = false;
  }
  const auto in_wave_corridor = [this](double x, double y) {
    if (!wave_latch_active_ || wave_latch_length_ <= 0.0 || wave_latch_half_width_ <= 0.0)
      return false;
    const Eigen::Vector2d delta(x - wave_latch_start_.x(), y - wave_latch_start_.y());
    const double along = delta.dot(wave_latch_dir_);
    const double lateral = std::abs(delta.x() * wave_latch_dir_.y() -
      delta.y() * wave_latch_dir_.x());
    return along >= -0.2 && along <= wave_latch_length_ && lateral <= wave_latch_half_width_;
  };
  int min_x = ex + 1, min_y = ey + 1, max_x = bx - 1, max_y = by - 1;
  for (int y = 0; y < oh; ++y) {
    for (int x = 0; x < ow; ++x) {
      const auto & o = observations[static_cast<size_t>(y) * ow + x];
      const bool corridor = in_wave_corridor((bx + x + 0.5) * config_.resolution,
        (by + y + 0.5) * config_.resolution);
      if (!corridor && !o.occupied && o.free_count < free_coverage &&
        o.terrain_label == 255U) continue;
      min_x = std::min(min_x, bx + x); min_y = std::min(min_y, by + y);
      max_x = std::max(max_x, bx + x); max_y = std::max(max_y, by + y);
    }
  }
  std::lock_guard<std::mutex> update_lock(update_mutex_);
  const auto previous = snapshot();
  if (source.sequence <= last_source_sequence_ || source.stamp <= last_source_stamp_) {
    return UpdateResult::NO_NEW_OBSERVATION;
  }
  if (min_x > max_x) {
    last_source_sequence_ = source.sequence; last_source_stamp_ = source.stamp;
    return UpdateResult::NO_NEW_OBSERVATION;
  }
  const int nx = width_ ? std::min(origin_x_, min_x - kPadding) : min_x - kPadding;
  const int ny = height_ ? std::min(origin_y_, min_y - kPadding) : min_y - kPadding;
  const int right = std::max(width_ ? origin_x_ + width_ - 1 : max_x + kPadding, max_x + kPadding);
  const int top = std::max(height_ ? origin_y_ + height_ - 1 : max_y + kPadding, max_y + kPadding);
  const int width = right - nx + 1, height = top - ny + 1;
  if (static_cast<uint64_t>(width) * height > config_.max_cells) return UpdateResult::CELL_LIMIT;
  auto next_cells = std::vector<Cell>(static_cast<size_t>(width) * height);
  for (int y = 0; y < height_; ++y) {
    std::copy_n(cells_.begin() + static_cast<size_t>(y) * width_, width_,
      next_cells.begin() + static_cast<size_t>(y + origin_y_ - ny) * width + origin_x_ - nx);
  }
  for (int y = 0; y < oh; ++y) {
    for (int x = 0; x < ow; ++x) {
      auto & o = observations[static_cast<size_t>(y) * ow + x];
      const bool corridor = in_wave_corridor((bx + x + 0.5) * config_.resolution,
        (by + y + 0.5) * config_.resolution);
      if (!corridor && !o.occupied && o.free_count < free_coverage &&
        o.terrain_label == 255U) continue;
      auto & cell = next_cells[static_cast<size_t>(by + y - ny) * width + bx + x - nx];
      // The latched corridor is a confirmed fixed-length wave. Clear only its
      // height-jump evidence; real obstacle columns and their occupancy stay.
      if (corridor && o.thick_edges == 0 && o.real_occupied == 0) {
        // The latched strip is known drivable ground. Keep raw type-3 measured
        // obstacles, but a quantized/filled/inferred column must not push the
        // path toward the wave edge.
        cell.height_discontinuity = false;
        cell.height_clear_hits = 0;
        cell.last_height_clear_stamp = 0.0;
        o.height_discontinuity = false;
        o.height_measured = false;
        o.occupied = false;
        o.free_count = std::max(o.free_count, free_coverage);
        o.continuous_ground_count = std::max(o.continuous_ground_count, free_coverage);
      }
      // A measured drop is not a transient filled/held obstacle. Preserve it
      // through sliding and free rays; clear only after fresh, fully covered
      // continuous ground has been observed on consecutive snapshots.
      // A barrier bridged across a sparse FREE gap only says the edge lies
      // somewhere in that span (up to 1 m). The live layer keeps blocking the
      // whole span; history keeps only adjacent measured jumps. Latching the
      // span closed narrow landings: their interior is repainted every frame,
      // so it can never be re-observed as continuous ground and cleared.
      const bool inferred_only = o.height_discontinuity && !o.height_measured;
      if (o.clear_wave_barrier) {
        // The repeated-wave witness covers a whole local patch; waiting for
        // two consecutive frames would let a single spurious edge re-latch the
        // barrier. Collision occupancy is still handled independently.
        cell.height_discontinuity = false;
        cell.height_clear_hits = 0;
        cell.last_height_clear_stamp = 0.0;
      } else if (inferred_only) {
        // History and its clearing progress are intentionally unchanged.
      } else if (o.height_discontinuity) {
        cell.height_discontinuity = true;
        cell.height_clear_hits = 0;
        cell.last_height_clear_stamp = 0.0;
      } else if (cell.height_discontinuity && !o.occupied &&
        o.continuous_ground_count >= free_coverage) {
        const bool consecutive = cell.last_height_clear_stamp == last_source_stamp_ &&
          source.stamp - last_source_stamp_ <= config_.confirmation_max_gap_s;
        cell.height_clear_hits = consecutive ? static_cast<uint8_t>(std::min(
          config_.occupied_confirmations, static_cast<int>(cell.height_clear_hits) + 1)) : 1U;
        cell.last_height_clear_stamp = source.stamp;
        if (cell.height_clear_hits >= config_.occupied_confirmations)
          cell.height_discontinuity = false;
      } else {
        cell.height_clear_hits = 0;
        cell.last_height_clear_stamp = 0.0;
      }
      // Confirm special terrain over distinct, consecutive snapshots before
      // retaining it outside the live window. Unknown observations do not erase
      // history; complete ordinary observations can correct a false label.
      if (o.terrain_label != 255U) {
        cell.ordinary_hits = 0;
        if (cell.terrain_label == o.terrain_label) {
          cell.terrain_hits = 0;
          cell.pending_terrain_label = 255U;
        } else {
          const bool consecutive = cell.pending_terrain_label == o.terrain_label &&
            cell.last_terrain_observation_sequence == last_source_sequence_ &&
            source.stamp - last_source_stamp_ <= config_.confirmation_max_gap_s;
          cell.terrain_hits = consecutive ? static_cast<uint8_t>(
            std::min(config_.terrain_confirmations, static_cast<int>(cell.terrain_hits) + 1)) : 1U;
          cell.pending_terrain_label = o.terrain_label;
          cell.last_terrain_observation_sequence = source.sequence;
          if (cell.terrain_hits >= config_.terrain_confirmations) {
            cell.terrain_label = o.terrain_label;
            cell.terrain_hits = 0;
            cell.pending_terrain_label = 255U;
          }
        }
        if (cell.terrain_label == 5U && o.terrain_label == 5U &&
            std::hypot(o.uphill_x, o.uphill_y) > 0.5f) {
          const float length = std::hypot(o.uphill_x, o.uphill_y);
          cell.uphill_x = o.uphill_x / length;
          cell.uphill_y = o.uphill_y / length;
        }
      } else if (o.ordinary_count >= free_coverage) {
        cell.terrain_hits = 0;
        cell.pending_terrain_label = 255U;
        const bool consecutive = cell.last_terrain_observation_sequence == last_source_sequence_ &&
          source.stamp - last_source_stamp_ <= config_.confirmation_max_gap_s;
        cell.ordinary_hits = consecutive ? static_cast<uint8_t>(std::min(
          config_.terrain_clear_confirmations, static_cast<int>(cell.ordinary_hits) + 1)) : 1U;
        cell.last_terrain_observation_sequence = source.sequence;
        if (cell.ordinary_hits >= config_.terrain_clear_confirmations) {
          cell.terrain_label = 255U;
          cell.uphill_x = 0.0F;
          cell.uphill_y = 0.0F;
        }
      } else {
        cell.terrain_hits = 0;
        cell.pending_terrain_label = 255U;
        cell.ordinary_hits = 0;
      }
      if (!o.occupied && o.free_count < free_coverage) continue;
      // The live layer blocks a bridged gap; history keeps confirmed ground.
      if (inferred_only && !o.occupied_not_inferred) continue;
      if (o.occupied && !(cell.observed && cell.evidence >= 0)) {
        const bool consecutive = cell.last_pending_stamp == last_source_stamp_ &&
          source.stamp - cell.last_pending_stamp <= config_.confirmation_max_gap_s;
        cell.pending_hits = consecutive ? static_cast<uint8_t>(
          std::min(config_.occupied_confirmations, static_cast<int>(cell.pending_hits) + 1)) : 1U;
        cell.last_pending_stamp = source.stamp;
        if (cell.pending_hits < config_.occupied_confirmations) {
          // Pending obstacles must never leave an old free cell traversable.
          cell.observed = false;
          cell.evidence = 0;
          continue;
        }
      }
      cell.pending_hits = 0;
      cell.last_pending_stamp = 0.0;
      cell.observed = true;
      // Confirmed obstacles block even after many free observations.
      cell.evidence = o.occupied ? static_cast<int8_t>(std::min(config_.evidence_limit,
        std::max(0, static_cast<int>(cell.evidence)) + config_.occupied_increment)) :
        static_cast<int8_t>(std::max(-config_.evidence_limit,
        static_cast<int>(cell.evidence) - config_.free_decrement));
    }
  }
  auto next = std::make_shared<OnlineGlobalMapSnapshot>();
  next->revision_ = previous ? previous->revision_ + 1 : 1;
  next->resolution_ = config_.resolution; next->stamp_ = source.stamp;
  next->wave_latch_ = wave_latch_active_;
  next->wave_latch_start_ = wave_latch_start_;
  next->wave_latch_dir_ = wave_latch_dir_;
  next->wave_latch_length_ = wave_latch_length_;
  next->wave_latch_half_width_ = wave_latch_half_width_;
  auto & grid = next->grid_;
  grid.header.frame_id = frame_;
  grid.header.stamp = rclcpp::Time(static_cast<int64_t>(source.stamp * 1e9));
  grid.info.map_load_time = previous ? previous->grid_.info.map_load_time : grid.header.stamp;
  grid.info.width = static_cast<uint32_t>(width); grid.info.height = static_cast<uint32_t>(height);
  grid.info.resolution = static_cast<float>(config_.resolution);
  grid.info.origin.position.x = nx * config_.resolution;
  grid.info.origin.position.y = ny * config_.resolution;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(next_cells.size());
  next->terrain_labels_.resize(next_cells.size());
  next->uphill_x_.resize(next_cells.size());
  next->uphill_y_.resize(next_cells.size());
  next->height_discontinuity_mask_.resize(next_cells.size(), 0U);
  for (size_t i = 0; i < next_cells.size(); ++i) {
    const auto & cell = next_cells[i];
    grid.data[i] = !cell.observed ? -1 : (cell.evidence < 0 ? 0 : 100);
    next->height_discontinuity_mask_[i] = cell.height_discontinuity ? 1U : 0U;
    if (cell.height_discontinuity) grid.data[i] = 100;
    next->terrain_labels_[i] = cell.terrain_label;
    next->uphill_x_[i] = cell.uphill_x;
    next->uphill_y_[i] = cell.uphill_y;
  }
  next->slope_side_mask_ = buildSlopeSideMask(width, height,
    next->terrain_labels_, next->uphill_x_, next->uphill_y_);
  for (size_t i = 0; i < grid.data.size(); ++i) {
    if (next->slope_side_mask_[i]) grid.data[i] = 100;
  }
  if (previous && width == width_ && height == height_ && nx == origin_x_ && ny == origin_y_ &&
    grid.data == previous->grid_.data) next->distances_ = previous->distances_;
  else next->distances_ = buildField(grid, config_.resolution);
  if (previous && width == width_ && height == height_ && nx == origin_x_ && ny == origin_y_ &&
    next->height_discontinuity_mask_ == previous->height_discontinuity_mask_)
    next->height_distances_ = previous->height_distances_;
  else {
    auto height_grid = grid;
    for (size_t i = 0; i < height_grid.data.size(); ++i)
      height_grid.data[i] = next->height_discontinuity_mask_[i] ? 100 : 0;
    next->height_distances_ = buildField(height_grid, config_.resolution);
  }
  cells_ = std::move(next_cells); origin_x_ = nx; origin_y_ = ny; width_ = width; height_ = height;
  last_source_sequence_ = source.sequence; last_source_stamp_ = source.stamp;
  {
    std::lock_guard<std::mutex> snapshot_lock(mutex_);
    snapshot_ = std::move(next);
  }
  return UpdateResult::UPDATED;
}

}  // namespace mas2027_nav_executor
