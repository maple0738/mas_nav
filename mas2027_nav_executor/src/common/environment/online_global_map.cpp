#include "mas2027_nav_executor/common/environment/online_global_map.hpp"

#include <algorithm>
#include <cstring>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>
#include "rclcpp/time.hpp"
#include "rog_map/query_adapter.hpp"
#include "mas2027_nav_executor/common/environment/terrain_semantics.hpp"

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

bool OnlineGlobalMapSnapshot::terrainTransition(double from_x, double from_y,
  double to_x, double to_y) const
{
  return terrainSideTransitionAllowed(static_cast<int>(sizeX()),
    static_cast<int>(sizeY()), resolution_, originX(), originY(), slope_side_mask_,
    from_x, from_y, to_x, to_y, false);
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
  rog_map::QueryResult result;
  if (!position.allFinite()) {
    result.status = rog_map::QueryStatus::NONFINITE_INPUT;
    return result;
  }
  if (!distances_ || distances_->size() != grid_.data.size()) return result;
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
  const auto & d = *distances_;
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
  };
  std::vector<Observation> observations(static_cast<size_t>(ow) * oh);
  for (int y = 0; y < source.height; ++y) {
    for (int x = 0; x < source.width; ++x) {
      const size_t i = static_cast<size_t>(y) * source.width + x;
      const uint8_t type = source.types[i];
      if (type > 3U) return UpdateResult::INVALID_INPUT;
      if (type == 0U) continue;  // Unknown != observed obstacle, even if its cost is 254.
      const uint8_t raw_type = source.raw_types.empty() ? type : source.raw_types[i];
      if (raw_type > 3U) return UpdateResult::INVALID_INPUT;
      // Filter-filled and held obstacles belong to current local safety, not
      // persistent sensor evidence. Ignore rather than reinterpret as free.
      if (raw_type == 0U || (type == 3U && raw_type != 3U)) continue;
      // Projection morphology can fill an unobserved hole with an occupied
      // type. Its observation confidence remains zero: do not memorize it.
      if (!source.confidence.empty() && (!std::isfinite(source.confidence[i]) ||
        source.confidence[i] <= 0.0F)) continue;
      int gx, gy;
      if (!latticeIndex(source.origin_x + (x + 0.5) * source.resolution, config_.resolution, gx) ||
        !latticeIndex(source.origin_y + (y + 0.5) * source.resolution, config_.resolution, gy)) {
        return UpdateResult::INVALID_INPUT;
      }
      auto & observation = observations[static_cast<size_t>(gy - by) * ow + gx - bx];
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
      if (type == 3U || source.values[i] >= 253U) observation.occupied = true;
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
  int min_x = ex + 1, min_y = ey + 1, max_x = bx - 1, max_y = by - 1;
  for (int y = 0; y < oh; ++y) {
    for (int x = 0; x < ow; ++x) {
      const auto & o = observations[static_cast<size_t>(y) * ow + x];
      if (!o.occupied && o.free_count < free_coverage && o.terrain_label == 255U) continue;
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
      const auto & o = observations[static_cast<size_t>(y) * ow + x];
      if (!o.occupied && o.free_count < free_coverage && o.terrain_label == 255U) continue;
      auto & cell = next_cells[static_cast<size_t>(by + y - ny) * width + bx + x - nx];
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
  for (size_t i = 0; i < next_cells.size(); ++i) {
    const auto & cell = next_cells[i];
    grid.data[i] = !cell.observed ? -1 : (cell.evidence < 0 ? 0 : 100);
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
  cells_ = std::move(next_cells); origin_x_ = nx; origin_y_ = ny; width_ = width; height_ = height;
  last_source_sequence_ = source.sequence; last_source_stamp_ = source.stamp;
  {
    std::lock_guard<std::mutex> snapshot_lock(mutex_);
    snapshot_ = std::move(next);
  }
  return UpdateResult::UPDATED;
}

}  // namespace mas2027_nav_executor
