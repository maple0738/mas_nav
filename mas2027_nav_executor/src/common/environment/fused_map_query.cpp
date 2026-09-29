#include "mas2027_nav_executor/common/environment/fused_map_query.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mas2027_nav_executor {

FusedMapQuery::FusedMapQuery(
  std::shared_ptr<rog_map::MapQueryInterface> dynamic_query,
  std::shared_ptr<rog_map::MapQueryInterface> static_query)
: dynamic_query_(std::move(dynamic_query)), static_query_(std::move(static_query))
{
}

void FusedMapQuery::setStaticTransform(
  double translation_x, double translation_y, double yaw)
{
  Transform2D next;
  if (std::isfinite(translation_x) && std::isfinite(translation_y) && std::isfinite(yaw)) {
    next.translation_x = translation_x;
    next.translation_y = translation_y;
    next.cosine = std::cos(yaw);
    next.sine = std::sin(yaw);
    next.valid = std::isfinite(next.cosine) && std::isfinite(next.sine);
  }
  std::lock_guard<std::mutex> lock(transform_mutex_);
  transform_ = next;
}

void FusedMapQuery::clearStaticTransform()
{
  std::lock_guard<std::mutex> lock(transform_mutex_);
  transform_ = Transform2D{};
}

bool FusedMapQuery::hasStaticTransform() const
{
  return transformSnapshot().valid;
}

FusedMapQuery::Transform2D FusedMapQuery::transformSnapshot() const
{
  std::lock_guard<std::mutex> lock(transform_mutex_);
  return transform_;
}

Eigen::Vector3d FusedMapQuery::toStatic(
  const Eigen::Vector3d & point, const Transform2D & transform) const
{
  return {
    transform.cosine * point.x() - transform.sine * point.y() + transform.translation_x,
    transform.sine * point.x() + transform.cosine * point.y() + transform.translation_y,
    point.z()};
}

Eigen::Vector3d FusedMapQuery::gradientToDynamic(
  const Eigen::Vector3d & gradient, const Transform2D & transform) const
{
  // static_position = R * dynamic_position + t, therefore
  // grad_dynamic = R^T * grad_static.
  return {
    transform.cosine * gradient.x() + transform.sine * gradient.y(),
    -transform.sine * gradient.x() + transform.cosine * gradient.y(),
    gradient.z()};
}

bool FusedMapQuery::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  return dynamic_query_ && dynamic_query_->worldToMap(wx, wy, mx, my);
}

void FusedMapQuery::mapToWorld(
  unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  if (!dynamic_query_) {
    wx = 0.0;
    wy = 0.0;
    return;
  }
  dynamic_query_->mapToWorld(mx, my, wx, wy);
}

unsigned int FusedMapQuery::sizeX() const
{
  return dynamic_query_ ? dynamic_query_->sizeX() : 0U;
}

unsigned int FusedMapQuery::sizeY() const
{
  return dynamic_query_ ? dynamic_query_->sizeY() : 0U;
}

double FusedMapQuery::resolution() const
{
  return dynamic_query_ ? dynamic_query_->resolution() : 0.0;
}

double FusedMapQuery::originX() const
{
  return dynamic_query_ ? dynamic_query_->originX() : 0.0;
}

double FusedMapQuery::originY() const
{
  return dynamic_query_ ? dynamic_query_->originY() : 0.0;
}

double FusedMapQuery::snapshotStampSeconds() const
{
  return dynamic_query_ ? dynamic_query_->snapshotStampSeconds()
                        : std::numeric_limits<double>::quiet_NaN();
}

bool FusedMapQuery::staticCellForDynamicCell(
  unsigned int mx, unsigned int my, unsigned int & static_mx, unsigned int & static_my) const
{
  if (!dynamic_query_ || !static_query_ || !dynamic_query_->isValid(mx, my)) {
    return false;
  }
  const Transform2D transform = transformSnapshot();
  if (!transform.valid) {
    return false;
  }
  double wx = 0.0;
  double wy = 0.0;
  dynamic_query_->mapToWorld(mx, my, wx, wy);
  const Eigen::Vector3d point_static = toStatic(Eigen::Vector3d(wx, wy, 0.0), transform);
  return static_query_->worldToMap(
    point_static.x(), point_static.y(), static_mx, static_my);
}

uint8_t FusedMapQuery::value(unsigned int mx, unsigned int my) const
{
  if (!dynamic_query_ || !static_query_ || !dynamic_query_->isValid(mx, my)) {
    return kLethalCost;
  }
  unsigned int static_mx = 0U;
  unsigned int static_my = 0U;
  if (!staticCellForDynamicCell(mx, my, static_mx, static_my) ||
    !static_query_->isFree(static_mx, static_my))
  {
    return kLethalCost;
  }
  return dynamic_query_->value(mx, my);
}

const unsigned char * FusedMapQuery::values() const
{
  // The fused grid depends on the current frame transform, so exposing the raw
  // dynamic buffer would silently omit static obstacles.
  return nullptr;
}

bool FusedMapQuery::copyValues(std::vector<unsigned char> & out) const
{
  const unsigned int width = sizeX();
  const unsigned int height = sizeY();
  if (width == 0U || height == 0U) {
    out.clear();
    return false;
  }
  out.resize(static_cast<size_t>(width) * height);
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      out[static_cast<size_t>(y) * width + x] = value(x, y);
    }
  }
  return true;
}

bool FusedMapQuery::isValid(unsigned int mx, unsigned int my) const
{
  return dynamic_query_ && dynamic_query_->isValid(mx, my);
}

bool FusedMapQuery::isFree(unsigned int mx, unsigned int my) const
{
  return isValid(mx, my) && value(mx, my) < kInscribedCost;
}

rog_map::QueryResult FusedMapQuery::query(const Eigen::Vector3d & pos) const
{
  rog_map::QueryResult result;
  if (!pos.allFinite()) {
    result.status = rog_map::QueryStatus::NONFINITE_INPUT;
    return result;
  }
  if (!dynamic_query_ || !static_query_) {
    result.status = rog_map::QueryStatus::SNAPSHOT_INVALID;
    return result;
  }
  const Transform2D transform = transformSnapshot();
  if (!transform.valid) {
    result.status = rog_map::QueryStatus::TF_FAILED;
    return result;
  }

  const rog_map::QueryResult dynamic_result = dynamic_query_->query(pos);
  if (!dynamic_result.ok) {
    return dynamic_result;
  }

  rog_map::QueryResult static_result = static_query_->query(toStatic(pos, transform));
  if (!static_result.ok) {
    return static_result;
  }
  static_result.gradient = gradientToDynamic(static_result.gradient, transform);
  if (!static_result.gradient.allFinite()) {
    static_result.ok = false;
    static_result.status = rog_map::QueryStatus::NONFINITE_OUTPUT;
    return static_result;
  }

  // Occupancy is the union of both maps, so signed clearance is their minimum.
  if (static_result.distance < dynamic_result.distance) {
    static_result.projection_sequence = dynamic_result.projection_sequence;
    static_result.mask_sequence = dynamic_result.mask_sequence;
    static_result.field_sequence = dynamic_result.field_sequence;
    static_result.snapshot_sequence = dynamic_result.snapshot_sequence;
    static_result.field_age_ms = dynamic_result.field_age_ms;
    return static_result;
  }
  return dynamic_result;
}

bool FusedMapQuery::evaluate(
  const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  const rog_map::QueryResult result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

}  // namespace mas2027_nav_executor
