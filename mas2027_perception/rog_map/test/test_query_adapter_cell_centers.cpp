#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "rog_map/field_layer.hpp"
#include "rog_map/query_adapter.hpp"

namespace {

TEST(QueryAdapterCellCenters, ReturnsTheEdtValueAtEachCellCenter)
{
  constexpr int width = 11;
  constexpr int height = 11;
  constexpr double resolution = 0.05;
  constexpr int obstacle_x = 5;
  constexpr int obstacle_y = 5;
  std::vector<uint8_t> mask(width * height, 1U);
  mask[obstacle_y * width + obstacle_x] = 0U;

  auto field = std::make_shared<rog_map::DynamicLayer>();
  field->rebuild(width, height, resolution, Eigen::Vector2d(-1.0, 2.0),
    mask, 0.0);
  auto snapshot = std::make_shared<rog_map::MapSnapshot>();
  snapshot->width = width;
  snapshot->height = height;
  snapshot->resolution = resolution;
  snapshot->origin_x = -1.0;
  snapshot->origin_y = 2.0;
  snapshot->values.assign(width * height, 0U);
  snapshot->values[obstacle_y * width + obstacle_x] = 254U;
  snapshot->distances = field->distances();

  rog_map::QueryAdapter adapter;
  for (const auto interpolation : {
      rog_map::InterpolationMode::BILINEAR, rog_map::InterpolationMode::QUADRATIC}) {
    snapshot->interpolation = interpolation;
    adapter.update(snapshot, field);
    for (const auto cell : {std::pair<int, int>(5, 5), {6, 5}, {5, 6}, {8, 8}}) {
      double wx = 0.0;
      double wy = 0.0;
      adapter.mapToWorld(cell.first, cell.second, wx, wy);
      const auto result = adapter.query({wx, wy, 0.0});
      ASSERT_TRUE(result.ok);
      EXPECT_NEAR(result.distance,
        snapshot->distances[cell.second * width + cell.first], 1e-9);
      double direct_distance = 0.0;
      Eigen::Vector3d direct_gradient = Eigen::Vector3d::Zero();
      field->evaluate({wx, wy, 0.0}, direct_distance, direct_gradient);
      EXPECT_NEAR(direct_distance,
        snapshot->distances[cell.second * width + cell.first], 1e-9);
    }
  }

  double wx = 0.0;
  double wy = 0.0;
  adapter.mapToWorld(obstacle_x, obstacle_y, wx, wy);
  EXPECT_LT(adapter.query({wx, wy, 0.0}).distance, 0.0);
}

}  // namespace
