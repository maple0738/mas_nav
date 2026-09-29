#include "mas2027_nav_executor/common/environment/fused_map_query.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace {

class TestQuery final : public rog_map::MapQueryInterface
{
public:
  TestQuery(unsigned int width, unsigned int height, double resolution)
  : width_(width), height_(height), resolution_(resolution), cells_(width * height, 0U) {}

  void block(unsigned int x, unsigned int y) {cells_[y * width_ + x] = 254U;}
  void setField(double distance, const Eigen::Vector3d & gradient)
  {
    distance_ = distance;
    gradient_ = gradient;
  }

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < 0.0 || wy < 0.0 ||
      wx >= width_ * resolution_ || wy >= height_ * resolution_)
    {
      return false;
    }
    mx = static_cast<unsigned int>(std::floor(wx / resolution_));
    my = static_cast<unsigned int>(std::floor(wy / resolution_));
    return true;
  }
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override
  {
    wx = (static_cast<double>(mx) + 0.5) * resolution_;
    wy = (static_cast<double>(my) + 0.5) * resolution_;
  }
  unsigned int sizeX() const override {return width_;}
  unsigned int sizeY() const override {return height_;}
  double resolution() const override {return resolution_;}
  double originX() const override {return 0.0;}
  double originY() const override {return 0.0;}
  double snapshotStampSeconds() const override {return 42.0;}
  uint8_t value(unsigned int mx, unsigned int my) const override
  {
    return isValid(mx, my) ? cells_[static_cast<size_t>(my) * width_ + mx] : 254U;
  }
  const unsigned char * values() const override {return cells_.data();}
  bool isValid(unsigned int mx, unsigned int my) const override
  {
    return mx < width_ && my < height_;
  }
  bool isFree(unsigned int mx, unsigned int my) const override
  {
    return isValid(mx, my) && value(mx, my) < 253U;
  }
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override
  {
    unsigned int mx = 0U;
    unsigned int my = 0U;
    if (!worldToMap(pos.x(), pos.y(), mx, my)) return false;
    dist = distance_;
    grad = gradient_;
    return std::isfinite(dist) && grad.allFinite();
  }

private:
  unsigned int width_;
  unsigned int height_;
  double resolution_;
  std::vector<unsigned char> cells_;
  double distance_{1.0};
  Eigen::Vector3d gradient_{Eigen::Vector3d::UnitX()};
};

TEST(FusedMapQuery, RequiresStaticTransform)
{
  auto dynamic_query = std::make_shared<TestQuery>(20U, 20U, 1.0);
  auto static_query = std::make_shared<TestQuery>(20U, 20U, 1.0);
  mas2027_nav_executor::FusedMapQuery fused(dynamic_query, static_query);
  const auto result = fused.query(Eigen::Vector3d(2.5, 2.5, 0.0));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.status, rog_map::QueryStatus::TF_FAILED);
}

TEST(FusedMapQuery, UsesMinimumClearanceAndRotatesStaticGradient)
{
  auto dynamic_query = std::make_shared<TestQuery>(20U, 20U, 1.0);
  dynamic_query->setField(2.0, Eigen::Vector3d(0.25, 0.5, 0.0));
  auto static_query = std::make_shared<TestQuery>(20U, 20U, 1.0);
  static_query->setField(0.4, Eigen::Vector3d::UnitX());
  mas2027_nav_executor::FusedMapQuery fused(dynamic_query, static_query);
  fused.setStaticTransform(5.0, 5.0, 1.5707963267948966);

  const auto result = fused.query(Eigen::Vector3d(1.5, 2.5, 0.0));
  ASSERT_TRUE(result.ok);
  EXPECT_NEAR(result.distance, 0.4, 1e-9);
  EXPECT_NEAR(result.gradient.x(), 0.0, 1e-9);
  EXPECT_NEAR(result.gradient.y(), -1.0, 1e-9);
  EXPECT_DOUBLE_EQ(fused.snapshotStampSeconds(), 42.0);
}

TEST(FusedMapQuery, KeepsDynamicResultWhenItIsCloser)
{
  auto dynamic_query = std::make_shared<TestQuery>(20U, 20U, 1.0);
  dynamic_query->setField(0.2, Eigen::Vector3d(0.25, 0.5, 0.0));
  auto static_query = std::make_shared<TestQuery>(20U, 20U, 1.0);
  static_query->setField(0.4, Eigen::Vector3d::UnitX());
  mas2027_nav_executor::FusedMapQuery fused(dynamic_query, static_query);
  fused.setStaticTransform(0.0, 0.0, 0.0);

  const auto result = fused.query(Eigen::Vector3d(2.5, 2.5, 0.0));
  ASSERT_TRUE(result.ok);
  EXPECT_NEAR(result.distance, 0.2, 1e-9);
  EXPECT_NEAR(result.gradient.x(), 0.25, 1e-9);
  EXPECT_NEAR(result.gradient.y(), 0.5, 1e-9);
}

TEST(FusedMapQuery, StaticObstacleMasksFreeDynamicCell)
{
  auto dynamic_query = std::make_shared<TestQuery>(6U, 4U, 1.0);
  auto static_query = std::make_shared<TestQuery>(8U, 4U, 1.0);
  static_query->block(2U, 1U);
  mas2027_nav_executor::FusedMapQuery fused(dynamic_query, static_query);
  // Dynamic cell (1,1) has center (1.5,1.5), transformed to static cell (2,1).
  fused.setStaticTransform(1.0, 0.0, 0.0);

  EXPECT_EQ(fused.value(1U, 1U), 254U);
  EXPECT_FALSE(fused.isFree(1U, 1U));
  EXPECT_TRUE(fused.isFree(0U, 1U));
  EXPECT_EQ(fused.values(), nullptr);

  std::vector<unsigned char> values;
  ASSERT_TRUE(fused.copyValues(values));
  ASSERT_EQ(values.size(), 24U);
  EXPECT_EQ(values[1U * 6U + 1U], 254U);
}

TEST(FusedMapQuery, FailsClosedOutsideStaticMap)
{
  auto dynamic_query = std::make_shared<TestQuery>(20U, 20U, 1.0);
  auto static_query = std::make_shared<TestQuery>(2U, 2U, 1.0);
  mas2027_nav_executor::FusedMapQuery fused(dynamic_query, static_query);
  fused.setStaticTransform(0.0, 0.0, 0.0);

  const auto result = fused.query(Eigen::Vector3d(5.5, 5.5, 0.0));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.status, rog_map::QueryStatus::INTERPOLATION_FAILED);
  EXPECT_EQ(fused.value(5U, 5U), 254U);
}

}  // namespace
