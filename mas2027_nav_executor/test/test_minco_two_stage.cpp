#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "mas2027_nav_executor/path_planner/trajectory/minco_optimizer.hpp"

namespace {

class FreeMap final : public rog_map::MapQueryInterface
{
public:
  bool worldToMap(double, double, unsigned int & mx, unsigned int & my) const override
  { mx = my = 0U; return true; }
  void mapToWorld(unsigned int, unsigned int, double & wx, double & wy) const override
  { wx = wy = 0.0; }
  unsigned int sizeX() const override { return 1U; }
  unsigned int sizeY() const override { return 1U; }
  double resolution() const override { return 0.05; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  uint8_t value(unsigned int, unsigned int) const override { return 0U; }
  const unsigned char * values() const override { static const unsigned char data = 0U; return &data; }
  bool isValid(unsigned int, unsigned int) const override { return true; }
  bool isFree(unsigned int, unsigned int) const override { return true; }
  rog_map::QueryResult query(const Eigen::Vector3d &) const override
  {
    rog_map::QueryResult result;
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    result.distance = 0.80;
    result.gradient.setZero();
    return result;
  }
  bool evaluate(const Eigen::Vector3d &, double & dist,
    Eigen::Vector3d & gradient) const override
  {
    const auto result = query(Eigen::Vector3d::Zero());
    dist = result.distance;
    gradient = result.gradient;
    return true;
  }
};

minco_planner::MincoOptimizer::Config config()
{
  minco_planner::MincoOptimizer::Config cfg;
  cfg.safe_dist = 0.25;
  cfg.max_vel = 1.0;
  cfg.max_acc = 2.0;
  cfg.turn_angle_deadzone = 0.0;
  cfg.turn_angle_saturation = 1.57;
  cfg.min_turn_vel = 0.1;
  cfg.decay_power = 1.0;
  cfg.rho = 100.0;
  cfg.smooth_eps = 0.01;
  cfg.integral_res = 8;
  cfg.opt_accuracy = 1.0e-3;
  cfg.print_optimizer_log = false;
  cfg.two_stage_enable = true;
  cfg.stage1_time_balance_weight = 10.0;
  cfg.stage1_time_balance_ratio = 0.10;
  cfg.stage2_directional_gradient_enable = true;
  cfg.penaltyWeights.resize(5);
  cfg.penaltyWeights << 100.0, 100.0, 100.0, 10.0, 10.0;
  cfg.magnitudeBounds.resize(3);
  cfg.magnitudeBounds << 0.25, 1.0, 2.0;
  return cfg;
}

TEST(MincoTwoStage, ProducesFiniteTrajectoryFromColdSeed)
{
  auto map = std::make_shared<FreeMap>();
  minco_planner::MincoOptimizer optimizer(config());
  optimizer.setMap(map);

  const std::vector<Eigen::Vector3d> waypoints{
    {0.0, 0.0, 0.0}, {0.8, 0.1, 0.0}, {1.6, 0.0, 0.0}};
  Eigen::Matrix3d start = Eigen::Matrix3d::Zero();
  start.col(0) = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Matrix3d end = Eigen::Matrix3d::Zero();
  end.col(0) = waypoints.back();
  super_utils::VecDf local_magnitudes(2);
  local_magnitudes << 1.0, 1.0;

  traj_opt::Trajectory trajectory;
  const double cost = optimizer.optimizeTwoStage(
    waypoints, start, end, local_magnitudes, trajectory);
  EXPECT_TRUE(std::isfinite(cost));
  EXPECT_TRUE(optimizer.lastFineStageUsed());
  EXPECT_GT(trajectory.getTotalDuration(), 0.0);
  EXPECT_GE(trajectory.getPieceNum(), 1);
}

}  // namespace
