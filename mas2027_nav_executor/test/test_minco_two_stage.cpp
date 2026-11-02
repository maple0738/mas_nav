#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "mas2027_nav_executor/path_planner/trajectory/minco_optimizer.hpp"
#include "mas2027_nav_executor/common/environment/terrain_semantics.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/safe_path_prefix.hpp"

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

TEST(MincoTwoStage, StoppedSeedPrefixKeepsCornersAndBoundsVelocityAndAcceleration)
{
  const std::vector<Eigen::Vector3d> path{{0,0,0},{.3,0,0},{.3,.5,0},{1,1,0}};
  std::vector<Eigen::Vector3d> prefix;
  const auto trajectory = mas2027_nav_executor::stoppedPathPrefix(path,.6,.5,1.2,prefix);
  ASSERT_EQ(trajectory.getPieceNum(),2);
  ASSERT_EQ(prefix.size(),3U);
  EXPECT_LT((prefix[1]-path[1]).norm(),1e-9);
  EXPECT_LT((prefix.back()-Eigen::Vector3d(.3,.3,0)).norm(),1e-9);
  double start=0;
  for (int i=0;i<trajectory.getPieceNum();++i) {
    EXPECT_LT(trajectory.getVel(start).norm(),1e-8);
    EXPECT_LT(trajectory.getAcc(start).norm(),1e-8);
    const double duration=trajectory[i].getDuration();
    for (int j=0;j<=100;++j) {
      const double t=start+duration*j/100.;
      EXPECT_LE(trajectory.getVel(t).norm(),.5+1e-9);
      EXPECT_LE(trajectory.getAcc(t).norm(),1.2+1e-9);
      const auto p=trajectory.getPos(t);
      if (i==0) EXPECT_NEAR(p.y(),0,1e-9);
      else EXPECT_NEAR(p.x(),.3,1e-9);
    }
    start+=duration;
  }
  EXPECT_LT(trajectory.getVel(start).norm(),1e-8);
  EXPECT_LT(trajectory.getAcc(start).norm(),1e-8);
  EXPECT_TRUE(mas2027_nav_executor::stoppedPathPrefix(path,.6,0,1.2,prefix).empty());
}

TEST(MincoTwoStage, SmoothingRespectsTheSameForbiddenCellsAsTheTerrainGate)
{
  constexpr int width = 48, height = 32;
  constexpr double res = 0.05, ox = -0.4, oy = -0.8;
  std::vector<uint8_t> mask(width * height, 0U);
  for (int y = 6; y <= 17; ++y) for (int x = 20; x <= 27; ++x) {
    mask[y * width + x] = 1U;
  }
  auto cfg = config();
  cfg.penaltyWeights[0] = 100000.0;
  const std::vector<Eigen::Vector3d> points{
    {0,0,0}, {0.45,0.20,0}, {0.8,0.25,0}, {1.15,0.20,0}, {1.6,0,0}};
  Eigen::Matrix3d start = Eigen::Matrix3d::Zero(), end = start;
  end.col(0) = points.back();
  super_utils::VecDf speeds = super_utils::VecDf::Constant(4, 1.0);
  const auto crosses_side = [&](const traj_opt::Trajectory & trajectory) {
    Eigen::Vector3d previous = trajectory.getPos(0);
    for (int i = 1; i <= 500; ++i) {
      const auto current = trajectory.getPos(trajectory.getTotalDuration() * i / 500.0);
      if (!mas2027_nav_executor::terrainSideTransitionAllowed(
        width, height, res, ox, oy, mask,
        previous.x(), previous.y(), current.x(), current.y(), true)) return true;
      previous = current;
    }
    return false;
  };
  minco_planner::MincoOptimizer optimizer(cfg);
  optimizer.setMap(std::make_shared<FreeMap>());
  traj_opt::Trajectory unconstrained, constrained;
  ASSERT_TRUE(std::isfinite(optimizer.optimizeTwoStage(points, start, end, speeds, unconstrained)));
  EXPECT_TRUE(crosses_side(unconstrained));
  optimizer.setPositionConstraint([&](const Eigen::Vector3d & p) {
    const auto side = mas2027_nav_executor::terrainSideDistance(
      width, height, res, ox, oy, mask, p.x(), p.y());
    rog_map::QueryResult result;
    result.ok = std::isfinite(side.distance);
    result.distance = cfg.safe_dist + side.distance - 0.5 * res;
    result.gradient = {side.gradient_x, side.gradient_y, 0.0};
    return result;
  });
  optimizer.setInitPsAndTs({}, {});
  ASSERT_TRUE(std::isfinite(optimizer.optimizeTwoStage(points, start, end, speeds, constrained)));
  EXPECT_FALSE(crosses_side(constrained));
}


TEST(MincoTwoStage, RunupExposureReachesBothOptimizerStages)
{
  for (const double distance : {0.2, 0.5, 0.85}) {
    auto cfg=config();cfg.runup.enable=true;
    cfg.runup.minimum_alignment_cosine = 0.85;
    cfg.runup.distance = distance;
    cfg.runup.transition = distance > 0.5 ? 0.2 : 0.1;
    cfg.runup.alignment_weight=100;cfg.runup.curvature_weight=100;
    minco_planner::MincoOptimizer optimizer(cfg);optimizer.setMap(std::make_shared<FreeMap>());
    int samples=0;
    optimizer.setTerrainSampler([&samples](double x,double) {
      ++samples;rog_map::TerrainDirectionSample sample;
      sample.known=true;sample.label=x>1.0?5:0;sample.uphill={1,0};
      // A continuous ramp entry with its exact derivative.
      const double t=std::tanh((x-1.0)/0.1);
      sample.body=0.5*(1+t);sample.body_gradient={5*(1-t*t),0};
      return sample;
    });
    const std::vector<Eigen::Vector3d> waypoints{{0,0,0},{0.7,0.15,0},{1.5,0,0},{2.2,0,0}};
    Eigen::Matrix3d start=Eigen::Matrix3d::Zero(),end=start;end.col(0)=waypoints.back();
    super_utils::VecDf limits(3);limits<<1,1,1;traj_opt::Trajectory trajectory;
    EXPECT_TRUE(std::isfinite(optimizer.optimizeTwoStage(waypoints,start,end,limits,trajectory)));
    EXPECT_TRUE(optimizer.lastFineStageUsed());EXPECT_GT(samples,100);
    EXPECT_LT((trajectory.getPos(trajectory.getTotalDuration())-waypoints.back()).norm(),1e-6);
    for(double t=0;t<trajectory.getTotalDuration();t+=0.05)
      EXPECT_TRUE(trajectory.getPos(t).allFinite());
  }
}
}  // namespace
