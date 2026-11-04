#include <gtest/gtest.h>

#include "mas2027_nav_executor/common/environment/history_frontier.hpp"
#include "mas2027_nav_executor/common/environment/online_global_map.hpp"
#include "mas2027_nav_executor/path_planner/search/smac/smac_planner_2d_simple.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/runup_cost.hpp"
#include "rog_map/query_adapter.hpp"

using namespace mas2027_nav_executor;
namespace {
rog_map::MapSnapshot ramp() {
  rog_map::MapSnapshot m;
  m.width = 80;
  m.height = 60;
  m.resolution = 0.1;
  m.sequence = 1;
  m.stamp = 1;
  const int n = m.width * m.height;
  m.values.assign(n, 0);
  m.types.assign(n, 1);
  m.terrain_labels.assign(n, 0);
  m.uphill_x.assign(n, 0);
  m.uphill_y.assign(n, 0);
  m.distances.assign(n, 1);
  m.field_stale = false;
  m.field_stamp = 1;
  m.field_sequence = 1;
  // 几字形通路：左坡上行、上方平台连接、右坡下行。
  // Only its two ramps connect the raised region to the foreground.
  // Close the platform's rear too; otherwise a longer runway can make an
  // entirely flat route around the back cheaper than either physical ramp.
  for (int y = 20; y < m.height; ++y)
    for (int x = 15; x < 65; ++x) {
      const int k = y * m.width + x;
      const bool left = x >= 20 && x < 30 && y >= 20 && y < 40;
      const bool right = x >= 50 && x < 60 && y >= 20 && y < 40;
      const bool top = x >= 20 && x < 60 && y >= 40 && y < 50;
      if (!left && !right && !top) {
        m.values[k] = 254;
        m.types[k] = 3;
      }
      if (left || right) {
        m.terrain_labels[k] = 5;
        m.uphill_y[k] = 1;
      }
    }
  return m;
}
TerrainSampler sampler(const rog_map::MapSnapshot& map) {
  return [&map](double x, double y) { return map.sampleTerrain(x, y); };
}

TEST(DirectedRamp, SamplesLiveAndAccumulatedSlopeAndHandlesInvalidInput) {
  auto m = ramp();
  const auto sample = m.sampleTerrain(2.55, 2.55);
  EXPECT_TRUE(sample.known);
  EXPECT_EQ(sample.label, 5);
  EXPECT_NEAR(sample.uphill.y(), 1, 1e-9);
  EXPECT_NEAR(sample.body, 1, 1e-9);
  EXPECT_FALSE(m.sampleTerrain(-1, 0).known);
  EXPECT_FALSE(m.sampleTerrain(NAN, 0).known);
  OnlineGlobalMapConfig config;
  config.terrain_confirmations = 1;
  config.occupied_confirmations = 1;
  OnlineGlobalMap history(config, "odom");
  ASSERT_EQ(history.update(m), OnlineGlobalMap::UpdateResult::UPDATED);
  const auto pinned = history.snapshot();
  EXPECT_EQ(pinned->sampleTerrain(2.55, 2.55).label, 5);
  EXPECT_NEAR(pinned->sampleTerrain(2.55, 2.55).uphill.y(), 1, 1e-9);
  m.sequence++;
  m.stamp++;
  m.terrain_labels.assign(m.terrain_labels.size(), 0);
  history.update(m);
  EXPECT_EQ(pinned->sampleTerrain(2.55, 2.55).label, 5);  // immutable pin
  m.uphill_x.clear();
  EXPECT_FALSE(m.sampleTerrain(2.55, 2.55).known);
}
TEST(DirectedRamp, AllowsBothSlopeDirectionsAndRejectsSidewaysAndMissingAxes) {
  auto m = ramp();
  auto cfg = DirectedTerrainConfig{true, 0.8, 0.5};
  const auto s = sampler(m);
  EXPECT_TRUE(directed_terrain_edge_allowed(s, {2.55, 2.5}, {2.55, 3.5}, cfg, 0.05));
  EXPECT_TRUE(directed_terrain_edge_allowed(s, {2.55, 3.5}, {2.55, 2.5}, cfg, 0.05));
  EXPECT_FALSE(directed_terrain_edge_allowed(s, {2.5, 2.55}, {2.7, 2.55}, cfg, 0.05));
  EXPECT_TRUE(directed_terrain_edge_allowed(s, {2.5, 2.55}, {2.5, 2.55}, cfg, 0.05));
  m.uphill_y.assign(m.uphill_y.size(), 0);
  EXPECT_FALSE(directed_terrain_edge_allowed(s, {2.55, 2.5}, {2.55, 3.5}, cfg, 0.05));
  cfg.enable = false;
  EXPECT_TRUE(directed_terrain_edge_allowed(s, {2.55, 2.5}, {2.55, 3.5}, cfg, 0.05));
}
TEST(DirectedRamp, ReservesHalfMeterInFrontInsteadOfTurningAtTheLip) {
  auto m = ramp();
  DirectedTerrainConfig cfg{true, 0.8, 0.5};
  const auto front =
      makeDirectedTerrainSampler(m.width, m.height, m.resolution, 0, 0, sampler(m), cfg);
  EXPECT_FALSE(directed_terrain_edge_allowed(front, {2.35, 1.65}, {2.45, 1.65}, cfg, 0.05));
  EXPECT_TRUE(directed_terrain_edge_allowed(front, {2.35, 1.35}, {2.45, 1.35}, cfg, 0.05));
  EXPECT_TRUE(directed_terrain_edge_allowed(front, {2.55, 1.55}, {2.55, 2.25}, cfg, 0.05));
  EXPECT_TRUE(directed_terrain_edge_allowed(front, {2.55, 2.25}, {2.55, 1.55}, cfg, 0.05));
}
TEST(DirectedRamp, SyntheticHairpinCrossesViaFrontRunwaysInBothDirections) {
  for (const double approach : {0.5, 0.85}) {
    auto m = std::make_shared<rog_map::MapSnapshot>(ramp());
    auto query = std::make_shared<rog_map::QueryAdapter>();
    query->update(m, nullptr);
    DirectedTerrainConfig cfg{true, 0.8, approach};
    const auto direction =
        makeDirectedTerrainSampler(m->width, m->height, m->resolution, 0, 0, sampler(*m), cfg);
    smac::SmacPlanner2DSimple search;
    search.setParameters(false, 100000, 0.01);
    search.setMap(query);
    search.setESDFQuery(query);
    search.setMinimumClearance(0.27);
    search.setEdgeFilter([&](unsigned ax, unsigned ay, unsigned bx, unsigned by) {
      return directed_terrain_edge_allowed(direction, {(ax + 0.5) * 0.1, (ay + 0.5) * 0.1},
                                           {(bx + 0.5) * 0.1, (by + 0.5) * 0.1}, cfg, 0.05);
    });
    for (bool reverse : {false, true}) {
      smac::SmacPlanner2DSimple::CoordinateVector route;
      ASSERT_TRUE(search.createPath(reverse ? 55 : 25, 15, reverse ? 25 : 55, 15, route,
                                    [] { return false; }));
      // Foreground direct route also exists. Force the mission onto the platform
      // separately so both ascent and descent connectivity are exercised.
      ASSERT_TRUE(search.createPath(reverse ? 70 : 8, 15, reverse ? 55 : 25, 45, route,
                                    [] { return false; }));
      bool slope = false;
      for (const auto& cell : route) {
        EXPECT_NE(m->values[static_cast<int>(cell.y) * m->width + static_cast<int>(cell.x)], 254);
        slope |=
            m->terrain_labels[static_cast<int>(cell.y) * m->width + static_cast<int>(cell.x)] == 5;
      }
      EXPECT_TRUE(slope);
      for (size_t i = 1; i < route.size(); ++i) {
        const auto& a = route[i - 1];
        const auto& b = route[i];
        const double x = (b.x + 0.5) * 0.1, y = (b.y + 0.5) * 0.1;
        if (y < 2.0 && y > 2.0 - approach && ((x >= 2.0 && x < 3.0) || (x >= 5.0 && x < 6.0)))
          EXPECT_EQ(a.x, b.x) << "Lateral travel entered the front runway";
      }
      // Downhill, including reversed travel along the same physical uphill axis.
      ASSERT_TRUE(search.createPath(reverse ? 25 : 55, 45, reverse ? 8 : 70, 15, route,
                                    [] { return false; }));
    }
  }
}

TEST(DirectedRamp, SmoothedCornerBeforeThePhysicalSlopeIsRejectedByLiveAndHistoryRunways) {
  auto m = ramp();
  const int unknown = 14 * m.width + 28, blocked = 14 * m.width + 29;
  m.values[unknown] = 255;
  m.types[unknown] = 0;
  m.terrain_labels[unknown] = 255;
  m.values[blocked] = 254;
  m.types[blocked] = 3;
  m.terrain_labels[blocked] = 255;
  DirectedTerrainConfig cfg{true, 0.8, 0.85};
  OnlineGlobalMapConfig history_config;
  history_config.terrain_confirmations = 1;
  history_config.occupied_confirmations = 1;
  OnlineGlobalMap history(history_config, "odom");
  history.update(m);
  const auto remembered = history.snapshot();
  const auto live_front = makeDirectedTerrainSampler(
    m.width, m.height, m.resolution, 0, 0, sampler(m), cfg);
  const auto history_front = makeDirectedTerrainSampler(
    remembered->sizeX(), remembered->sizeY(), remembered->resolution(),
    remembered->originX(), remembered->originY(),
    [remembered](double x, double y) { return remembered->sampleTerrain(x, y); }, cfg,
    [remembered](int x, int y) { return remembered->isFree(x, y); });
  // A smoothing shortcut entirely on the flat passes a physical-slope-only
  // check, but enters the reserved runway diagonally. Test both directions.
  const Eigen::Vector2d a(2.35, 1.45), b(2.55, 1.65);
  EXPECT_EQ(remembered->sampleTerrain(a.x(), a.y()).label, 255);
  EXPECT_TRUE(directed_terrain_edge_allowed(sampler(m), a, b, cfg, 0.025));
  for (const auto & front : {live_front, history_front}) {
    EXPECT_EQ(front(2.85, 1.45).label, 255);  // genuinely unknown
    EXPECT_EQ(front(2.95, 1.45).label, 255);  // blocked, not ordinary/free
    EXPECT_FALSE(directed_terrain_edge_allowed(front, a, b, cfg, 0.025));
    EXPECT_FALSE(directed_terrain_edge_allowed(front, b, a, cfg, 0.025));
    EXPECT_TRUE(directed_terrain_edge_allowed(front, {2.55, 1.25}, {2.55, 2.25}, cfg, 0.025));
    EXPECT_TRUE(directed_terrain_edge_allowed(front, {2.55, 2.25}, {2.55, 1.25}, cfg, 0.025));
    EXPECT_TRUE(directed_terrain_edge_allowed(front, {2.35, 0.95}, {2.55, 0.95}, cfg, 0.025));
  }
}
TEST(DirectedRamp, RunupEnabledStillEscapesDeadEndBackwardAlongRampAxis) {
  auto m = ramp();
  // An observed enclosed corridor, with the sole frontier west of the start.
  m.width = 60;
  m.height = 40;
  const int n = m.width * m.height;
  m.values.assign(n, 0);
  m.types.assign(n, 1);
  m.terrain_labels.assign(n, 0);
  m.uphill_x.assign(n, 0);
  m.uphill_y.assign(n, 0);
  for (int y = 0; y < m.height; ++y)
    for (int x = 0; x < m.width; ++x) {
      const int k = y * m.width + x;
      if (x < 5) {
        m.types[k] = 0;
        m.values[k] = 255;
      }
      if (y < 2 || y >= 38 || x >= 59) {
        m.types[k] = 3;
        m.values[k] = 254;
      }
      if (x >= 30 && x < 55 && y >= 8 && y < 32) {
        m.terrain_labels[k] = 5;
        m.uphill_x[k] = 1;
      }
    }
  OnlineGlobalMapConfig config;
  config.terrain_confirmations = 1;
  config.occupied_confirmations = 1;
  OnlineGlobalMap history(config, "odom");
  history.update(m);
  HistoryFrontierQuery request;
  request.start = {5, 2};
  request.goal = {10, 2};
  request.directed_terrain = {true, 0.8, 0.5};
  const auto route = findHistoryFrontierRoute(*history.snapshot(), request);
  ASSERT_TRUE(route);
  EXPECT_LT(route->target.x(), 1.6);
  EXPECT_GT(route->target_to_start.size(), 30);
}
TEST(DirectedRamp, RunupGateIsContinuousAtBothEnds) {
  EXPECT_EQ(runup_distance_gate(0.2).first, 1);
  EXPECT_NEAR(runup_distance_gate(0.25).first, 0.5, 1e-12);
  EXPECT_NEAR(runup_distance_gate(0.3).first, 0, 1e-12);
  for (double d : {0.2, 0.3}) EXPECT_NEAR(runup_distance_gate(d).second, 0, 1e-12);
}
TEST(DirectedRamp, PathHeadingUsesMapScaleAtStopsAndStillRejectsRealSidewaysTravel) {
  TerrainSampler runway = [](double,double) {
    rog_map::TerrainDirectionSample s; s.known=true;s.label=5;s.body=1;s.uphill={1,0};return s;
  };
  DirectedTerrainConfig cfg{true,0.8,0.5};
  EXPECT_TRUE(directed_terrain_path_allowed(runway,{{0,0},{.1,0},{.2,0},{.2,.0001}},cfg,.025));
  EXPECT_TRUE(directed_terrain_path_allowed(runway,{{0,0},{0,.0001},{.1,0},{.2,0}},cfg,.025));
  EXPECT_TRUE(directed_terrain_path_allowed(runway,{{.2,0},{.1,0},{0,0},{0,.0001}},cfg,.025));
  EXPECT_TRUE(directed_terrain_path_allowed(runway,{{.2,0},{.2,0},{.2,0}},cfg,.025));
  EXPECT_FALSE(directed_terrain_path_allowed(runway,{{0,0},{.1,0},{.1,.1}},cfg,.025));
  EXPECT_FALSE(directed_terrain_path_allowed(runway,{{0,0},{.1,.1},{.2,.2}},cfg,.025));
  TerrainSampler missing = [](double,double) {
    rog_map::TerrainDirectionSample s;s.label=5;return s;
  };
  EXPECT_FALSE(directed_terrain_path_allowed(missing,{{0,0},{.1,0}},cfg,.025));
}
TEST(DirectedRamp, ExposureAdjointMatchesNumericalPositionVelocityAccelerationAndTimeGradients) {
  TerrainSampler terrain = [](double x, double y) {
    rog_map::TerrainDirectionSample s;
    s.known = true;
    s.label = 5;
    s.body = 0.6 + 0.1 * std::sin(x) + 0.1 * std::cos(y);
    s.body_gradient = {0.1 * std::cos(x), -0.1 * std::sin(y)};
    const double theta = 0.3 * x - 0.2 * y;
    s.uphill = {std::cos(theta), std::sin(theta)};
    const Eigen::Vector2d derivative(-std::sin(theta), std::cos(theta));
    s.uphill_jacobian.col(0) = 0.3 * derivative;
    s.uphill_jacobian.col(1) = -0.2 * derivative;
    return s;
  };
  std::vector<RunupSample> samples;
  for (int i = 0; i < 6; ++i)
    samples.push_back({{0.12 * i, 0.04 * i}, {1.1, 0.2 + 0.1 * i}, {0.2, -0.3}, 0.06});
  RunupConfig cfg;
  cfg.enable = true;
  cfg.minimum_alignment_cosine = 0.98;  // Exercises both active and inactive cone samples.
  std::vector<RunupGradient> gradient;
  const double original = runupCost(samples, terrain, cfg, gradient);
  ASSERT_GT(original, 0);
  for (size_t i = 0; i < samples.size(); ++i)
    for (int dim = 0; dim < 7; ++dim) {
      auto plus = samples, minus = samples;
      auto component = [dim](RunupSample& s) -> double& {
        if (dim < 2) return s.position(dim);
        if (dim < 4) return s.velocity(dim - 2);
        if (dim < 6) return s.acceleration(dim - 4);
        return s.time_measure;
      };
      const double expected = dim < 2   ? gradient[i].position(dim)
                              : dim < 4 ? gradient[i].velocity(dim - 2)
                              : dim < 6 ? gradient[i].acceleration(dim - 4)
                                        : gradient[i].time_measure;
      constexpr double epsilon = 1e-6;
      component(plus[i]) += epsilon;
      component(minus[i]) -= epsilon;
      std::vector<RunupGradient> unused;
      const double numeric =
          (runupCost(plus, terrain, cfg, unused) - runupCost(minus, terrain, cfg, unused)) /
          (2 * epsilon);
      EXPECT_NEAR(expected, numeric, 1e-5 * (1 + std::abs(numeric)))
          << "sample=" << i << " dim=" << dim;
    }
}
TEST(DirectedRamp, FutureRampPenalizesApproachTurnButNotOrdinaryGround) {
  RunupConfig cfg;
  cfg.enable = true;
  std::vector<RunupGradient> gradient;
  std::vector<RunupSample> samples;
  for (int i = 0; i < 12; ++i) samples.push_back({{0.05 * i, 0}, {1, 0}, {0, 0}, 0.05});
  TerrainSampler terrain = [](double x, double) {
    rog_map::TerrainDirectionSample s;
    s.uphill = {1, 0};
    s.body = x >= 0.3 ? 1 : 0;
    return s;
  };
  EXPECT_NEAR(runupCost(samples, terrain, cfg, gradient), 0, 1e-12);
  samples[3].acceleration = {0, 1};
  EXPECT_GT(runupCost(samples, terrain, cfg, gradient), 0.1);
  EXPECT_NEAR(
      runupCost(
          samples, [](double, double) { return rog_map::TerrainDirectionSample{}; }, cfg, gradient),
      0, 1e-12);
  cfg.enable = false;
  EXPECT_EQ(runupCost(samples, terrain, cfg, gradient), 0);
}
TEST(DirectedRamp, ReusedExposureWorkspaceClearsAdjointsWhenSamplesOrTerrainChange) {
  RunupConfig cfg; cfg.enable = true;
  RunupCostWorkspace workspace;
  std::vector<RunupGradient> cached_gradient;
  for (int n : {18, 5, 27, 0, 9}) {
    std::vector<RunupSample> samples;
    for (int i = 0; i < n; ++i)
      samples.push_back({{.05*i,.01*i},{1.,.2},{.1,-.2},.05});
    for (bool ramp : {true, false}) {
      TerrainSampler sampler = [ramp](double x,double) {
        rog_map::TerrainDirectionSample s;
        if (ramp) { s.body=.5+.1*std::sin(x);s.body_gradient={.1*std::cos(x),0.};s.uphill={1.,0.}; }
        return s;
      };
      std::vector<RunupGradient> fresh;
      const double expected = runupCost(samples,sampler,cfg,fresh);
      EXPECT_DOUBLE_EQ(runupCost(samples,sampler,cfg,cached_gradient,&workspace),expected);
      ASSERT_EQ(cached_gradient.size(),fresh.size());
      for (size_t i=0;i<fresh.size();++i) {
        EXPECT_TRUE(cached_gradient[i].position.isApprox(fresh[i].position,1e-14));
        EXPECT_TRUE(cached_gradient[i].velocity.isApprox(fresh[i].velocity,1e-14));
        EXPECT_TRUE(cached_gradient[i].acceleration.isApprox(fresh[i].acceleration,1e-14));
        EXPECT_DOUBLE_EQ(cached_gradient[i].time_measure,fresh[i].time_measure);
      }
    }
  }
}
}  // namespace
