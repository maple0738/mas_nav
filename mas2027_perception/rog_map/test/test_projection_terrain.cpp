#include <rog_map/projection_layer.hpp>
#include <rog_map/body_return_filter.hpp>
#include <rog_map/undulating_profile.hpp>
#include <rog_map/local_surface_grid.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr int kSize = 21;
constexpr double kResolution = 0.05;

TEST(BodyReturnFilter, UsesRotatedSquareAndPreservesNearbyWallReturns)
{
  super_utils::Pose pose{
    super_utils::Vec3f(1.0, 2.0, 0.0),
    super_utils::Quatf(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()))};
  const double half_side = 0.302 / std::sqrt(2.0);
  const Eigen::Vector2d axis(std::sqrt(0.5), std::sqrt(0.5));
  const Eigen::Vector2d center(1.0, 2.0);
  const Eigen::Vector2d self = center + axis * (half_side - 0.01);
  const Eigen::Vector2d wall = center + axis * (half_side + 0.01);
  const rog_map::BodyReturnFilter filter(pose, 0.302);
  EXPECT_TRUE(filter.contains(self.x(), self.y()));
  EXPECT_FALSE(filter.contains(wall.x(), wall.y()));
  EXPECT_FALSE(rog_map::BodyReturnFilter(pose, 0.0).contains(self.x(), self.y()));
}

rog_map::ColumnStats ground(double z)
{
  rog_map::ColumnStats stats;
  stats.observed_count = 10;
  stats.occupied_count = 1;
  stats.occupied_z_index_min = static_cast<int>(std::round(z / kResolution));
  stats.occupied_z_index_max = stats.occupied_z_index_min;
  stats.occupied_z_min_abs = z;
  stats.occupied_z_max_abs = z;
  return stats;
}

std::vector<uint8_t> labelsFor(const rog_map::ProjectionLayer::ColumnScanner & scanner)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.min_observed_voxels = 1;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0,
    config, scanner);
  return layer.terrainLabels();
}

TEST(ProjectionTerrain, FlatGroundIsNotSlope)
{
  const auto labels = labelsFor([](int, int) { return ground(1.0); });
  EXPECT_EQ(labels[10 * kSize + 10], 0U);
}

TEST(LocalSurface, RequiresCoverageFreshnessAndRejectsRingAliases)
{
  rog_map::LocalSurfaceGrid grid;
  grid.begin(0, 0, 1);
  const auto fill = [&](double x, double y) {
    for (int i=0; i<3; ++i) EXPECT_TRUE(grid.add(x,y,.013));
  };
  fill(.0125,.0125);
  EXPECT_FALSE(grid.surface(.025,.025,.05,1).valid);
  fill(.0375,.0125);
  EXPECT_FALSE(grid.surface(.025,.025,.05,1,false).valid);
  EXPECT_FALSE(grid.surface(.025,.025,.05,1).valid);
  fill(.0125,.0375); fill(.0375,.0375);
  EXPECT_TRUE(grid.surface(.025,.025,.05,1,false).valid);
  const auto surface=grid.surface(.025,.025,.05,1);
  ASSERT_TRUE(surface.valid); EXPECT_NEAR(surface.height,.013,1e-8);
  EXPECT_FALSE(grid.surface(.025,.025,.05,2.01).valid);
  grid.begin(5.025,0,1.1); fill(5.0375,.0125);
  grid.begin(0,0,1.2);
  EXPECT_FALSE(grid.surface(.025,.025,.05,1.2).valid);
  grid.begin(0,0,.5);
  EXPECT_FALSE(grid.surface(.025,.025,.05,.5).valid);
}

TEST(LocalSurface, MedianDoesNotHideVerticalOutliersAndFloatCentersStayAligned)
{
  rog_map::LocalSurfaceGrid grid; grid.begin(10,10,1);
  for (int y=0;y<2;++y) for (int x=0;x<2;++x)
    for (int n=0;n<3;++n) grid.add(10+(x+.5)*.025,10+(y+.5)*.025,.013);
  const double center=static_cast<float>(10.025);
  ASSERT_TRUE(grid.surface(center,center,.05,1).valid);
  grid.add(10.0125,10.0125,.5);
  const auto surface=grid.surface(center,center,.05,1);
  ASSERT_TRUE(surface.valid);
  EXPECT_NEAR(surface.height,.013,1e-8);
  EXPECT_GT(surface.high-surface.low,.4);
  grid.begin(10,10,2.1);
  for (int y=0;y<2;++y) for (int x=0;x<2;++x)
    for (int n=0;n<3;++n) grid.add(10+(x+.5)*.025,10+(y+.5)*.025,.013);
  EXPECT_LT(grid.surface(center,center,.05,2.1).high,.02);
  EXPECT_FALSE(grid.add(std::numeric_limits<double>::infinity(),10,0));
  EXPECT_FALSE(grid.add(10,10,std::numeric_limits<double>::quiet_NaN()));
}

TEST(LocalSurface, PartialHeightRequiresAWellSupportedGroundPlane)
{
  for (double slope : {.4,3.}) {
    rog_map::LocalSurfaceGrid grid;grid.begin(0,0,1);
    for(int y=0;y<2;++y) for(int x=0;x<2;++x) for(int n=0;n<3;++n) {
      const double px=(x+.5)*.025,py=(y+.5)*.025;
      grid.add(px,py,slope*px);
    }
    EXPECT_EQ(grid.surface(.025,.025,.05,1,false).valid,slope<1.) << slope;
  }
  rog_map::LocalSurfaceGrid wall;wall.begin(0,0,1);
  for(int y=0;y<2;++y) for(int n=0;n<8;++n) wall.add(.025,(y+.5)*.025,n*.01);
  EXPECT_FALSE(wall.surface(.025,.025,.05,1,false).valid);
}

TEST(LocalSurface, PlaneHeightUsesColumnCenterDespiteAsymmetricSampling)
{
  rog_map::LocalSurfaceGrid grid;grid.begin(0,0,1);
  for(int y=0;y<2;++y) for(double x : {.007,.010,.013,.032,.035,.038})
    grid.add(x,(y+.5)*.025,.04+.8*x);
  const auto surface=grid.surface(.025,.025,.05,1,false);
  ASSERT_TRUE(surface.valid);EXPECT_NEAR(surface.height,.06,1e-7);
}

TEST(LocalSurface, ExpandedSupportMustEncloseTheColumnAndPreserveRealSteps)
{
  for (int kind=0;kind<3;++kind) {
    rog_map::LocalSurfaceGrid grid;grid.begin(0,0,1);
    for(int iy=-1;iy<3;++iy) for(int ix=-1;ix<3;++ix) {
      const double x=(ix+.5)*.025,y=(iy+.5)*.025;
      if(kind==1 && x<.025) continue;
      const double z=kind==2 ? (x<.025?0.:.1) : .04+.035*(1-std::cos(2*M_PI*x/.24));
      for(int n=0;n<3;++n) grid.add(x,y,z);
    }
    const auto surface=grid.surface(.025,.025,.10,1,false,true);
    EXPECT_EQ(surface.valid,kind==0) << kind;
    if(kind==0) EXPECT_LT(surface.high-surface.low,.08);
  }
}

TEST(LocalSurface, EnclosingPatchCanExplainOccupancyMissedBySparseColumnFit)
{
  rog_map::LocalSurfaceGrid grid;grid.begin(0,0,1);
  for(int iy=-1;iy<3;++iy) for(int ix=1;ix<5;++ix) {
    const double x=(ix+.5)*.025,y=(iy+.5)*.025;
    for(int n=0;n<3;++n) grid.add(x,y,.04+.035*(1-std::cos(2*M_PI*x/.24)));
  }
  const auto own=grid.surface(.075,.025,.05,1,false);
  const auto wide=grid.surface(.075,.025,.10,1,false,true);
  ASSERT_TRUE(own.valid);ASSERT_TRUE(wide.valid);
  auto stats=ground(.025);stats.occupied_z_max_abs=.125;
  const auto explains=[&](const rog_map::LocalSurfaceGrid::Surface & model) {
    stats.surface_valid=model.valid;stats.surface_z=model.height;
    stats.surface_low=model.low;stats.surface_high=model.high;
    return rog_map::preciseSurfaceExplainsColumn(stats,.05,.08);
  };
  EXPECT_FALSE(explains(own));EXPECT_TRUE(explains(wide));
  stats.surface_conflict=true;EXPECT_FALSE(explains(wide));stats.surface_conflict=false;
  stats.occupied_z_max_abs=.225;
  EXPECT_FALSE(explains(wide));
}

TEST(ProjectionTerrain, RawSurfaceResolvesQuantizedDiagonalWaves)
{
  for (double phase : {0., .5, 1., 1.5}) {
    rog_map::LocalSurfaceGrid grid; grid.begin(.5,.5,1);
    const auto zAt = [phase](double x,double y) {
      return .035*std::sin(2*M_PI*(x+y)/std::sqrt(2.)/.24+phase);
    };
    for (int y=0; y<2*kSize; ++y) for (int x=0; x<2*kSize; ++x) {
      const double px=(x+.5)*.025, py=(y+.5)*.025;
      for (int n=0;n<3;++n) grid.add(px,py,zAt(px,py));
    }
    const auto labels=labelsFor([&](int x,int y) {
      const auto s=grid.surface((x+.5)*kResolution,(y+.5)*kResolution,kResolution,1);
      EXPECT_TRUE(s.valid);
      auto stats=ground(std::round(s.low/kResolution)*kResolution);
      stats.occupied_z_max_abs=std::round(s.high/kResolution)*kResolution;
      stats.occupied_z_index_max=static_cast<int>(std::round(stats.occupied_z_max_abs/kResolution));
      stats.occupied_count=stats.occupied_z_index_max-stats.occupied_z_index_min+1;
      stats.surface_valid=s.valid; stats.surface_z=s.height;
      stats.surface_low=s.low; stats.surface_high=s.high;
      return stats;
    });
    EXPECT_EQ(labels[10*kSize+10],7U) << phase;
  }
}

TEST(ProjectionTerrain, RawFloorDoesNotEraseOverhangWallOrRealStep)
{
  for (int kind=0; kind<3; ++kind) {
    rog_map::ProjectionLayer layer; rog_map::ProjectionLayerConfig config;
    config.hysteresis_en=false;config.mask_filter_en=false;
    layer.updateFull(kSize,kSize,kResolution,Eigen::Vector2d::Zero(),1,config,
      [kind](int x,int) {
        const double z=kind==2 && x>=10 ? .2 : 0.;
        auto s=ground(z); s.surface_valid=true; s.surface_z=z;
        s.surface_low=z; s.surface_high=z;
        if (kind<2) {s.occupied_z_max_abs=.5; s.occupied_z_index_max=10; s.occupied_count=kind==0?11:2;}
        return s;
      });
    EXPECT_EQ(layer.values()[10*kSize+10],254U) << kind;
  }
}

TEST(ProjectionGround, OnlyCoveredRawSpanCanCorrectTenCentimeterVoxelSpan)
{
  for (bool valid : {false,true}) {
    rog_map::ProjectionLayer layer; rog_map::ProjectionLayerConfig config;
    config.surface_height_delta_max=.08;
    config.hysteresis_en=false; config.mask_filter_en=false;
    layer.updateFull(kSize,kSize,kResolution,Eigen::Vector2d::Zero(),1,config,
      [valid](int,int) {
        auto s=ground(-.05);
        s.occupied_z_index_max=1;s.occupied_z_max_abs=.05;s.occupied_count=3;
        s.surface_valid=valid;s.surface_z=0;s.surface_low=-.03;s.surface_high=.03;
        return s;
      });
    EXPECT_EQ(layer.values()[10*kSize+10],valid?0U:254U);
  }
}

TEST(ProjectionGround, SparseRevisitKeepsVerifiedPrecisionButNewObstacleInvalidatesIt)
{
  rog_map::ProjectionLayer layer; rog_map::ProjectionLayerConfig config;
  config.surface_height_delta_max=.08; config.hysteresis_en=false; config.mask_filter_en=false;
  const auto scan=[](bool precise,bool obstacle) {
    auto s=ground(-.05);s.occupied_count=obstacle?6:3;
    s.occupied_z_index_max=obstacle?4:1;s.occupied_z_max_abs=obstacle?.20:.05;
    s.surface_valid=precise;s.surface_z=.013;s.surface_low=-.03;s.surface_high=.03;
    return s;
  };
  layer.updateFull(kSize,kSize,kResolution,Eigen::Vector2d::Zero(),1,config,
    [&](int,int){return scan(true,false);});
  EXPECT_NEAR(layer.cells()[10*kSize+10].ground_reference_z,.013,1e-8);
  layer.updateFull(kSize,kSize,kResolution,Eigen::Vector2d::Zero(),2,config,
    [&](int,int){return scan(false,false);});
  EXPECT_EQ(layer.values()[10*kSize+10],0U);
  EXPECT_NEAR(layer.cells()[10*kSize+10].ground_reference_z,.013,1e-8);
  layer.updateFull(kSize,kSize,kResolution,Eigen::Vector2d::Zero(),2.5,config,
    [&](int,int){auto s=scan(false,false);s.occupied_z_max_abs=0;
      s.occupied_z_index_max=0;s.occupied_count=2;return s;});
  EXPECT_NEAR(layer.cells()[10*kSize+10].ground_reference_z,.013,1e-8);
  EXPECT_TRUE(std::isfinite(layer.cells()[10*kSize+10].precise_ground_z));
  layer.updateFull(kSize,kSize,kResolution,Eigen::Vector2d::Zero(),3,config,
    [&](int,int){return scan(false,true);});
  EXPECT_EQ(layer.values()[10*kSize+10],254U);
}

TEST(ProjectionGround, DirectRawReturnsRefreshRetainedGroundAfterRayClearing)
{
  for(bool supported : {false,true}) {
    rog_map::ProjectionLayer layer; rog_map::ProjectionLayerConfig config;
    config.surface_height_delta_max=.08;config.hysteresis_en=false;config.mask_filter_en=false;
    layer.updateFull(kSize,kSize,kResolution,Eigen::Vector2d::Zero(),1,config,
      [](int x,int){return ground(x<10?.025:.125);});
    EXPECT_TRUE(layer.cells()[10*kSize+9].height_discontinuity);
    layer.updateFull(kSize,kSize,kResolution,Eigen::Vector2d::Zero(),2,config,
      [supported](int x,int){
        rog_map::ColumnStats s;s.observed_count=3;
        s.surface_valid=supported;s.surface_z=x<10?.095:.108;
        s.surface_low=s.surface_z-.01;s.surface_high=s.surface_z+.01;return s;
      });
    EXPECT_EQ(layer.cells()[10*kSize+9].height_discontinuity,!supported);
    EXPECT_EQ(std::isfinite(layer.cells()[10*kSize+9].precise_ground_z),supported);
  }
  rog_map::ProjectionLayer layer;rog_map::ProjectionLayerConfig config;
  config.hysteresis_en=false;config.mask_filter_en=false;
  layer.updateFull(kSize,kSize,kResolution,Eigen::Vector2d::Zero(),1,config,
    [](int,int){rog_map::ColumnStats s;s.observed_count=3;s.surface_valid=true;
      s.surface_z=.1;s.surface_low=.09;s.surface_high=.11;return s;});
  EXPECT_FALSE(std::isfinite(layer.cells()[10*kSize+10].ground_reference_z));
}

TEST(ProjectionTerrain, Repeated240mm70mmWavesBecomeUndulating)
{
  for (double phase : {0.0, 0.5, 1.0, 1.5}) {
    const auto labels = labelsFor([phase](int x, int) {
      const double z = .035 * std::sin(2 * M_PI * x * kResolution / .24 + phase);
      // Match the actual map's five-centimeter height quantization.
      return ground(std::round(z / kResolution) * kResolution);
    });
    EXPECT_EQ(labels[10 * kSize + 10], 7U) << phase;
  }
}

TEST(ProjectionTerrain, SingleBumpStepAndMissingSupportAreNotUndulating)
{
  for (int kind = 0; kind < 3; ++kind) {
    const auto labels = labelsFor([kind](int x, int y) {
      if (kind == 0) return ground(x == 10 ? .07 : 0.0);
      if (kind == 1) return ground(x >= 10 ? .07 : 0.0);
      if ((x >= 5 && x <= 8) || y == 9) return rog_map::ColumnStats{};
      return ground(.035 * std::sin(2 * M_PI * x * kResolution / .24));
    });
    EXPECT_NE(labels[10 * kSize + 10], 7U) << kind;
  }
}

TEST(ProjectionTerrain, RotatedWavesAreUndulatingButSmallNoiseIsNot)
{
  for (double angle : {M_PI / 2, M_PI / 4}) {
    const auto labels = labelsFor([angle](int x, int y) {
      const double distance = kResolution * (x * std::cos(angle) + y * std::sin(angle));
      const double z = .035 * std::sin(2 * M_PI * distance / .24);
      return ground(z);
    });
    EXPECT_EQ(labels[10 * kSize + 10], 7U) << angle;
  }
  const auto labels = labelsFor([](int x, int y) {
    return ground(.01 * std::sin(x * 1.7 + y * 3.1));
  });
  EXPECT_NE(labels[10 * kSize + 10], 7U);
  // A coarse diagonal observation can quantize to 10 cm jumps. Preserve the
  // existing collision barrier rather than relabeling it as safe terrain.
  const auto quantized = labelsFor([](int x, int y) {
    const double z = .035 * std::sin(2 * M_PI * kResolution * (x + y) / std::sqrt(2.) / .24);
    return ground(std::round(z / kResolution) * kResolution);
  });
  EXPECT_NE(quantized[10 * kSize + 10], 7U);
}

TEST(ProjectionTerrain, ContinuousInclineIsSlope)
{
  const auto labels = labelsFor([](int x, int) {
    return ground(1.0 + 0.2 * x * kResolution);
  });
  EXPECT_EQ(labels[10 * kSize + 10], 5U);
}

TEST(ProjectionTerrain, NoisyContinuousInclineStillSlope)
{
  const auto labels = labelsFor([](int x, int y) {
    const double noise = ((x * 13 + y * 7) % 11 - 5) * 0.004;
    return ground(1.0 + 0.2 * x * kResolution + noise);
  });
  EXPECT_EQ(labels[10 * kSize + 10], 5U);
}

TEST(ProjectionTerrain, UphillVectorFollowsHeightGradient)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.min_observed_voxels = 1;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0,
    config, [](int x, int y) { return ground(1.0 + (0.24 * x - 0.18 * y) * kResolution); });
  const auto geometry = layer.terrainGeometry();
  const size_t center = 10 * kSize + 10;
  ASSERT_EQ(geometry.labels[center], 5U);
  EXPECT_NEAR(geometry.uphill_x[center], 0.8, 0.05);
  EXPECT_NEAR(geometry.uphill_y[center], -0.6, 0.05);
}

TEST(ProjectionTerrain, RaisedThinSurfaceDoesNotTiltGroundPlane)
{
  const auto labels = labelsFor([](int x, int y) {
    if (y < 9 || y > 11 || x < 5 || x > 15) {
      return rog_map::ColumnStats{};
    }
    // One 20 cm raised thin surface gives a single high-leverage sample.
    // A least-squares plane can still fit the remaining flat ground with low
    // RMS, but the max residual exposes the step.
    return ground(x == 15 && y == 9 ? 1.2 : 1.0);
  });
  EXPECT_NE(labels[10 * kSize + 10], 5U);
}

TEST(ProjectionTerrain, ObservedGapWithNeighborSupportIsTunnel)
{
  const auto labels = labelsFor([](int, int) {
    auto stats = ground(0.0);
    stats.observed_count = 21;
    stats.occupied_count = 2;
    stats.occupied_z_index_min = 0;
    stats.occupied_z_index_max = 7;
    stats.occupied_z_max_abs = 0.35;
    stats.free_between_occupied_count = 6;
    return stats;
  });
  EXPECT_EQ(labels[10 * kSize + 10], 6U);
}

TEST(ProjectionTerrain, ConfiguredTunnelSpanIsUsed)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.min_observed_voxels = 1;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  config.tunnel_height_delta_max = 0.75;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int, int) {
      auto stats = ground(0.0);
      stats.observed_count = 21;
      stats.occupied_count = 2;
      stats.occupied_z_index_min = 0;
      stats.occupied_z_index_max = 13;
      stats.occupied_z_max_abs = 0.65;
      stats.free_between_occupied_count = 12;
      return stats;
    });
  const size_t center = 10 * kSize + 10;
  EXPECT_EQ(layer.cells()[center].type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(layer.terrainLabels()[center], 6U);
}

TEST(ProjectionTerrain, OccupiedGapIsNotTunnel)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.min_observed_voxels = 1;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int, int) {
      auto stats = ground(0.0);
      stats.observed_count = 21;
      stats.occupied_count = 2;
      stats.occupied_z_index_min = 0;
      stats.occupied_z_index_max = 20;
      stats.occupied_z_max_abs = 1.0;
      stats.free_between_occupied_count = 19;
      return stats;
    });
  const size_t center = 10 * kSize + 10;
  EXPECT_EQ(layer.cells()[center].type, rog_map::CellType::OCCUPIED);
  EXPECT_NE(layer.terrainLabels()[center], 6U);
}

TEST(ProjectionTerrain, UnknownGapIsNotTunnel)
{
  const auto labels = labelsFor([](int, int) {
    auto stats = ground(0.0);
    stats.observed_count = 2;
    stats.occupied_count = 2;
    stats.occupied_z_index_min = 0;
    stats.occupied_z_index_max = 7;
    stats.occupied_z_max_abs = 0.35;
    return stats;
  });
  EXPECT_NE(labels[10 * kSize + 10], 6U);
}

rog_map::ProjectionLayerConfig groundConfig()
{
  rog_map::ProjectionLayerConfig config;
  config.min_observed_voxels = 1;
  config.surface_height_delta_max = 0.08;
  config.wall_height_delta_min = 0.8;
  config.ground_filter_en = true;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  config.obstacle_hold_time = 1.5;
  return config;
}

rog_map::ColumnStats span(double low, double high)
{
  auto stats = ground(low);
  stats.occupied_z_index_max = static_cast<int>(std::round(high / kResolution));
  stats.occupied_z_max_abs = high;
  stats.occupied_count = stats.occupied_z_index_max - stats.occupied_z_index_min + 1;
  stats.observed_count = std::max(10, stats.occupied_count);
  return stats;
}

TEST(ProjectionGround, NoisyGroundSpanUsesSurroundingPlane)
{
  rog_map::ProjectionLayer layer;
  const auto config = groundConfig();
  const auto scanner = [](int x, int y) {
    return x == 10 && y == 10 ? span(-0.05, 0.05) : ground(0.0);
  };
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config, scanner);
  const auto & cell = layer.cells()[10 * kSize + 10];
  EXPECT_EQ(cell.raw_type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(cell.type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(cell.mask, 1U);
}

TEST(ProjectionGround, RaisedTenCentimeterObstacleAndTallWallStayBlocked)
{
  rog_map::ProjectionLayer layer;
  const auto scanner = [](int x, int y) {
    if (x == 10 && y == 10) return span(0.0, 0.1);
    if (x == 15 && y == 15) return span(0.0, 1.0);
    return ground(0.0);
  };
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0,
    groundConfig(), scanner);
  EXPECT_EQ(layer.cells()[10 * kSize + 10].type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(layer.cells()[15 * kSize + 15].type, rog_map::CellType::OCCUPIED);
}

TEST(ProjectionGround, UnknownAndNonPlanarNeighborsCannotProveGround)
{
  for (bool unknown : {true, false}) {
    rog_map::ProjectionLayer layer;
    const auto scanner = [unknown](int x, int y) {
      if (x == 10 && y == 10) return span(-0.05, 0.05);
      return unknown ? rog_map::ColumnStats{} : ground((x + y) % 2 ? -0.1 : 0.1);
    };
    layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0,
      groundConfig(), scanner);
    EXPECT_EQ(layer.cells()[10 * kSize + 10].type, rog_map::CellType::OCCUPIED);
  }
}

TEST(ProjectionGround, ContinuousSlopeCanSupportNoisySurface)
{
  rog_map::ProjectionLayer layer;
  const auto scanner = [](int x, int y) {
    const double z = 0.2 * (x - 10) * kResolution;
    return x == 10 && y == 10 ? span(-0.05, 0.05) : ground(z);
  };
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0,
    groundConfig(), scanner);
  EXPECT_EQ(layer.cells()[10 * kSize + 10].type, rog_map::CellType::PASSABLE);
}

TEST(ProjectionGround, DirtyNeighborChangesInvalidateOldCorrection)
{
  rog_map::ProjectionLayer layer;
  const auto config = groundConfig();
  const auto initial = [](int x, int y) {
    return x == 10 && y == 10 ? span(-0.05, 0.05) : ground(0.0);
  };
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config, initial);
  ASSERT_EQ(layer.cells()[10 * kSize + 10].type, rog_map::CellType::PASSABLE);
  std::vector<int> dirty;
  for (int y = 7; y <= 13; ++y) for (int x = 7; x <= 13; ++x) {
    if (x != 10 || y != 10) dirty.push_back(y * kSize + x);
  }
  const auto changed = [](int x, int y) {
    return x == 10 && y == 10 ? span(-0.05, 0.05) : ground(0.2);
  };
  layer.updateDirty(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 2.0,
    config, changed, dirty, false);
  EXPECT_EQ(layer.cells()[10 * kSize + 10].raw_type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(layer.cells()[10 * kSize + 10].mask, 0U);
}

TEST(ProjectionGround, DisablingFilterPreservesOriginalClassification)
{
  rog_map::ProjectionLayer layer;
  auto config = groundConfig(); config.ground_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int x, int y) { return x == 10 && y == 10 ? span(-0.05, 0.05) : ground(0.0); });
  EXPECT_EQ(layer.cells()[10 * kSize + 10].type, rog_map::CellType::OCCUPIED);
}

TEST(ProjectionSteps, ThinStairTopsBlockBothSidesWithoutRiserReturns)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int x, int) { return ground(x < 10 ? 0.0 : 0.20); });
  for (int y = 0; y < kSize; ++y) {
    for (const int x : {9, 10}) {
      const auto index = y * kSize + x;
      EXPECT_TRUE(layer.cells()[index].height_discontinuity);
      EXPECT_TRUE(layer.cells()[index].height_discontinuity_measured);
      EXPECT_EQ(layer.values()[index], 254U);
      EXPECT_EQ(layer.mask()[index], 0U);
      EXPECT_EQ(layer.cells()[index].traversable, 0U);
    }
  }
  EXPECT_EQ(layer.values()[10 * kSize + 5], 0U);
  EXPECT_EQ(layer.values()[10 * kSize + 15], 0U);
}

TEST(ProjectionSteps, ContinuousQuantizedSlopeRemainsTraversable)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int x, int) {
      return ground(std::round(0.7 * x * kResolution / kResolution) * kResolution);
    });
  for (const auto value : layer.values()) EXPECT_NE(value, 254U);
}

TEST(ProjectionSteps, DirtyHeightUpdateClearsNeighboringStepMask)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int x, int y) { return ground(x == 10 && y == 10 ? 0.20 : 0.0); });
  ASSERT_EQ(layer.values()[10 * kSize + 9], 254U);
  layer.updateDirty(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 2.0,
    config, [](int, int) { return ground(0.0); }, {10 * kSize + 10}, false);
  for (const auto value : layer.values()) EXPECT_EQ(value, 0U);
}

TEST(ProjectionSteps, TenCentimeterStaircaseIsNotAContinuousRamp)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int x, int) { return ground((x / 4) * 0.10); });
  for (const int boundary : {4, 8, 12, 16}) {
    EXPECT_EQ(layer.values()[10 * kSize + boundary - 1], 254U);
    EXPECT_EQ(layer.values()[10 * kSize + boundary], 254U);
  }
}

TEST(ProjectionSteps, FreeRaysAboveKnownStepDoNotEraseItsHeightEvidence)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.hysteresis_en = false; config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int x, int) { return ground(x < 10 ? 0.0 : 0.30); });
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 2.0, config,
    [](int, int) { rog_map::ColumnStats s; s.observed_count = 20; return s; });
  EXPECT_EQ(layer.values()[10 * kSize + 9], 254U);
  EXPECT_EQ(layer.values()[10 * kSize + 10], 254U);
  // A newly observed continuous floor is sufficient to clear the old edge.
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 3.0, config,
    [](int, int) { return ground(0.0); });
  for (const auto value : layer.values()) EXPECT_EQ(value, 0U);
}

TEST(ProjectionSteps, SparseGroundOnBothSidesBlocksInterveningAirColumns)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.hysteresis_en = false; config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int x, int) {
      if (x >= 8 && x <= 12) { rog_map::ColumnStats s; s.observed_count = 20; return s; }
      return ground(x < 8 ? 0.0 : 0.30);
    });
  for (int x = 7; x <= 13; ++x) {
    EXPECT_EQ(layer.values()[10 * kSize + x], 254U);
    // The edge lies somewhere in the bridged gap: block it live, but mark it
    // as inferred so history does not treat the gap as a measured step.
    EXPECT_FALSE(layer.cells()[10 * kSize + x].height_discontinuity_measured) << x;
  }
}

TEST(ProjectionSteps, SparseContinuousGradeDoesNotBecomeAFalseStep)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.hysteresis_en = false; config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int x, int) {
      if (x >= 8 && x <= 12) { rog_map::ColumnStats s; s.observed_count = 20; return s; }
      return ground(0.5 * x * kResolution);
    });
  for (const auto value : layer.values()) EXPECT_NE(value, 254U);
}

TEST(ProjectionSteps, DirtyUpdateClearsAnEdgeAcrossTheWholeScanGap)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.hysteresis_en = false; config.mask_filter_en = false;
  const auto scan = [](int x, int) {
    if (x >= 8 && x <= 12) { rog_map::ColumnStats s; s.observed_count = 20; return s; }
    return ground(x < 8 ? 0.0 : 0.30);
  };
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config, scan);
  ASSERT_EQ(layer.values()[10 * kSize + 7], 254U);
  std::vector<int> dirty;
  for (int y = 0; y < kSize; ++y)
    for (int x = 13; x < kSize; ++x) dirty.push_back(y*kSize+x);
  layer.updateDirty(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 2.0, config,
    [](int, int) { return ground(0.0); }, dirty, false);
  for (const auto value : layer.values()) EXPECT_EQ(value, 0U);
}

TEST(ProjectionTerrain, ObservedSlopeSurvivesFreeAirRaysButNotSlidingOutOfWindow)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.hysteresis_en = false; config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int x, int) { return ground(0.2*x*kResolution); });
  const auto before = layer.terrainLabels();
  ASSERT_EQ(before[10*kSize+10], 5U);
  const auto free_scan = [](int, int) { rog_map::ColumnStats s; s.observed_count = 20; return s; };
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 2.0, config, free_scan);
  EXPECT_EQ(layer.terrainLabels(), before);
  layer.syncSlidingWindow(kSize, kSize, kResolution, Eigen::Vector2i(200, 0),
    Eigen::Vector2d(10.0, 0.0), config);
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d(10.0, 0.0), 3.0, config, free_scan);
  EXPECT_EQ(layer.terrainLabels()[10*kSize+10], 255U);
}

}  // namespace

TEST(UndulatingProfile, SmallInteriorGapsDoNotInventGroundOrBridgeLongMissingRuns)
{
  std::array<double,17> heights;
  for (int i=0;i<17;++i) heights[i]=.035*std::sin(2*M_PI*i*.05/.24);
  ASSERT_TRUE(rog_map::isUndulatingProfile(heights,.05));
  heights[2]=heights[7]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(rog_map::isUndulatingProfile(heights,.05));
  for (int i=5;i<=8;++i) heights[i]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(rog_map::isUndulatingProfile(heights,.05));
}

TEST(UndulatingProfile, EntryWindowRecognizesARealCycleButNotOneBumpOrGrade)
{
  std::array<double,13> wave,bump,ramp;
  for(int i=0;i<13;++i) {
    const double x=(i-4)*.05;
    wave[i]=x<0?0:.035*(1-std::cos(2*M_PI*x/.24));
    bump[i]=i==6?.07:0;
    ramp[i]=i*.01;
  }
  EXPECT_TRUE(rog_map::isUndulatingProfile(wave,.05));
  EXPECT_FALSE(rog_map::isUndulatingProfile(bump,.05));
  EXPECT_FALSE(rog_map::isUndulatingProfile(ramp,.05));
}
