#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <rog_map/ground_evidence.hpp>
#include <rog_map/query_adapter.hpp>

namespace {

rog_map::MapSnapshot groundPatch()
{
  rog_map::MapSnapshot source;
  source.width = source.height = 21;
  source.resolution = 0.05;
  const size_t count = 21 * 21;
  source.types.assign(count, 2U);
  source.values.assign(count, 0U);
  source.raw_types = source.types;
  source.ground_heights.assign(count, 0.0F);
  return source;
}

TEST(GroundEvidence, SparseEnclosingSupportDoesNotDependOnQueryOrder)
{
  auto source = groundPatch();
  for (int y = 0; y < source.height; ++y) for (int x = 0; x < source.width; ++x) {
    if (std::hypot(x - 10, y - 10) < 7) {
      const size_t i = static_cast<size_t>(y) * source.width + x;
      source.types[i] = source.raw_types[i] = 1U;
      source.ground_heights[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }
  const rog_map::GroundEvidenceView evidence(source);
  EXPECT_FALSE(evidence.measuredSupport(10, 10));
  EXPECT_FALSE(evidence.continuousSupport(10, 10, false));
  EXPECT_TRUE(evidence.continuousSupport(10, 10, true));
  EXPECT_FALSE(evidence.continuousSupport(10, 10, false));
}

TEST(GroundEvidence, FreeRaysNeedPreciseGroundBeforeTheyCountAsMeasuredSupport)
{
  auto source = groundPatch();
  source.types.assign(21 * 21, 1U);
  source.raw_types = source.types;
  EXPECT_FALSE(rog_map::GroundEvidenceView(source).measuredSupport(10, 10));
  source.ground_precise.assign(21 * 21, 1U);
  EXPECT_TRUE(rog_map::GroundEvidenceView(source).continuousSupport(10, 10, false));
  source.raw_types[10 * 21 + 10] = 3U;
  EXPECT_FALSE(rog_map::GroundEvidenceView(source).measuredSupport(10, 10));
}

TEST(GroundEvidence, OneSidedSupportAndTrueStepsDoNotCorrectHeightEdges)
{
  for (const bool step : {false, true}) {
    auto source = groundPatch();
    for (int y = 0; y < source.height; ++y) for (int x = 0; x < source.width; ++x) {
      if (x < 10) continue;
      const size_t i = static_cast<size_t>(y) * source.width + x;
      if (step) source.ground_heights[i] = 0.10F;
      else {
        source.types[i] = source.raw_types[i] = 1U;
        source.ground_heights[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }
    source.ground_precise.assign(21 * 21, 1U);
    const rog_map::GroundEvidenceView evidence(source);
    EXPECT_FALSE(evidence.continuousSupport(10, 10, true));
    EXPECT_FALSE(evidence.smoothSupport(10, 10));
  }
}

TEST(GroundEvidence, MissingOrMalformedEvidenceCannotCreateGround)
{
  for (int scenario = 0; scenario < 6; ++scenario) {
    auto source = groundPatch();
    switch (scenario) {
      case 0: source.ground_heights.clear(); break;
      case 1: source.ground_precise.assign(1, 1U); break;
      case 2: source.height_discontinuities.assign(1, 0U); break;
      case 3: source.raw_types.pop_back(); break;
      case 4: source.height_deltas.assign(1, 0.0F); break;
      case 5: source.resolution = 0.0; break;
    }
    const rog_map::GroundEvidenceView evidence(source);
    EXPECT_FALSE(evidence.measuredSupport(10, 10));
    EXPECT_FALSE(evidence.continuousSupport(10, 10, true));
    EXPECT_FALSE(evidence.smoothSupport(10, 10));
  }
}

}  // namespace
