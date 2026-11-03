#include "small_point_lio/plane_cache.h"
#include "small_point_lio/output_prediction.h"
#include <gtest/gtest.h>
#include <limits>
using namespace small_point_lio;
TEST(PlaneCache, ReusesOnlyExactOrderedSupport) {
    PlaneCache cache;
    std::vector<Eigen::Vector3f> points{{0,0,1},{1,0,1},{0,1,1},{1,1,1},{.5f,.5f,1}};
    auto first=cache.fit(points);auto second=cache.fit(points);
    ASSERT_TRUE(first.valid);EXPECT_EQ(cache.hits,1U);EXPECT_EQ(cache.misses,1U);
    EXPECT_EQ(first.offset,second.offset);EXPECT_TRUE(first.normal==second.normal);
    points[0].z()=2;auto changed=cache.fit(points);
    EXPECT_EQ(cache.misses,2U);EXPECT_GT(changed.maximum_residual,0);
    cache.clear();EXPECT_EQ(cache.hits,0U);cache.fit(points);EXPECT_EQ(cache.misses,1U);
}
TEST(Prediction, DelayedPosteriorReplaysImuWithoutChangingEstimator) {
    OutputPrediction p;
    p.add({1.,{0,0,0},{0,0,1}},1,false,10,10);
    p.add({1.05,{0,0,0},{0,0,1}},1,false,10,10);
    state s;s.gravity.setZero();s.velocity.x()=2;
    ASSERT_TRUE(p.correct(s,1.,1,false,10,10));
    EXPECT_NEAR(p.value().position.x(),.1,1e-12);
    EXPECT_NEAR(s.position.x(),0,1e-12);
    EXPECT_NEAR(p.value().rotation(1,0),std::sin(.05),1e-12);
    s.position.x()=1;
    p.correct(s,1.025,1,false,10,10);
    EXPECT_NEAR(p.value().position.x(),1.05,1e-12);
    EXPECT_TRUE(p.add({1.06,{0,0,0},{0,0,1}},1,false,10,10));
    EXPECT_DOUBLE_EQ(p.time(),1.06);
}
TEST(Prediction, StopsOnStalePosteriorGapAndReset) {
    OutputPrediction p;state s;
    p.add({1.,{0,0,0},{0,0,0}},1,false,10,10);p.correct(s,1.,1,false,10,10);
    EXPECT_TRUE(p.add({1.09,{0,0,0},{0,0,0}},1,false,10,10));
    EXPECT_FALSE(p.add({1.18,{0,0,0},{0,0,0}},1,false,10,10));
    EXPECT_FALSE(p.correct(s,1.,1,false,10,10));
    p.reset();EXPECT_FALSE(p.add({2.,{0,0,0},{0,0,0}},1,false,10,10));
}
TEST(Prediction, GravityBiasAndSaturationUseImuUnits) {
    OutputPrediction p; state s;
    s.gravity.z()=-9.81; s.ba.z()=.2; s.bg.z()=.1;
    p.add({1.,{0,0,1.02},{0,0,.1}},10,false,3,3);
    p.add({1.05,{0,0,1.02},{0,0,.1}},10,false,3,3);
    ASSERT_TRUE(p.correct(s,1.,10,false,3,3));
    EXPECT_NEAR(p.value().velocity.z(),.19*.05,1e-12);
    EXPECT_NEAR(p.value().rotation(1,0),0,1e-12);
    p.reset(); s.acceleration.z()=9.81;
    p.add({1.,{0,0,4},{0,0,4}},10,true,3,3);
    p.add({1.05,{0,0,4},{0,0,4}},10,true,3,3);
    ASSERT_TRUE(p.correct(s,1.,10,true,3,3));
    EXPECT_NEAR(p.value().velocity.norm(),0,1e-12);
    EXPECT_NEAR(p.value().rotation(1,0),0,1e-12);
}
TEST(Prediction, RejectsInvalidPosteriorAndOutOfOrderImu) {
    OutputPrediction p; state s;
    p.add({1.,{0,0,0},{0,0,0}},1,false,10,10);
    ASSERT_TRUE(p.correct(s,1.,1,false,10,10));
    EXPECT_FALSE(p.add({.99,{0,0,0},{0,0,0}},1,false,10,10));
    EXPECT_TRUE(p.add({1.01,{0,0,0},{0,0,0}},1,false,10,10));
    s.position.x()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(p.correct(s,1.01,1,false,10,10));
    EXPECT_FALSE(p.add({1.02,{0,0,0},{0,0,0}},1,false,10,10));
    p.reset(); p.add({2.,{0,0,0},{0,0,0}},1,false,10,10);
    s=state{}; ASSERT_TRUE(p.correct(s,2.,1,false,10,10));
    EXPECT_FALSE(p.add({2.2,{0,0,0},{0,0,0}},1,false,10,10));
}

TEST(Prediction, RollingHistoryAndBetweenSampleCorrectionsPreserveIntegration) {
    OutputPrediction p; state s;
    p.add({1.,{2,0,0},{0,0,0}},1,false,10,10);
    ASSERT_TRUE(p.correct(s,1.,1,false,10,10));
    for (int i=1; i<=1000; ++i) {
        const double elapsed=i*.005;
        ASSERT_TRUE(p.add({1.+elapsed,{2,0,0},{0,0,0}},1,false,10,10));
        EXPECT_NEAR(p.value().position.x(),elapsed*elapsed,1e-10);
        EXPECT_NEAR(p.value().velocity.x(),2*elapsed,1e-10);
        if (i%20==0) {
            const double lagged=elapsed-.0025;
            s.position.x()=lagged*lagged; s.velocity.x()=2*lagged;
            ASSERT_TRUE(p.correct(s,1.+lagged,1,false,10,10));
            EXPECT_NEAR(p.value().position.x(),elapsed*elapsed,1e-10);
            EXPECT_NEAR(p.value().velocity.x(),2*elapsed,1e-10);
        }
    }
}
