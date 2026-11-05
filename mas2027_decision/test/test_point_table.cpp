#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <map>
#include <string>

#include "mas2027_decision/point_table.hpp"

namespace mas2027_decision {
namespace {

// 每个用例写一个临时文件到当前工作目录（ctest 下是 build 目录）。
constexpr const char* kTempFile = "test_named_points.yaml";

void writeFile(const std::string& content) {
  std::ofstream file(kTempFile);
  file << content;
}

// 任意命名的点表应全部读入，缺 yaw 时补 0。
TEST(PointTable, LoadsNamedPoints) {
  writeFile(
      "home: [0.0, 0.0]\n"
      "a1: [1.5, -2.0, 0.7]\n"
      "a2: [3.0, 4.0]\n");

  std::map<std::string, Waypoint> points;
  std::string error;
  ASSERT_TRUE(loadNamedPoints(kTempFile, &points, &error)) << error;
  ASSERT_EQ(points.size(), 3u);

  EXPECT_DOUBLE_EQ(points.at("home").x, 0.0);
  EXPECT_DOUBLE_EQ(points.at("home").y, 0.0);
  EXPECT_DOUBLE_EQ(points.at("home").yaw, 0.0);

  EXPECT_DOUBLE_EQ(points.at("a1").x, 1.5);
  EXPECT_DOUBLE_EQ(points.at("a1").y, -2.0);
  EXPECT_DOUBLE_EQ(points.at("a1").yaw, 0.7);

  EXPECT_DOUBLE_EQ(points.at("a2").x, 3.0);
  EXPECT_DOUBLE_EQ(points.at("a2").y, 4.0);
  EXPECT_DOUBLE_EQ(points.at("a2").yaw, 0.0);

  std::remove(kTempFile);
}

// 任一值格式不对时整体失败，输出参数不被修改。
TEST(PointTable, RejectsInvalidEntry) {
  writeFile(
      "home: [0.0, 0.0]\n"
      "a1: [1.0]\n");

  std::map<std::string, Waypoint> points;
  points["keep"] = {9.0, 9.0, 0.0};
  std::string error;
  EXPECT_FALSE(loadNamedPoints(kTempFile, &points, &error));
  EXPECT_FALSE(error.empty());
  ASSERT_EQ(points.size(), 1u);
  EXPECT_DOUBLE_EQ(points.at("keep").x, 9.0);

  std::remove(kTempFile);
}

}  // namespace
}  // namespace mas2027_decision
