#include <map_server/utils.hpp>

#include <gtest/gtest.h>
#include <msgpack.hpp>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

namespace {

std::string writeTerrain(const std::vector<uint8_t>& labels,
    const std::vector<float>& uphill = {}) {
  msgpack::sbuffer buffer;
  msgpack::packer<msgpack::sbuffer> packer(buffer);
  packer.pack_map(uphill.empty() ? 4 : 5);
  packer.pack(std::string("width")); packer.pack(10);
  packer.pack(std::string("height")); packer.pack(10);
  packer.pack(std::string("resolution")); packer.pack(0.1);
  packer.pack(std::string("terrain"));
  packer.pack_array(static_cast<uint32_t>(labels.size()));
  for (const uint8_t label : labels) packer.pack(label);
  if (!uphill.empty()) {
    packer.pack(std::string("uphill")); packer.pack(uphill);
  }
  char path[] = "/tmp/map_server_terrain_XXXXXX";
  const int fd = mkstemp(path);
  if (fd < 0) throw std::runtime_error("mkstemp failed");
  close(fd);
  std::ofstream file(path, std::ios::binary);
  file.write(buffer.data(), static_cast<std::streamsize>(buffer.size()));
  file.close();
  return path;
}

TEST(TerrainDirection, LabelOnlyMapDerivesLongAxis)
{
  std::vector<uint8_t> labels(100, 0U);
  for (int y = 1; y <= 8; ++y) for (int x = 4; x <= 5; ++x) {
    labels[static_cast<size_t>(y) * 10 + x] = 5U;
  }
  const auto path = writeTerrain(labels);
  const auto map = map_server::map_utils::load_navigation_maps(path, {0.0, 0.0, 8.0});
  unlink(path.c_str());
  const auto axis = map.uphill_map.at<cv::Vec2f>(4, 4);
  EXPECT_LT(std::abs(axis[0]), 0.2F);
  EXPECT_GT(std::abs(axis[1]), 0.9F);
  EXPECT_EQ(map.uphill_map.at<cv::Vec2f>(0, 0), cv::Vec2f(0.0F, 0.0F));
}

TEST(TerrainDirection, ExplicitUphillVectorKeepsItsSign)
{
  std::vector<uint8_t> labels(100, 0U);
  std::vector<float> uphill(200, 0.0F);
  labels[44] = 5U;
  uphill[88] = -1.0F;
  const auto path = writeTerrain(labels, uphill);
  const auto map = map_server::map_utils::load_navigation_maps(path, {0.0, 0.0, 8.0});
  unlink(path.c_str());
  EXPECT_FLOAT_EQ(map.uphill_map.at<cv::Vec2f>(4, 4)[0], -1.0F);
}

}  // namespace
