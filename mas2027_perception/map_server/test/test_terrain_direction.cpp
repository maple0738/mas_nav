#include <map_server/utils.hpp>
#include <map_server/map_server_node.hpp>

#include <gtest/gtest.h>
#include <msgpack.hpp>

#include <cmath>
#include <chrono>
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

class StaticMapPublication : public ::testing::Test
{
protected:
  void SetUp() override
  {
    terrain_path_ = writeTerrain(std::vector<uint8_t>(100, 0U));
    yaml_path_ = terrain_path_ + ".yaml";
    image_path_ = terrain_path_ + ".pgm";
    std::ofstream image(image_path_);
    image << "P2\n10 10\n255\n";
    for (int i = 0; i < 100; ++i) image << "255 ";
    image << '\n';
    std::ofstream yaml(yaml_path_);
    yaml << "image: " << image_path_ << "\nresolution: 0.1\norigin: [1.0, 2.0, 0.0]\n";
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
  }

  void TearDown() override
  {
    if (context_) context_->shutdown("static map test complete");
    unlink(terrain_path_.c_str());
    unlink(yaml_path_.c_str());
    unlink(image_path_.c_str());
  }

  std::string terrain_path_, yaml_path_, image_path_;
  std::shared_ptr<rclcpp::Context> context_;
};

TEST_F(StaticMapPublication, LateSubscribersReceiveEachMapWithoutPeriodicRepublish)
{
  const std::string test_namespace = "/static_map_test_" + std::to_string(getpid());
  rclcpp::NodeOptions options;
  options.context(context_);
  options.arguments({"--ros-args", "-r", "__ns:=" + test_namespace});
  options.parameter_overrides({
    rclcpp::Parameter("terrain_map_path", terrain_path_),
    rclcpp::Parameter("map_yaml_path", yaml_path_)});
  // The server publishes in its constructor, before any subscriber exists.
  auto server = std::make_shared<map_server::MapServerNode>(options);
  rclcpp::NodeOptions reader_options;
  reader_options.context(context_);
  auto reader = std::make_shared<rclcpp::Node>("static_map_reader", reader_options);
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  int costs = 0, labels = 0, directions = 0;
  auto cost_sub = reader->create_subscription<nav_msgs::msg::OccupancyGrid>(
    test_namespace + "/cost_map", qos,
    [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
      ++costs;
      EXPECT_EQ(map->info.width, 10U);
      EXPECT_DOUBLE_EQ(map->info.origin.position.x, 1.0);
      EXPECT_DOUBLE_EQ(map->info.origin.position.y, 2.0);
    });
  auto label_sub = reader->create_subscription<sensor_msgs::msg::Image>(
    test_namespace + "/terrain_label_map", qos,
    [&](sensor_msgs::msg::Image::ConstSharedPtr map) {
      ++labels;
      EXPECT_EQ(map->encoding, "mono8");
      EXPECT_EQ(map->data.size(), 100U);
    });
  auto direction_sub = reader->create_subscription<sensor_msgs::msg::Image>(
    test_namespace + "/terrain_uphill_map", qos,
    [&](sensor_msgs::msg::Image::ConstSharedPtr map) {
      ++directions;
      EXPECT_EQ(map->encoding, "32FC2");
      EXPECT_EQ(map->data.size(), 100U * 2 * sizeof(float));
    });
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(server);
  executor.add_node(reader);
  using namespace std::chrono_literals;
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while ((!costs || !labels || !directions) && std::chrono::steady_clock::now() < deadline)
    executor.spin_once(20ms);
  ASSERT_EQ(costs, 1);
  ASSERT_EQ(labels, 1);
  ASSERT_EQ(directions, 1);
  // Cover the former 500 ms publish period after all retained samples arrived.
  const auto quiet_deadline = std::chrono::steady_clock::now() + 700ms;
  while (std::chrono::steady_clock::now() < quiet_deadline) executor.spin_once(20ms);
  EXPECT_EQ(costs, 1);
  EXPECT_EQ(labels, 1);
  EXPECT_EQ(directions, 1);
}

}  // namespace
