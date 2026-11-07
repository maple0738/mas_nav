#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <rog_map/rog_map_core/config.hpp>
#include <yaml-cpp/yaml.h>

namespace {
// Exercise the actual ROS loader without creating DDS participants or hardware.
struct ParameterNode {
  std::map<std::string, rclcpp::ParameterValue> values;
  bool has_parameter(const std::string &key) const {
    return values.count(key) != 0;
  }
  void declare_parameter(const std::string &key,
                         const rclcpp::ParameterValue &value) {
    values.emplace(key, value);
  }
  template <typename T>
  bool get_parameter(const std::string &key, T &value) const {
    const auto it = values.find(key);
    if (it == values.end())
      return false;
    value = it->second.get<T>();
    return true;
  }
  rclcpp::Logger get_logger() const {
    return rclcpp::get_logger("terrain_config_test");
  }
};

void loadYaml(const YAML::Node &yaml, const std::string &prefix,
              ParameterNode &node) {
  for (const auto &entry : yaml) {
    const auto key = prefix.empty()
                         ? entry.first.as<std::string>()
                         : prefix + "." + entry.first.as<std::string>();
    if (entry.second.IsMap()) {
      loadYaml(entry.second, key, node);
      continue;
    }
    if (entry.second.IsSequence()) {
      node.values[key] =
          rclcpp::ParameterValue(entry.second.as<std::vector<double>>());
      continue;
    }
    const auto text = entry.second.as<std::string>();
    if (text == "true" || text == "false") {
      node.values[key] = rclcpp::ParameterValue(entry.second.as<bool>());
    } else {
      try {
        node.values[key] = rclcpp::ParameterValue(entry.second.as<int>());
      } catch (const YAML::BadConversion &) {
        try {
          node.values[key] = rclcpp::ParameterValue(entry.second.as<double>());
        } catch (const YAML::BadConversion &) {
          node.values[key] = rclcpp::ParameterValue(text);
        }
      }
    }
  }
}

TEST(TerrainConfigLoading, RealYamlAndOverridesReachSharedConfiguration) {
  auto node = std::make_shared<ParameterNode>();
  const auto perception = YAML::LoadFile(PERCEPTION_CONFIG_FILE);
  loadYaml(perception["/**"]["ros__parameters"], "", *node);
  const auto yaml = YAML::LoadFile(TERRAIN_CONFIG_FILE);
  loadYaml(yaml["/**"]["ros__parameters"], "", *node);
  rog_map::Config config;
  EXPECT_NO_THROW(config.loadFromRosNode(node, "planner.rog_map"));
  EXPECT_DOUBLE_EQ(config.terrain.undulating.min_peak_to_trough_m, .045);
  EXPECT_DOUBLE_EQ(config.terrain.undulating.max_trend_gradient, .12);
  EXPECT_DOUBLE_EQ(config.terrain.undulating.min_neighbor_correlation, .85);
  EXPECT_TRUE(config.terrain.undulating.front_signature_enable);
  EXPECT_DOUBLE_EQ(config.terrain.undulating.front_crest_height_tolerance_m, .04);
  EXPECT_EQ(config.terrain.undulating.front_approach_cells, 5);
  EXPECT_DOUBLE_EQ(config.terrain.undulating.front_approach_tolerance_m, .03);
  EXPECT_DOUBLE_EQ(config.tunnel_height_delta_max, .75);
  node->values
      ["planner.rog_map.projection.terrain.undulating.min_peak_to_trough_m"] =
      rclcpp::ParameterValue(.055);
  node->values["planner.rog_map.projection.terrain.undulating.front_min_lateral_support"] =
      rclcpp::ParameterValue(2);
  node->values["planner.rog_map.projection.terrain.undulating.front_approach_cells"] =
      rclcpp::ParameterValue(3);
  node->values["planner.rog_map.projection.terrain.slope.fit_radius_cells"] =
      rclcpp::ParameterValue(7);
  node->values
      ["planner.rog_map.projection.terrain.tunnel.min_neighbor_support"] =
      rclcpp::ParameterValue(8);
  config.loadFromRosNode(node, "planner.rog_map");
  EXPECT_DOUBLE_EQ(config.terrain.undulating.min_peak_to_trough_m, .055);
  EXPECT_EQ(config.terrain.undulating.front_min_lateral_support, 2);
  EXPECT_EQ(config.terrain.undulating.front_approach_cells, 3);
  EXPECT_EQ(config.terrain.slope.fit_radius_cells, 7);
  EXPECT_EQ(config.terrain.tunnel.min_neighbor_support, 8);
  node->values["planner.rog_map.projection.terrain.undulating.min_neighbor_"
               "correlation"] = rclcpp::ParameterValue(1.2);
  EXPECT_THROW(config.loadFromRosNode(node, "planner.rog_map"),
               std::invalid_argument);
}
} // namespace
