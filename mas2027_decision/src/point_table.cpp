#include "mas2027_decision/point_table.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <exception>
#include <vector>

namespace mas2027_decision {

namespace {

// 把一个 YAML 序列解析成 Waypoint，支持 [x, y] 或 [x, y, yaw]。
// node: YAML 节点；wp: 输出参数。返回 false 表示格式不对或坐标非有限值。
bool parseWaypoint(const YAML::Node& node, Waypoint* wp) {
  if (!node || !node.IsSequence()) {
    return false;
  }
  if (node.size() != 2 && node.size() != 3) {
    return false;
  }
  try {
    wp->x = node[0].as<double>();
    wp->y = node[1].as<double>();
    wp->yaw = node.size() == 3 ? node[2].as<double>() : 0.0;
  } catch (const YAML::Exception&) {
    return false;
  }
  return std::isfinite(wp->x) && std::isfinite(wp->y) && std::isfinite(wp->yaw);
}

}  // namespace

bool loadPointTable(const std::string& path, PointTable* table, std::string* error) {
  if (!table) {
    return false;
  }
  if (path.empty()) {
    if (error) {
      *error = "points_file is empty";
    }
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    if (error) {
      *error = std::string("failed to read points file: ") + e.what();
    }
    return false;
  }

  if (!root.IsMap()) {
    if (error) {
      *error = "points file must be a flat map of name: [x, y(, yaw)]";
    }
    return false;
  }

  const std::vector<std::pair<std::string, Waypoint*>> entries{
      {"start", &table->start},           {"home", &table->home},
      {"heal", &table->heal},             {"fort", &table->fort},
      {"outpost_attack", &table->outpost_attack},
      {"outpost_help", &table->outpost_help},
      {"patrol_a", &table->patrol_a},     {"patrol_b", &table->patrol_b},
      {"enemy_base", &table->enemy_base},
  };

  for (const auto& [key, target] : entries) {
    if (!root[key]) {
      continue;
    }
    if (!parseWaypoint(root[key], target)) {
      if (error) {
        *error = "invalid point: " + key + " (expect [x, y] or [x, y, yaw])";
      }
      return false;
    }
  }

  table->loaded = true;
  return true;
}

PointTable mirrorPointTable(const PointTable& table) {
  PointTable out = table;
  for (Waypoint* wp : {&out.start, &out.home, &out.heal, &out.fort,
                       &out.outpost_attack, &out.outpost_help, &out.patrol_a,
                       &out.patrol_b, &out.enemy_base}) {
    wp->y = -wp->y;
    wp->yaw = -wp->yaw;
  }
  return out;
}

}  // namespace mas2027_decision
