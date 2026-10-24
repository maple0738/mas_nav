#pragma once

#include <string>

#include "mas2027_decision/types.hpp"

namespace mas2027_decision {

// 点表。坐标含义由 goal_frame 决定：map 模式填地图坐标，纯无图模式填相对起点的坐标。
// 所有点应通过示教得到（tools/teach_points.py），不要手工量取。
struct PointTable {
  bool loaded{false};        // 是否成功从 YAML 读入过

  Waypoint start{};          // 起始位置（仅记录，目前决策不使用）
  Waypoint home{};           // 守家点：基地被攻击时去这里
  Waypoint heal{};           // 回血点：低血量或弹药不足时去这里
  Waypoint fort{};           // 堡垒/补给点（预留，当前决策不使用）
  Waypoint outpost_attack{}; // 打敌方前哨站时的站位
  Waypoint outpost_help{};   // 支援己方前哨站时的站位
  Waypoint patrol_a{};       // 巡逻点 A
  Waypoint patrol_b{};       // 巡逻点 B
  Waypoint enemy_base{};     // 敌方基地：末段冲锋目标
};

// 读取 points.yaml 并写入 table。
// path: YAML 路径；值的写法是 [x, y] 或 [x, y, yaw]，没出现的键保持默认 (0,0,0)。
// table: 输出参数，成功时把 loaded 置为 true。
// error: 输出失败原因；不需要时可以传 nullptr。
// 返回: 成功 true；文件读不到、格式不对或坐标不是有限值时 false。
bool loadPointTable(const std::string& path, PointTable* table, std::string* error);

// 关于场地中线镜像（y -> -y，yaw -> -yaw），用于红蓝方复用同一份点表。
// table: 原始点表。
// 返回: 镜像后的副本，loaded 标志原样保留。
PointTable mirrorPointTable(const PointTable& table);

}  // namespace mas2027_decision
