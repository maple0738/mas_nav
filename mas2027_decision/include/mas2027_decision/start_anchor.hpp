#pragma once

#include <cmath>

#include "mas2027_decision/types.hpp"

namespace mas2027_decision {

// 纯无图模式的起点锚定：把第一帧 odom 位姿记为场坐标系原点。
//
// 前提是每次比赛把车摆在同一个物理位置。它不修正运行期漂移，只是让决策层
// 有一个"场坐标系"可用；一旦切到地图锚定模式（map -> odom 由 odom_localizer
// 提供），就不需要这个类。
class StartAnchor {
 public:
  // 是否已经用第一帧位姿锚定过。
  bool ready() const { return ready_; }

  // 清除锚定，等待下一次 init()。换场或重开比赛时调用。
  void reset() {
    ready_ = false;
    origin_x_ = 0.0;
    origin_y_ = 0.0;
    origin_yaw_ = 0.0;
  }

  // 用第一帧 odom 位姿初始化。已经初始化过则直接返回（不会覆盖原点）。
  // x, y: 起始位置的 odom 坐标，米。
  // yaw: 起始朝向的 odom 角度，弧度。
  void init(double x, double y, double yaw) {
    if (ready_) {
      return;
    }
    origin_x_ = x;
    origin_y_ = y;
    origin_yaw_ = yaw;
    cos_ = std::cos(-yaw);
    sin_ = std::sin(-yaw);
    ready_ = true;
  }

  // 把 odom 坐标换算到场坐标系。
  // x, y: odom 系下的位置（米）。
  // 返回: 相对起点的坐标（米），起点为 (0,0)，朝向与起点一致时即为 odom 朝向。
  Vec2 toField(double x, double y) const {
    const double dx = x - origin_x_;
    const double dy = y - origin_y_;
    return {cos_ * dx - sin_ * dy, sin_ * dx + cos_ * dy};
  }

  // 把场坐标系坐标换算回 odom。
  // p: 场坐标系下的点（米）。
  // 返回: odom 系下的位置（米）。
  Vec2 toOdom(const Vec2& p) const {
    return {origin_x_ + cos_ * p.x + sin_ * p.y,
            origin_y_ - sin_ * p.x + cos_ * p.y};
  }

  // 把 odom 朝向换算到场坐标系。
  // odom_yaw: odom 系下的朝向（弧度）。
  // 返回: 相对起始朝向的角度（弧度）。
  double fieldYaw(double odom_yaw) const { return odom_yaw - origin_yaw_; }

 private:
  bool ready_{false};        // 是否已锚定
  double origin_x_{0.0};     // 起点 odom x，米
  double origin_y_{0.0};     // 起点 odom y，米
  double origin_yaw_{0.0};   // 起点 odom 朝向，弧度
  double cos_{1.0};          // cos(-origin_yaw)，缓存避免每帧重算
  double sin_{0.0};          // sin(-origin_yaw)，缓存
};

}  // namespace mas2027_decision
