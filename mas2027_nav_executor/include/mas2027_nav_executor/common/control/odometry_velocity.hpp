#pragma once

#include <cmath>
#include <Eigen/Core>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mas2027_nav_executor {

// /Odometry pose and twist describe the same base_link origin. The upstream
// LIO already transfers IMU/lidar velocity to that origin. Match the MPC's
// SE(2) rotation; angular velocity must not manufacture another lever-arm term.
inline Eigen::Vector3d odometryVelocityInPlanningFrame(const nav_msgs::msg::Odometry & odom)
{
  const double yaw = tf2::getYaw(odom.pose.pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const auto & v = odom.twist.twist.linear;
  return {c*v.x-s*v.y,s*v.x+c*v.y,v.z};
}

}  // namespace mas2027_nav_executor
