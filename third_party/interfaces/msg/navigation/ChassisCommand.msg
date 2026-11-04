# Velocity and chassis mode belong to the same control tick.
std_msgs/Header header
float32 vx
float32 vy
uint8 mode
# Remote-style small-yaw position command relative to lower auto_yaw_offset.
# Degrees, counterclockwise positive, limited to [-180, 180]. Hold at alignment.
# The upper MPC feeds forward MINCO tangent yaw; measured LIO axis error trims
# this angle so base_link +X/-X stays collinear with the path.
float32 small_yaw_ref
# Request follow-gimbal mode: true=enter/hold following, false=no request.
bool follow_gimbal
