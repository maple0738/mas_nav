# MPC 一个预测时域内的期望状态序列。
# 来源：navi_minco_bit 的 src/ros_interfaces/msg/minco_traj/MpcPositionCommand.msg。
# 本仓库改动：cmds 的类型前缀 ros_interfaces/ -> interfaces/，
# 因为消息落在本仓库既有的 interfaces 包里，而不是上游的 ros_interfaces 包。
std_msgs/Header header
interfaces/PositionCommand[] cmds
uint32 mpc_horizon
uint8 command_flag
uint8 NORMAL_COMMAND = 1
uint8 BLOCK_COMMAND = 137