# 无先验地图导航：A 方案

## 启动

本模式使用 `small_point_lio` 的实时里程计与点云、进程内 ROGMap、MINCO 和 MPC。启动时不读取 `navigation_map.yaml`，不需要 PCD、PGM、地图 YAML 或 terrain MSGPack，不启动先验 GICP 定位器和静态地图服务。

先在 `mas2027_nav_executor/config/node_params.yaml` 中将 `/** → ros__parameters → node → map_source` 设为 `online`。整套启动模式仅从该 YAML 读取。

```bash
cd /home/mas/mas_nav_2027_native
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mas2027_nav_executor mas2027_nav_bringup \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 观察在线导航，关闭底盘通信桥。
ros2 launch mas2027_nav_bringup nav_executor_launch.py \
  use_ros2_comm:=False
```

需要底盘执行时使用 `use_ros2_comm:=True`。没有显示器时增加 `use_rviz:=False`。切回先验地图模式时，将上述 YAML 中的 `map_source` 改为 `prior`。

`map_source` 是启动时选择，切换模式需要重启。在线模式即使 `use_odom_localizer` 保持默认 `True`，也不会启动先验定位器。

## 坐标系与目标

- 本次启动的 `odom` 是规划和目标基准；TF 链为 `odom → base_link → lidar_link`。
- `tf_maintainer` 继续发布 `odom → base_link`，在线模式关闭其 `map → odom` 发布。
- RViz 自动使用 `odom` 作为 Fixed Frame；在在线点云与地图上点击目标即可。
- 目标可位于滚动地图之外或尚未观测的区域；接收目标时仍要求当前机器人位置的 ROGMap 查询有效，以及地图、里程计新鲜。
- 其他坐标系的目标必须能通过 TF 转换到 `odom`。在线模式没有固定场地坐标定位；重启后相同 `odom` 坐标不保证代表相同现场位置。

例如发送本次启动坐标中的 `(2, 0)`：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

## 在线规划行为

```text
/Odometry + /cloud_registered
          ↓
ROGMap：10×10 m 滚动地图、二维投影、距离场
          ↓
窗口内目标路径 / 已观测自由区中的推进子目标
          ↓
MINCO → 轨迹安全检查 → MPC → 指令安全检查
```

1. 目标可通行且在窗口内时，使用已有 SMAC 2D/A* 搜索。
2. 目标在窗口外、未知或暂不可达时，在可达区域中选择朝任务目标推进的子目标。候选不必位于地图窗口边缘，因此激光已观测区域小于窗口时也能推进。
3. 子目标需要至少推进 `max(0.5 m, 2 × planner.tolerance)`，比当前位置更接近任务目标至少 0.1 m，并满足 `collision_dist + 0.05 m` 的距离场净空。路径长度和目标方向参与候选评分，斜向扩展禁止穿过障碍角点。
4. 跟踪过程中约每秒重搜在线路径，到达路径子目标也会触发重搜；始终保留原始任务目标。任务完成需要机器人到达原始目标容差内并减速。
5. 没有可达、净空足够且朝目标推进的子目标时保持停车并重试。在线路径重搜失败时制动，不沿旧路线继续推进。

在线模式不创建静态 `TerrainGrid`，局部优化器、走廊和轨迹检查直接查询 ROGMap；先验模式继续使用 `FusedMapQuery`。

## 未知区域、速度与地形

在线启动配置统一设置：

| 设置 | 在线值 |
|---|---|
| `planner.exploration.unknown_as_occupied` | `true` |
| `planner.rog_map.projection.unknown_as_occupied` | `true` |
| `planner.rog_map.projection.prior_map.enable` | `false` |
| `planner.allow_unknown` | `false` |
| MINCO 最大速度 / 加速度 | 从 `planner_params.yaml` 读取，当前 `4.0 m/s` / `5.0 m/s²` |
| 普通、隧道、起伏路段速度 / 加速度上限 | 从 `region_control.yaml` 读取，当前 `3.0 m/s` / `4.0 m/s²` |
| 坡道速度 / 加速度上限 | 从 `region_control.yaml` 读取，当前 `4.0 m/s` / `5.0 m/s²` |

在线模式不会覆盖 YAML 中的速度或加速度配置。修改速度时同时核对 `planner_params.yaml` 的 MINCO 总上限、`mpc_params.yaml` 的控制约束，以及 `region_control.yaml` 的区域上限；实际执行速度仍受轨迹、区域控制和净空检查约束。配置修改后重启生效。此前合成闭环验证使用的是 `1.0 m/s` / `1.0 m/s²` 配置，不能作为提速后的运行验证。

未知柱同时在投影代价和距离场中作为障碍。规划只能执行当前观测支持的自由区域；随着新的点云到达，滚动地图扩展可用空间。直接运行节点、绕过 launch 配置时，在线模式会校验探索与投影的未知策略，以及先验融合开关，错误配置会拒绝初始化。

里程计超时、ROGMap 超时、距离查询失败、轨迹碰撞或 MPC 指令前视净空不足仍阻止运动；在线模式只移除对静态地图及其 TF 的要求。

区域控制使用新鲜的 `/rog_map/terrain_label` 在线坡道 `5`、隧道 `6` 标签；缺失、未知或过期时使用普通 mode。当前在线识别不提供起伏地形 `7`。模式窗口冲突会保持该轨迹，等待有效标注；坡道和隧道识别仍需现场标定。

## 查看运行状态

```bash
ros2 param get /nav_executor node.map_source
ros2 topic hz /cloud_registered
ros2 topic echo /nav_executor/global_plan --once
ros2 topic echo /opt_path --once
ros2 run tf2_ros tf2_echo odom base_link
```

RViz 可查看在线 ROGMap、`/nav_executor/global_plan` 和 MINCO 轨迹；在线模式没有静态 `/cost_map`、`/terrain_label_map` 或 `/planning_constraints`。日志中的 `Queued goal` 表示接收任务，`Navigation goal reached` 表示原始任务目标完成；`rog_map_stale` 表示执行侧因在线地图过期制动。

## 能力范围与验证

A 方案适用于近距离导航、开阔区域和局部可绕行障碍。滚动地图没有长期环境记忆，LIO 没有本模式新增的回环校正；需要先远离目标才能脱出的 U 形死胡同、超出窗口的大型绕行、可靠长距离返航和自动全场探索仍需要在线全局地图及探索策略。

已进行编译、无先验 launch 构造检查、搜索与指令安全单元测试，以及隔离 ROS 域中的合成点云和理想底盘闭环。该验证不替代实车：真实雷达稀疏点、遮挡、坡道/隧道、定位漂移、底盘制动及运动中新增障碍仍需现场验收。
