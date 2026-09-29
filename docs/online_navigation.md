# 无先验地图导航：A 方案与 B1 累计地图

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
3. 子目标需要至少推进 `max(0.5 m, 2 × planner.tolerance)`，比当前位置更接近任务目标至少 0.1 m，并满足 `collision_dist + 0.05 m` 的距离场净空。路径长度和目标方向参与候选评分，斜向扩展禁止穿过障碍角点。启用 SMAC 时，回退路径的途经格和斜向两侧格也必须满足 `smac_2d.minimum_clearance`（当前 0.27 m），仅起点允许沿用直接搜索的已有位置豁免。
4. 跟踪过程中约每秒重搜在线路径，到达路径子目标也会触发重搜；始终保留原始任务目标。任务完成需要机器人到达原始目标容差内并减速。
5. 没有可达、净空足够且朝目标推进的子目标时保持停车并重试。在线路径重搜失败时制动，不沿旧路线继续推进。

SMAC 拒绝目标时会区分栅格代价阻断、ESDF 净空不足和距离场查询失败。端点诊断中的 `cost_traversable` 只表示代价通过，`smac_traversable` 同时包含最低净空条件；日志记录实际净空、阈值、距离场状态及快照序号。当前搜索最低净空 0.27 m 比轨迹静态有效阈值 0.26 m 多 1 cm，但平滑和动态安全要求仍可能拒绝轨迹。局部种子起点净空为负时，日志补充坐标、格子代价和快照序号，需结合现场点云判断真实障碍或误占用，不能仅据负值放宽碰撞保护。

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

## 地面灰黑闪烁与邻域地面校验

在累计图的 `map` 配色中，灰色是未知、黑色是占用。先检查实际话题数据是否切换；多个共面的 Map 显示也可能产生视觉干扰。当前现场只读采样已观察到累计图真实的未知→占用变化，局部投影中也有靠近地面、跨度为 0.10–0.15 m 的占用柱。原始薄表面阈值仍为 0.08 m，因此这种柱子可能落入保守障碍分类；现场统计不是对每个占用柱的地面身份确认。

RViz 默认关闭 `ROGMap` 组中的 `Layer Type` 诊断栅格，显示累计图；需要对比局部分类时可手动打开，或暂时关闭累计图，避免共面栅格重叠。

`planner.rog_map.projection.ground_filter` 默认启用邻域校验：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `enable` | `true` | 对模糊低矮占用柱检查邻域平面 |
| `max_span` | `0.15` m | 允许参与校验的最大柱内高度跨度 |
| `plane_tolerance` | `0.075` m | 柱的最低和最高占用高度都必须落在拟合平面此范围内 |

拟合只使用半径 3 格内的原始薄表面，至少需要 8 个样本且目标四侧均有支持，平面残差不超过 0.025 m、坡度不超过 0.7。通过校验的模糊柱成为可通行地面；高于地面容差的凸起、较高墙体、未知邻域和不一致邻域保留障碍。被纠正的厚柱不会继续作为平面种子，避免校验沿障碍扩散。增量更新也会重新校验此前纠正的柱，邻域支持失效时恢复阻断。

这是一项有限容差的地面连续性校验，不能替代完整地面分割；更厚重影、稀疏远距返回和不平地形仍可能保留障碍，需要现场验证。参数需要重启导航生效，历史累计障碍仍由后续有效自由观测逐步清除。

已通过地面/地形 10 项和执行器 32 项测试。隔离 ROS 域的合成噪声点云对比中，测试地面区域的占用格从关闭时的 1784 降为开启后的 0，单独墙体仍占用；该结果不代表真实现场误判已全部消除。201×201、20% 噪声厚柱的二维投影基准中，中位耗时约 1.99→8.84 ms，开启后 p95≈14.56 ms，现场额外耗时尚未验证。

## 查看运行状态

```bash
ros2 param get /nav_executor node.map_source
ros2 topic hz /cloud_registered
ros2 topic echo /nav_executor/global_plan --once
ros2 topic echo /opt_path --once
ros2 run tf2_ros tf2_echo odom base_link
```

RViz 可查看在线 ROGMap、`/nav_executor/global_plan` 和 MINCO 轨迹；在线模式没有静态 `/cost_map`、`/terrain_label_map` 或 `/planning_constraints`。日志中的 `Queued goal` 表示接收任务，`Navigation goal reached` 表示原始任务目标完成；`rog_map_stale` 表示执行侧因在线地图过期制动。

## B1：在线累计全局地图

在线模式默认将新鲜的 ROGMap 二维投影累积到本次启动的 `odom` 地图中。机器人离开局部窗口后，已经观测过的区域仍保留。地图只存在于内存中，重启会清空；没有新增磁盘保存、回环校正或跨次启动定位。

配置位于 `mas2027_nav_executor/config/planner_params.yaml` 的 `planner.online_global_map`，以下参数在启动时读取，修改后需要重启：

| 参数 | 默认值 | 行为 |
|---|---|---|
| `enabled` | `true` | online 模式启用累计地图；prior 模式不创建 |
| `resolution` | `0.1` m | 必须为 ROGMap 分辨率的整数倍，倍数不超过 32 |
| `update_rate_hz` | `2.0` | 按此频率检查并累积新的、新鲜的投影快照 |
| `max_cells` | `2000000` | 扩展后矩形地图的格子数上限，包含未知区域 |
| `occupied_increment` | `3` | 已确认障碍增加的证据值 |
| `free_decrement` | `1` | 完整自由观测减少的证据值 |
| `evidence_limit` | `5` | 证据绝对值上限；自由格要求证据小于零 |
| `occupied_confirmations` | `2` | 新累计障碍需要的连续观测次数；待确认格按未知阻断 |
| `confirmation_max_gap_s` | `1.5` | 连续确认允许的最大时间间隔，超时重新计数 |

粗栅格内任一有原始观测依据的障碍会触发确认；连续两份新的观测支持后才写成累计占用，待确认期间该格为未知、不可通行。实时局部图仍立即阻断障碍。全部细栅格都观测为可通行时，才计入一次自由观测。未知区、投影滤波填充及局部保持生成的障碍不会作为新的累计障碍证据；它们也不会被直接当成自由区。旧障碍需要多次不同时间、不同序号的完整自由观测才能清除；重复快照不会增加证据。确认必须连续，未知观测中断或时间间隔超限后重新计数。此机制减少短暂地面误判被记成障碍；若原始局部投影持续误判地面，仍需现场核查高度跨度、点云重影与地面分类，累计确认不能代替地面识别。达到容量上限的扩展整次拒绝，并保留上一份地图。该实现使用密集矩形栅格，长距离或大面积运行需要评估内存和更新耗时。

发布话题为 `/nav_executor/online_global_map`，类型为 `nav_msgs/msg/OccupancyGrid`，值为未知 `-1`、自由 `0`、占用 `100`，采用 reliable、transient local、深度 1。时间戳对应最近接受的源观测。RViz 配置已加入 **Online Global Map** 显示；可单独查看：

```bash
ros2 topic echo /nav_executor/online_global_map --once \
  --qos-durability transient_local --qos-reliability reliable
```

代码通过 `PathPlanner::globalMapQuery()` 获取一份固定版本的 `MapQueryInterface`，支持坐标换算、逐格占用、批量代价，以及有符号距离场和梯度。一次搜索应持有同一份查询快照，避免地图扩展时原点变化。未知区在距离场中作为障碍，距离截断到 `[-1, 3]` 米，查询图外失败；批量代价值为自由 `0`、占用 `254`、未知 `255`，使用者必须禁止未知区通行。

**本次 B1 完成地图记忆、发布与查询接口。实际任务搜索仍采用 A 方案的局部滚动窗口及推进子目标。** 已用搜索单元测试验证累计地图接口可以查询并规划回到窗口外的历史自由区域；后续 B2 需要将任务的全局路线接到该接口，并保留实时 ROGMap 对局部轨迹和动态障碍的检查。

## 能力范围与验证

A 方案适用于近距离导航、开阔区域和局部可绕行障碍。B1 提供本次启动内的环境记忆，但当前任务搜索仍受局部窗口限制，LIO 没有本模式新增的回环校正；需要先远离目标才能脱出的 U 形死胡同、超出窗口的大型绕行、可靠长距离返航和自动全场探索仍需要接入累计地图的全局规划及探索策略。

已进行编译、无先验 launch 构造检查、搜索与指令安全单元测试，以及隔离 ROS 域中的合成点云和理想底盘闭环。该验证不替代实车：真实雷达稀疏点、遮挡、坡道/隧道、定位漂移、底盘制动及运动中新增障碍仍需现场验收。

B1 新增 10 项累计图测试，连同已有测试共 29 项通过。隔离闭环使用 1.0 m/s 限速，累计图从 81×83 扩为 168×89，确认窗口外的历史自由格仍保留；20 m 目标推进约 9.16 m 后点云中断，过期检查使最终指令归零。运行仍有点云丢帧和初始运动学重试告警，未验证大面积累计性能或实车返航。
