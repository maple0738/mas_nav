# MAS 2027 原生导航

ROS 2 Jazzy，运行链路统一为独立 `nav_executor`：不依赖 Nav2 server、Behavior Tree、Costmap 插件、
`minco_planner` / `minco_controller` 插件包。唯一入口 `mas2027_nav_bringup/nav_executor_launch.py`。
本文件是链路、TF、话题与排障总说明；专题文档见[在线导航](docs/online_navigation.md)、[现场记录与调试](docs/field_debugging.md)和[RoboMaster 无图导航优化方案](docs/robomaster_no_map_navigation.md)。

## 组件职责

| 组件 | 职责（输入 → 输出） |
|---|---|
| `mid360_driver` | 双 MID360 驱动内融合到参考雷达系 → `/mid360_driver/lidar`、`/mid360_driver/imu` |
| `small_point_lio` | 融合点云 + IMU → `/Odometry`、`/cloud_registered` |
| `odom_localizer` | GICP 六自由度先验定位（`lab3.pcd`）→ `/tf_maintainer/map_to_odom` |
| `tf_maintainer` | 唯一动态 TF 发布者：`map→odom`、`odom→base_link` |
| `terrain_map_server` | 静态地形服务 → `/cost_map`、`/terrain_label_map` |
| `nav_executor` | TaskManager + ROGMap + SMAC/MINCO 规划 + MPC + 区域控制 → `/opt_path`、`/cmd_vel`、`/nav_executor/chassis_cmd` |
| `nav_serial_bridge`（可选） | 消费 `/nav_executor/chassis_cmd` → CDC1 导航专用串口 |
| `robot_state_publisher` | URDF → 车体 / 雷达外参 |

`ROGMap` 在 `nav_executor` 进程内（`nav_executor_planner`），无独立节点，参数在 `planner.rog_map.*`；
静态地形代价图是搜索、轨迹验收和 MPC 制动的规划输入；区域 mode 优先采用 ROGMap 在线坡道/隧道标签，缺失时采用静态标签图。没有 `Global/Local Costmap` 是预期行为。

## 在线链路

```text
MID360 ×2 → mid360_driver → small_point_lio → /Odometry + /cloud_registered
                     ├─→ odom_localizer → map→odom → tf_maintainer → odom→base_link
                     └─→ ROGMap（进程内：在线占据 / 距离场 / 坡道隧道语义）

lab3_terrain.msgpack → terrain_map_server → /cost_map + /terrain_label_map
                           → SMAC 2D 搜索 → MINCO 轨迹 → MPC → 区域控制
ROGMap → /rog_map/terrain_label ──────────────────────────↑
```

- 全局主搜索是 **SMAC 2D**，在 ROGMap 动态格和坡道侧边约束上搜索；`planner.use_smac:false` 时使用 Astar。输出仍是纯几何折线。
- 局部轨迹由 **MINCO** 生成，并经过发布前整条轨迹校验和执行前指令门。
  全局折线进局部滑窗后逐段验净空：被截断则窗内重搜接回终点，完全封死则只发障碍前停车前缀（零末速度）。
- 热启动重规划只复用旧轨迹剩余段最近 `replan_continuity_time_s`（默认 0.3 s），作为 MINCO 初值前缀；
  如果旧前缀末端与新路径拼接跳跃超过 0.6 m，会自动缩短/放弃旧前缀，避免旧路线拖住新规划。
- 执行许可由 `TaskManager` 管（`MincoFsm` 已删）：新目标抢占后旧轨迹失效；里程计超
  `node.odom_timeout_s` 停规划与执行。任一雷达掉线自动降级单雷达（含 IMU 切换）、恢复后自动配对，
  参数见 `small_point_lio_params.yaml` 的 `merge_*`、`packet_resync_silence`。

## 关键话题

| 话题 | 说明 |
|---|---|
| `/goal_pose`、`/Odometry`、`/cloud_registered` | 目标入口、里程计、配准点云 |
| `/cost_map`、`/terrain_label_map` | 静态地形代价 / 原始区域标签（当前地图已有平地坡道模式测试区） |
| `/rog_map/terrain_label` | 在线坡道 `5` / 隧道 `6` 候选标签；有效在线结果优先用于区域 mode |
| `/opt_path`、`/cmd_vel`、`/nav_executor/chassis_cmd` | MINCO 轨迹、观测用速度、速度与模式的同周期底盘命令 |
| `/nav_executor/global_plan`、`/nav_executor/debug/global_plan` | 全局折线的 Path / Marker（0.15 m 青粗线） |
| `/nav_executor/minco_path` | MINCO 局部优化轨迹的 Path 可视化 |
| `/nav_executor/debug/minco_trajectory` | MINCO 轨迹 Marker（0.07 m，按速度染色） |
| `/planning_constraints`（+`_markers`） | 静态地形规划约束图 |

## TF 与坐标系

`map → odom → base_link → lidar_link`。`small_point_lio.publish_odom_tf=false`、
`odom_localizer.tf.publish_direct=false`，动态 TF 统一由 `tf_maintainer` 发布。控制坐标系是 `odom`，
输出可用 `node.output_in_body_frame` 切到车体系。

## 净空判据与排障

安全链路只保留两道硬门：

1. **发布前**：`MincoPlanner::validateTrajectory()` 检查整条 MINCO 轨迹的动力学、终点、地形和净空；
2. **执行前**：`PathExecutor::checkCommandSafety()` 每个控制周期检查下一段指令的 ROGMap 时效、地形 transition 和近场/远场净空。

两道门共用 `common/environment/clearance_gate.hpp` 的同一阈值：

```text
effective_clearance = max(0, collision_dist - kEsdfJitterTolerance)
                    = max(0, 0.28 - 0.02) = 0.26 m
```

不再有独立的 20 Hz 全轨迹监视，也没有 `monitor_margin`、`replan_react_time`、`speed_aware_clearance` 等第二套速度相关口径。执行侧指令门是动态障碍的最终兜底；轨迹发布门防止明显不安全的整条轨迹进入跟踪。

- **近场放宽**：起点弧长 `collision_dist` 以内只要求「不比当前实测净空更差」（留 0.02 m），否则贴墙停车后发目标点车不动。日志 `Near-field exemption active: ... X ... Y ... Z` 中 X 是实测净空、Y 是下调后要求、Z 是完整要求，`X > Z` 属正常。
- **搜索启发式不是安全门**：SMAC 的 `minimum_clearance: 0.27` 和在线子目标 `collision_dist`(0.28) 只影响搜索/回退；最终仍由上面两道硬门验收。目标格被占据或净空不足时，沿 `start → goal` 反方向最多回退 `goal_approach_max_shorten`（默认 1.5 m）找可通行接近点，不放松最终阈值。
- **局部兜底**：正常路径 → 窗内绕行 → 完整停车前缀（安全段需 0.45 m）都失败时，发一个不差于当前净空、末速为零的短前缀脱困；`planner.minco_optimizer.stuck_escape.enable: false` 可关闭。
- **不再因引导/语义失败急停**：在线全局刷新失败但当前局部轨迹未过期时继续跟随；在线语义标签冲突时回退无特殊区段的普通 profile。

失败原因按 `failure_log_first_n`(10) 逐条、每 `failure_log_every_n`(25) 次采样、每 `failure_summary_every`(50) 次汇总。`LOCAL_SEED_INVALID` / `..._AFTER_REPAIR` = 没进优化器；`OPTIMIZER_FAILED` = 不收敛；`COLLISION` / `TERRAIN_COLLISION` = 硬校验被否。局部兜底全败时 `Live obstacle blocks the local route...` 会带 `*_reject=` 现场与自动判读 `verdict=`：`GEOMETRY` 连近场规则都过不了（拒绝正确，别动阈值）、`SEED_GATE_STRICTER` 只被更严的种子门否掉、`PREFIX_TOO_SHORT` 卡安全段长度、`TERRAIN` 死在 terrain/动态层、`NONE` 未捕获。

反复急停时先看 `Braking:` 的 `reason=`：`rog_map_stale` / `tf_unavailable` 是数据新鲜度问题，`clearance` 是执行侧指令净空，`terrain_transition` 是地形方向门；不要直接减小 `collision_dist`。窄道碰撞与轨迹拒绝的现场核对步骤见[现场记录与调试](docs/field_debugging.md#窄道碰撞与轨迹拒绝)。

## ROGMap 与 RViz

- ROGMap 订阅 `/cloud_registered` + `/Odometry`；RViz `ROGMAP` 分组默认显示紫色 `Occupied`（0.05 m）
  与 `2D Distance Field`。先验图融合关闭，`Static Layer Value` 不代表地形图；投影高度、地面/墙体
  阈值与 `planner.minco_optimizer.collision_dist` 需在静止实车上标定。已移除目标跟踪与未来预测（防振动误判）。
- MPC 仍为全向模型（`linear.x/y`、`angular.z`，内部角速度限制 ±2 rad/s、角加速度限制 ±4 rad/s²，需按底盘标定）；最终 `w` 由下面的角速度配置覆盖。
  区域控制沿最终轨迹标注坡道、隧道和起伏路，以 `/nav_executor/chassis_cmd` 同步发布速度与
  mode 及小 yaw 对齐请求；`/cmd_vel` 保留供观测。`nav_serial_bridge` 从同一消息提取平移速度、相对下位机 auto_yaw_offset 的遥控式位置指令 `small_yaw_ref` 和 `follow_gimbal`，
  通过导航专用 CDC1 串口直接发送；下发包没有 `w`、导航状态或 `mode` 字段。
  普通路段 ROS 中的 `w` 为 `+2 rad/s`，特殊跟随显示 MPC 内部角速度，均不经串口发送；下位机自动模式在导航在线且平移速度有效时自行固定 `+4 rad/s`，离线或速度无效时清零。
  区域 mode 只在导航内部生效；特殊地形的底盘朝向闭环尚需相对角度 `δ` 反馈。
- `lab3_terrain.msgpack` 只有平地/障碍，无坡道台阶与方向数据；方向层实测全 0，逐边方向约束只在
  `validateTrajectory` 的 `terrain->transition()`（失败记 `TERRAIN_COLLISION`）。
- **折线与轨迹是两条线**：粗青线 = 搜索想怎么绕，细彩线 = 车准备怎么走，明显不重合时问题在 MINCO
  走廊/净空。粗青线末段从 `planner.tolerance`(0.30) 跳到精确目标点。RViz 视图
  `nav_executor_view.rviz`：`HW Cost Map` / `HW Direction Map` 显示两个地形话题，`Planning Constraints`
  默认稀疏点层（红 = 不可通行、黄 = 仅限语义方向），完整栅格需手动开 `(raster diagnostic)`。
- 点目标无路径：先看 `Received goal on /goal_pose`（`Queued goal` 只代表入队），再看
  `MINCO trajectory not published` 的原因；出现 `Trajectory clearance ... below required ...`
  先核实配准与实际障碍距离，不要减小 `collision_dist`。

## 环境与编译

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd /home/mas/mas_nav_2027_native
rosdep install -r --from-paths src --ignore-src --rosdistro jazzy -y

colcon build --symlink-install --parallel-workers 4 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

额外系统库：`ros-jazzy-rmw-cyclonedds-cpp libdw-dev libomp-dev python3-pip`。

### 代码跳转（clangd）

`CMAKE_EXPORT_COMPILE_COMMANDS=ON`（已在上面的构建命令里）让每个包导出自己的编译命令，再把它们合并成
一份给编辑器用：

```bash
bash src/tools/gen_compile_commands.sh   # 重新 configure 各包，合并到 build/compile_commands.json
```

编译库只能放 `build/`（不入库），所以仓库里用 `src/.clangd` 显式指向它——这几行配置与编辑器无关，
clangd 打开任意文件都能拿到正确的 `-I`/`-D`（ROS 2 Jazzy、PCL、Eigen、本工作区各包头文件）。
`src/compile_commands.json` 是一条指向同一文件的软链（不入库），给只认目录祖先搜索的工具兜底。

**新增/删除源文件、切分支、改过 `CMakeLists.txt` 之后要重跑一次**；只改注释或函数体不必重跑。
症状对照：编译库里没有该文件时，clangd 会拿别的文件的命令去猜，表现为满屏
`'xxx.hpp' file not found`、`use of undeclared identifier`，同时跨文件跳转失效。

## 启动

```bash
source /opt/ros/jazzy/setup.bash
source /home/mas/mas_nav_2027_native/install/setup.bash
ros2 launch mas2027_nav_bringup nav_executor_launch.py
```

导航从特殊地形入口前的 `PREPARING` 阶段开始发送 `follow_gimbal=1` 和角度制 `small_yaw_ref`，经过 `ARMED`、`COMMITTED`、`INSIDE` 和 `RELEASING` 阶段持续跟随，到释放点后清除请求。普通路段不请求跟随。无轨迹/许可、失效输入或串口指令超时会清除跟随请求并停止平移；串口桥退出前发送零速度、无跟随请求的最后一帧。

特殊地形跟随按轨迹轴线对齐：`gimbal_follow.allow_reverse=true`，在轨迹切线及其反方向中选择较近朝向；正反方向切换使用 10° 滞回。`base_link` 的 +X 就是云台正方向，因此 +X 或 -X 与轨迹共线即可，不要求车头一定朝前；同一 MPC 的整个预测窗采用相同选择，MINCO 位置和世界系速度不反号。角度指令由 MPC 角速度前馈加上 LIO 实测轴线误差修正（`heading_error_gain_s=1.0`、`heading_deadband_deg=0.5`、`max_heading_rate_deg_s=120`），所以安全门只停止平移时仍会继续把云台轴线转回轨迹。跟随阶段等待所选轴线误差进入 5°并稳定 0.3 s；健康 MPC 周期承接上一拍命令继续加速，仅在失效/拒绝时锚到实测速度。普通路段上限仍为 3 m/s，角速度/角加速度限制仍为 ±2 rad/s、±4 rad/s²。跟随阶段及参数见[区域控制说明](mas2027_nav_executor/docs/region_control.md)。

`force_follow_gimbal` 启动参数已删除；跟随请求完全由特殊地形区域阶段控制。修改跟随控制参数需重启导航，不需要改下位机。

常用启动参数：`use_rviz`、`use_odom_localizer`、`use_nav_serial`、`nav_serial_device`、`output_topic`。
启动模式仅由 `mas2027_nav_executor/config/node_params.yaml` 中的 `node.map_source` 选择，修改后重启生效。
设为 `prior` 时，先验地图文件统一在 `mas2027_nav_bringup/config/navigation_map.yaml` 中选择。
设为 `online` 时使用无先验模式：不读取地图文件、不启动 GICP 定位器和静态地图服务，
使用 `odom` 下的 LIO 与在线 ROGMap 分段推进；速度与加速度采用规划器、MPC 和区域控制 YAML，未知区域作为障碍。
在线模式默认启用 B1 累计地图，在 `/nav_executor/online_global_map` 保留本次启动内的历史观测，并提供固定版本的全局查询接口；当前任务搜索仍使用 A 方案，后续 B2 接入累计地图路线。
具体启动、目标坐标和能力范围见 [无先验地图导航](docs/online_navigation.md)。
`use_nav_serial` 默认 `True`，通过 CDC1 发送 `vx`、`vy`、小 yaw ref 和请求标志；只想观察导航时置
`False`。导航桥默认读取 USB 接口自身的 `iInterface`，仅连接字符串为 `PCCOMM-NAV` 的
CDC 串口，不依赖 `/dev/ttyACM*` 编号、`if02` 顺序或设备产品名称；
`nav_serial_device` 可显式指定串口路径。RViz 的 2D Goal Pose 发到 `/goal_pose`。

### 只有控制板时检查串口频率

不要同时启动整套导航和独立串口桥。确认导航接口对应的串口没有其他占用后，按上面的方式加载
ROS 环境，单独运行 `ros2 run nav_serial_bridge nav_serial_bridge_node`，保持约 30 秒再按 Ctrl+C。
没有 ROS 底盘命令时，桥发送 `vx=vy=0`、小 yaw 请求清零的 16 字节帧；自动模式的下位机只要导航通信在线，零平移时仍固定 `+4 rad/s` 自转。每 5 秒的 `CDC1` 日志给出：

- `tx_ok` 和括号内的 Hz：主机成功写入串口的帧数与平均频率，目标约 50 Hz；
  `tx_failed` 为短写或系统调用失败数，`max_tx_gap` 为相邻成功写入调用的最大间隔。
- `rx_valid` 和 Hz：通过 CRC 的下位机回传帧数与平均频率，当前板子约 500 Hz；
  `rx_crc_bad` 为 CRC 不符的候选帧数，`rx_discarded` 为同步时丢弃的字节数。

这些数字能检查本机定时器、串口写入和回传帧质量。当前固件回传帧没有下行命令的序号或确认计数，
所以 `tx_ok` **不能证明板子收到了每一帧**，`rx_valid` 也无法严格统计板子发出后在上行丢失的帧。
要测真实端到端丢包率，需要在固件接收侧计数，或给下行加序号并在上行回显；改协议时上位机和固件须同时更新。

### 同时检查视觉 CDC0 与导航 CDC1

控制板单独接 USB 时，可手动在两个终端同时运行。先在视觉工程 `/home/mas/桌面/mas_vision`
执行 `cmake --build build --target vision_serial_bench -j4`。导航桥默认查找
`PCCOMM-NAV` 接口；视觉程序的端口配置需另行确认。

终端一从本导航工作区运行：

```bash
source install/setup.bash
ros2 run nav_serial_bridge nav_serial_bridge_node
```

终端二从视觉工程运行：

```bash
cd /home/mas/桌面/mas_vision/build
./vision_serial_bench 30
```

视觉台架入口不启动相机，只在 CDC0 发送零目标角、`fire_advice=0`。导航桥只接收
`/nav_executor/chassis_cmd` 的导航系统指令；没有新指令或指令超过 300 ms 时在 CDC1
发送零平移速度、小 yaw ref 和请求标志均为零的 16 字节帧。下位机自动模式的自转由下位机自身生成。不要同时运行视觉 `./base` 或第二个
导航桥。视觉每 3 秒的 `[serial-rate]` 目标为发送约 250 Hz、接收约 500 Hz，
`tx_err`、`crc_errors`、`dropped_bytes`、`queue_dropped` 应为 0；导航每 5 秒的
`CDC1` 目标为发送约 50 Hz、接收约 500 Hz，写入和 CRC 错误应为 0。
这些是主机串口写入与有效回传计数；板端收到多少下行帧仍需固件接收计数或序号回显确认。

## 核心配置

- `mas2027_nav_executor/config/node_params.yaml`：频率、话题、坐标系、轨迹/里程计/ROGMap 超时。
- `mas2027_nav_executor/config/planner_params.yaml`：全局搜索（`use_smac`、`smac_2d.*`、`tolerance`）、
  MINCO 与恢复参数、`planner.rog_map.*`、`planner.online_global_map.*`。
- `mas2027_nav_executor/config/mpc_params.yaml`：MPC 约束与权重。
- `mas2027_nav_bringup/config/small_point_lio_params.yaml`：雷达与 LIO（含双雷达降级超时）。
- `mas2027_nav_bringup/config/navigation_map.yaml`：导航地图唯一选择入口（定位 PCD、二维地图
  YAML、terrain msgpack）。
- `mas2027_perception/Localization/odom_localizer/config/params.yaml`：先验 PCD 定位。
- `mas2027_nav_bringup/map/`、`pcd/`：`navigation_map.yaml` 所选的地图产物目录；当前选择
  `lab_map_20260921_211523` 这一组（先验模式）。

源码：搜索与轨迹在 `src/path_planner/`，MPC 与安全监测在 `src/path_executor/`，
第三方数值后端与 qpOASES 在 `vendor/`（均在 `mas2027_nav_executor` 内）。

## 地图更新

同一坐标系的一组导航地图由三份配置关联：PCD（GICP 定位）、Nav2 PGM/YAML（地图原点与
分辨率元数据）和 terrain msgpack（静态地形规划与 RViz）。定位用 GICP 的完整六自由度
`map→odom`；改点云或雷达外参后先在 RViz 确认地板、天花板重合。

**实时障碍由 ROGMap 独占负责**。主导航栈已删除 `/dynamic_cost_map`、其心跳闭锁、
二维动态格融合和对应可视化；ROGMap 在在线快照、局部种子路径、轨迹发布前整条检查
与执行前指令前视中仍保持 fail closed。`node.rog_map_timeout_s` 直接检查进程内 ROGMap
最后一次完成快照的时间：点云断流或工作线程停滞时拒绝新目标并制动，不需要替代心跳话题。
`map_server` 仍然必要，但职责只是发布静态
`/cost_map` 和 `/terrain_label_map`，已不依赖点云、先验 PCD、PCL 或 TF。

建图与换图（不进在线链路）统一走 `/home/mas/mapping_web_ui` 的三维建图控制台。本仓库不再自带
`pcd2pgm`、`save_pcd_and_make_map.sh`、`pgm_to_terrain_msgpack.py` 与 `map_edit`：控制台一次完成
累计点云 → PCD、二维占用图切片、PGM 修图、map 原点/朝向统一与 PGM→terrain 语义标注。它直接从
`/cloud_registered` 累计，所以导出的 PCD 天然就是先验定位要的坐标系，不需要旧脚本那一步
`T_odom_from_internal` 纠正。控制台默认把地图写在自己的 `data/` 下，产物要手动拷进本仓库：

```bash
cd /home/mas/mapping_web_ui && ./run.sh      # 建图 → 结束并保存 → 地图编辑 → 生成 terrain MSG

DST=/home/mas/mas_nav_2027_native/src/mas2027_nav_bringup
cp data/pcd/<名称>.pcd              "$DST"/pcd/
cp data/map/<名称>.pgm              "$DST"/map/
cp data/map/<名称>.yaml             "$DST"/map/
cp data/map/<名称>_terrain.msgpack  "$DST"/map/
```

复制完成后只改一个文件：

```yaml
# mas2027_nav_bringup/config/navigation_map.yaml
map_files:
  localization_pcd: pcd/<名称>.pcd
  occupancy_yaml: map/<名称>.yaml
  terrain_msgpack: map/<名称>_terrain.msgpack
```

相对路径以安装后的 `mas2027_nav_bringup` share 目录为基准，也可以填写绝对路径。随后重新执行
`colcon build --symlink-install --packages-select mas2027_nav_bringup`，让清单选中的 PCD、地图 YAML、
该 YAML 引用的 PGM 和 terrain msgpack 进入 `install/`，再启动导航。其他历史地图留在源码目录，
不会复制到新安装包；绝对路径指向仓库外时，部署机器需要自行提供该文件。`map_server` 会直接读取 `occupancy_yaml` 中的
`origin: [x, y, yaw]` 和 `resolution`，不再接受手填 `origin_x/origin_y`；同时核对该 YAML 引用的 PGM
与 terrain msgpack 的宽、高、分辨率，防止误配不同批次地图。当前地形查询只支持 `yaw: 0`，控制台
导出时应先把 PCD、PGM 和 terrain 成组旋转到最终 map 坐标系。

## 快速检查

```bash
ros2 topic hz /Odometry && ros2 topic hz /cloud_registered
ros2 topic echo /cost_map --once && ros2 topic echo /opt_path --once
ros2 topic echo /cmd_vel --once
ros2 run tf2_ros tf2_echo map odom && ros2 run tf2_ros tf2_echo odom base_link
```

点目标后没有轨迹：依次确认 ROGMap 已收到点云与里程计、目标在当前滑窗可达范围内、`/opt_path`
有发布；MPC 输入过期、TF 失败或求解失败时执行器发零速。实车复跑要保留地图与轨迹证据时，先按
按[现场记录与调试](docs/field_debugging.md#现场录包)启动记录器；[在线导航说明](docs/online_navigation.md)解释无先验模式。修改记录见
[`docs/change-history.md`](docs/change-history.md)。
