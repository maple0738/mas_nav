# MAS 2027 原生导航

ROS 2 Jazzy，运行链路统一为独立 `nav_executor`：不依赖 Nav2 server、Behavior Tree、Costmap 插件、
`minco_planner` / `minco_controller` 插件包。唯一入口 `mas2027_nav_bringup/nav_executor_launch.py`。
本文件是链路、TF、话题与排障总说明；按日期的专题文档在 `docs/`。

## 组件职责

| 组件 | 职责（输入 → 输出） |
|---|---|
| `mid360_driver` | 双 MID360 驱动内融合到参考雷达系 → `/mid360_driver/lidar`、`/mid360_driver/imu` |
| `small_point_lio` | 融合点云 + IMU → `/Odometry`、`/cloud_registered` |
| `odom_localizer` | GICP 六自由度先验定位（`lab3.pcd`）→ `/tf_maintainer/map_to_odom` |
| `tf_maintainer` | 唯一动态 TF 发布者：`map→odom`、`odom→base_link` |
| `terrain_map_server` | 静态地形服务 → `/cost_map`、`/terrain_label_map` |
| `nav_executor` | TaskManager + ROGMap + 全局搜索 + MINCO + MPC + 区域控制 → `/opt_path`、`/cmd_vel`、`/nav_executor/chassis_cmd` |
| `ros2_comm`（可选） | 消费 `/nav_executor/chassis_cmd` → UDP 底盘协议 |
| `robot_state_publisher` | URDF → 车体 / 雷达外参 |

`ROGMap` 在 `nav_executor` 进程内（`nav_executor_planner`），无独立节点，参数在 `planner.rog_map.*`；
静态地形代价图是搜索、轨迹验收和 MPC 制动的规划输入；区域 mode 优先采用 ROGMap 在线坡道/隧道标签，缺失时采用静态标签图。没有 `Global/Local Costmap` 是预期行为。

## 在线链路

```text
MID360 ×2 → mid360_driver → small_point_lio → /Odometry + /cloud_registered
                     ├─→ odom_localizer → map→odom → tf_maintainer → odom→base_link
                     └─→ ROGMap（进程内：在线占据 / 距离场 / 坡道隧道语义）

lab3_terrain.msgpack → terrain_map_server → /cost_map + /terrain_label_map
                                         → 全局搜索 → MINCO → MPC → 区域控制 → /nav_executor/chassis_cmd
ROGMap → /rog_map/terrain_label ──────────────────────────↑
```

- 全局主搜索是移植的 **SMAC 2D**（`path_planner/search/smac/`）：在静态地形栅格上做
  8 邻域 A\*，用同图距离场做 ESDF 软代价（`smac_2d.*`）。输出是纯几何折线，时间
  参数化与运动学可行性交给弧长速度剖面 + MINCO；`planner.use_smac: false` 退回 Astar，SMAC 失败
  **不自动回退**。
- MINCO 的走廊、优化与碰撞检查用 ROGMap 在线占据/距离场。全局折线进局部滑窗后逐段验净空：
  被截断则窗内重搜接回终点，完全封死则只发障碍前停车前缀（零末速度）。地图、距离场或
  `map→odom` 缺失时不接受目标、不发运动指令。
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

发布前校验、20 Hz 运行时监视、MPC 指令检查共用 `common/environment/clearance_gate.hpp` 的一条判据：

```
required(v) = collision_dist + max(v * replan_react_time, monitor_margin)
```

当前 `collision_dist: 0.28`、`monitor_margin: 0`、`replan_react_time: 0`，`node.rog_map_clearance`
也是 0.28（两处必须一致）。三条关键规则：

- **两道门同公式**：发布门 0.30 / 监视门 0.50 曾导致轨迹一发就被判不安全 → 急停 + 重规划、
  `/cmd_vel` 只在瞬间有值。监视门另减 `kMonitorClearanceTolerance`（0.05 m）防逐帧抖动误杀。
- **近场放宽**：起点弧长 `collision_dist` 以内只要求「不比当前实测净空更差」（留 0.02 m），
  否则贴墙停车后发目标点车不动。日志 `Near-field exemption active: ... X ... Y ... Z` 中
  X 是实测净空、Y 是下调后要求、Z 是完整要求，`X > Z` 属正常。
- **第四层兜底**：正常路径 → 窗内绕行 → 完整停车前缀（安全段需 0.45 m）都失败时，发一个不差于
  当前净空、末速为零的短前缀脱困；`planner.minco_optimizer.stuck_escape.enable: false` 可关闭。

失败原因按 `failure_log_first_n`(10) 逐条、每 `failure_log_every_n`(25) 次采样、每
`failure_summary_every`(50) 次汇总。`LOCAL_SEED_INVALID` / `..._AFTER_REPAIR` = 没进优化器；
`OPTIMIZER_FAILED` = 不收敛；`COLLISION` / `TERRAIN_COLLISION` = 硬校验被否。
四层兜底全败时 `Live obstacle blocks the local route...` 会带 `*_reject=` 现场与自动判读 `verdict=`：
`GEOMETRY` 连近场规则都过不了（拒绝正确，别动阈值）、`SEED_GATE_STRICTER` 只被更严的种子门否掉
（判据不一致）、`PREFIX_TOO_SHORT` 卡安全段长度、`TERRAIN` 死在 terrain/动态层、`NONE` 未捕获。

反复急停 = 两道门又不一致；只有 `not published` = 近场外净空不够，先查配准与 ROGMap 是否把车体
自身算成障碍，**不要直接减小 `collision_dist`**。细节见 `docs/refusal_triage_2026-09-16.md`。

## ROGMap 与 RViz

- ROGMap 订阅 `/cloud_registered` + `/Odometry`；RViz `ROGMAP` 分组默认显示紫色 `Occupied`（0.05 m）
  与 `2D Distance Field`。先验图融合关闭，`Static Layer Value` 不代表地形图；投影高度、地面/墙体
  阈值与 `node.rog_map_clearance` 需在静止实车上标定。已移除目标跟踪与未来预测（防振动误判）。
- MPC 仍为全向模型（`linear.x/y`、`angular.z`，±2 rad/s、±4 rad/s²，需按底盘标定）。
  区域控制沿最终轨迹标注坡道、隧道和起伏路，以 `/nav_executor/chassis_cmd` 同步下发速度与
  mode；`/cmd_vel` 保留供观测。`ros2_comm` 默认继续发兼容 mas_vision 的 9 字节
  `vx`/`vy`/`nav_state`，不会转发 `angular.z` 或新 mode。模式透传需要按
  [区域控制集成说明](docs/region_control_2026-09-24.md) 选定传输方式。
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
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 4
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

常用启动参数：`use_rviz`、`use_odom_localizer`、`use_ros2_comm`、`output_topic`。
启动模式仅由 `mas2027_nav_executor/config/node_params.yaml` 中的 `node.map_source` 选择，修改后重启生效。
设为 `prior` 时，先验地图文件统一在 `mas2027_nav_bringup/config/navigation_map.yaml` 中选择。
设为 `online` 时使用无先验模式：不读取地图文件、不启动 GICP 定位器和静态地图服务，
使用 `odom` 下的 LIO 与在线 ROGMap 分段推进；速度与加速度采用规划器、MPC 和区域控制 YAML，未知区域作为障碍。
具体启动、目标坐标和能力范围见 [无先验地图导航](docs/online_navigation.md)。
`use_ros2_comm` 默认 `True`——它消费 `/nav_executor/chassis_cmd`，不开会「指令一直有值但车不动」；
只想看导航时置 `False` 并确认底盘与急停状态。RViz 的 2D Goal Pose 发到 `/goal_pose`。

## 核心配置

- `mas2027_nav_executor/config/node_params.yaml`：频率、话题、坐标系、`node.rog_map_clearance`
  与 `node.rog_map_timeout_s`。
- `mas2027_nav_executor/config/planner_params.yaml`：全局搜索（`use_smac`、`smac_2d.*`、`tolerance`）、
  MINCO 与恢复参数、`planner.rog_map.*`。
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
二维动态格融合和对应可视化；ROGMap 在在线快照、局部种子路径、轨迹发布/20 Hz
监视与 MPC 指令前视中仍保持 fail closed。`node.rog_map_timeout_s` 直接检查进程内 ROGMap
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
`colcon build --symlink-install --packages-select mas2027_nav_bringup`，让新增地图文件与配置进入
`install/`，再启动导航。`map_server` 会直接读取 `occupancy_yaml` 中的
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
有发布；MPC 输入过期、TF 失败或求解失败时执行器发零速。修改记录见
[`docs/change-history.md`](docs/change-history.md)。
