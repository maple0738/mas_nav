# mas2027_nav_executor：导航执行链路总览

本文讲的是本包当前代码如何把一个导航目标变成底盘速度与模式请求。它不是 Nav2 的 `controller_server` 插件：入口是独立的 `mas2027_nav_executor_node`，内部组合静态地形、在线 ROGMap、全局搜索、MINCO 轨迹、任务状态机、区域控制和 MPC。地图发布、定位和硬件通信分别由仓库中的其他包承担。

## 一眼看懂数据流

```text
map_server ── /cost_map、/terrain_label_map ──┐
点云/里程计 ── ROGMap（在线局部障碍、ESDF） ────────────────┤
RViz/Foxglove ── /goal_pose ────────────────────────────────┤
                                                          ▼
                目标接纳 → TaskManager → 全局搜索
                                            ↓
                           局部路径/走廊 → MINCO → /opt_path
                                                      ↓
                            最终轨迹区域标注 → RegionController
                                                      ↓
/Odometry ───────────────────────────────→ PathExecutor / MPC
                                                      ↓
                                      指令安全门 + 零速降级
                                                      ↓
                        /cmd_vel + /nav_executor/chassis_cmd
                                                      ↓
                                    nav_serial_bridge → CDC1 → 下位机
```

可以把它分成两个持续运行的循环：`PathPlanner` 中的 `TaskManager` 每 50 ms 检查目标与重规划；顶层节点以 `node.control_rate_hz`（默认 20 Hz）执行一次控制和发布。节点使用 `MultiThreadedExecutor` 同时管理顶层 `nav_executor` 和内部的生命周期节点 `nav_executor_planner`；后者承载 ROGMap、MINCO 和任务定时器。两个循环不是“规划一次、执行到底”：局部轨迹会按安全性、有效期和强制重规划周期更新。

## 地图、坐标系与各自职责

| 输入/组件 | 主要用途 | 注意点 |
|---|---|---|
| `/cost_map`、`/terrain_label_map` → `TerrainGrid` | 静态可通行性、特殊区域类别 | 两者组成一致的地形快照；静态语义地图通常在 `map` 系。 |
| `/rog_map/terrain_label` | 在线坡道/隧道语义 | 有效在线 `5/6` 优先用于区域 mode；未知、普通或过期时采用静态标签。 |
| `ROGMapROS` | 在线点云障碍、局部距离与梯度，供局部优化和安全检查 | 在 `odom` 系；目标接纳和命令安全门均要求其快照足够新。 |
| `/Odometry` 与 TF | 机器人位姿、速度，以及地图/规划/执行坐标转换 | `planner.frames.rog_frame`、ROGMap frame、`node.frames.odom` 必须一致。 |
| `/goal_pose` | 目标位姿 | 入口会检查目标有效性、地图和里程计准备状态，并转换到规划坐标系。 |

`TerrainGrid` 根据静态代价图判断栅格是否可通行；区域 mode 沿最终轨迹优先读取在线坡道/隧道识别，其他位置回退静态标签，详见[区域控制说明](docs/region_control.md)。在线 ROGMap 同时处理当前局部障碍。

## 一个目标的完整生命周期

1. `NavExecutorNode::accept_goal()` 收到 `/goal_pose`，检查坐标有效性；`PathPlanner::acceptGoal()` 再检查地形快照、ROGMap 是否存在且新鲜、TF、目标静态可通行、里程计新鲜以及机器人当前位置的在线地图状态。不满足时拒收目标，不直接向底盘发运动指令。
2. `TaskManager::submitGoal()` 暂停旧目标的运动许可；新目标优先。任务状态包括 `IDLE`、`PLANNING`、`FOLLOWING`、`RECOVERING`。同一目标且旧轨迹仍安全时可继续跟随，否则重新进行全局/局部规划。
3. `MincoPlanner::PlanGlobalPath()` 通过全局搜索器在静态地形上找路线；搜索实现可按配置选择，当前配置的 `planner_mode` 为 `EXPLORATION`。`/nav_executor/global_plan` 显示这条全局折线，**不是**实际跟踪的平滑轨迹。
4. `MincoPlanner::ReplanLocal()` 从全局路线取得局部引导路径，处理稀疏点、生成安全走廊、分配速度，再做 MINCO 优化并检查轨迹安全性。成功后发布 `/opt_path`（`interfaces/MpcPositionCommand`）。`/nav_executor/minco_path` 和调试 Marker 用于查看优化结果。
5. 顶层节点在收到 `/opt_path` 时，沿**最终轨迹**采样地形 label，形成 `RegionPlan`。运行时 `RegionController` 用当前位置在路径上的进度决定区域阶段、请求的 mode 和速度/加速度上限；普通路段也走同一控制循环。
6. `PathExecutor::computeCommand()` 检查运动许可、轨迹/里程计时效，把轨迹转为执行坐标系，构造 MPC 参考并求解；区域 profile 收紧平移速度和加速度约束。随后 `checkCommandSafety()` 用静态地形规则与在线 ROGMap 检查下一段指令。
7. 顶层节点每周期发布 `/cmd_vel`，并把同周期的 `vx`、`vy`、`w`、`mode` 打包为 `/nav_executor/chassis_cmd`。`nav_serial_bridge` 消费后者，经 CDC1 直接发送 `vx`、`vy`、`w`、`nav_state` 给下位机；当前串口协议不含 mode。`/cmd_vel` 仍可用于观察。

跟随期间，`TaskManager` 会检查当前轨迹是否安全、是否过期以及是否到了局部重规划周期。路径变得不安全时先撤销运动许可并发布紧急停止轨迹，再重新规划；局部重规划失败但旧轨迹仍有效时可以继续使用旧轨迹。目标到达且平移速度足够低后回到 `IDLE`。贴近障碍且重规划失败时，代码中还有受限逃逸 `RECOVERING` 分支；它不是常态导航路径。

## 代码从哪里读

| 位置 | 职责 |
|---|---|
| [`src/nav_executor_node.cpp`](src/nav_executor_node.cpp) | ROS 入口、参数与订阅发布、轨迹区域标注、20 Hz 控制、可视化。 |
| [`src/path_planner/path_planner.cpp`](src/path_planner/path_planner.cpp) | 内部规划节点、ROGMap/MINCO 初始化和目标接纳门。 |
| [`src/task_manager/task_manager.cpp`](src/task_manager/task_manager.cpp) | 目标状态机、全局/局部规划调度、重规划与恢复。 |
| [`src/path_planner/search/`](src/path_planner/search/) | 全局 A*/SMAC 搜索及静态地形查询适配。 |
| [`src/path_planner/trajectory/`](src/path_planner/trajectory/) | 局部路径、走廊、速度分配、MINCO 优化与轨迹安全检查。 |
| [`src/common/environment/`](src/common/environment/) | `TerrainGrid` 地形快照、地图查询适配、区域段标注及 `RegionController`。 |
| [`src/path_executor/path_executor.cpp`](src/path_executor/path_executor.cpp) | 轨迹参考、MPC 调用、速度输出与状态归因。 |
| [`src/path_executor/monitoring/command_safety.cpp`](src/path_executor/monitoring/command_safety.cpp) | 命令发送前的静态地形与在线障碍安全门。 |
| [`src/path_executor/mpc/`](src/path_executor/mpc/) | MPC 求解器与约束配置。 |

`include/mas2027_nav_executor/` 下是这些组件的接口；`vendor/` 是随包构建的 MINCO、MPC 相关外部实现，理解主流程可先跳过。

## 常用话题与观察点

以下是当前 `config/node_params.yaml` 的默认名称，实际可被启动参数覆盖。

| 话题 | 方向 | 作用 |
|---|---|---|
| `/goal_pose` | 输入 | 新导航目标。 |
| `/Odometry` | 输入 | 规划和控制使用的里程计。 |
| `/cost_map`、`/terrain_label_map` | 输入 | 静态占据与区域标签。 |
| `/rog_map/terrain_label` | 输入 | 在线坡道和隧道标签，优先于静态特殊区域标注。 |
| `/opt_path` | 内部规划 → 执行 | MPC 使用的最终局部轨迹。 |
| `/nav_executor/global_plan` | 输出 | 全局搜索折线。 |
| `/nav_executor/minco_path` | 输出 | MINCO 局部轨迹可视化。 |
| `/planning_constraints` | 输出 | RViz 中的静态规划约束栅格。 |
| `/cmd_vel` | 输出 | 当前最终速度；`angular.z` 始终为 +4 rad/s，线速度按导航许可和安全检查输出。 |
| `/nav_executor/chassis_cmd` | 输出至 `nav_serial_bridge` | 同一条消息里的 `vx`、`vy`、`w`、`mode`；串口发送 `vx`、`vy`、`w`、`nav_state`。 |
| `/nav_executor/region_status` | 诊断输出 | 请求的 label、mode、阶段、轨迹进度与速度上限；不是底盘确认回执。 |

## 区域控制在总链路中的位置

本车关注上坡 `label=mode=5`、隧道 `label=mode=6`、起伏路 `label=mode=7`。三类区域各有最大速度、最大加速度以及进入前准备/激活/提交、离开后释放距离。MINCO 先按区域调整路径速度；最终轨迹标注后，执行器按路径进度切换内部模式并平滑过渡 MPC 约束。当前 mode 不发给下位机，`w` 始终为 +4 rad/s；按坡面或隧道方向进行底盘对正的上位机控制尚未接入。

完整的阶段、label、协议边界和参数说明见[区域控制说明](docs/region_control.md)。当前配置为普通路段 `mode=4`、坡道 `5`、隧道 `6`、起伏路 `7`。当前 CDC1 接收协议没有 mode 字段，因此底盘不会随导航的区域状态切换模式；`/nav_executor/region_status` 只表示导航内部状态。

## 线速度停车、安全与排障

`PathExecutor` 在无运动许可、缺轨迹/里程计、数据过期、参考无法转换、MPC 求解失败或指令安全门拒绝时返回默认零速度。安全门会检查地形与 TF、ROGMap 时效，以及局部障碍净空；顶层控制循环仍按周期发布命令，避免只发一次停机。顶层将最终 `w` 固定为 +4 rad/s，因此这些情形只令 `vx/vy=0`；自动模式下底盘执行收到的 `w`，不应另加自转。

排查时建议按数据流顺序看：目标是否被接纳 → `/nav_executor/global_plan` 是否出现 → `/opt_path` 是否持续更新 → `/Odometry`、ROGMap 与 TF 是否新鲜 → `/nav_executor/chassis_cmd` 的速度 → `nav_serial_bridge` 的连接和收发日志 → 下位机 `minipc_nav` 在线状态。底盘 mode 当前不经 CDC1 发送。

## 配置与启动

| 文件 | 主要配置 |
|---|---|
| [`config/node_params.yaml`](config/node_params.yaml) | 话题、控制频率、轨迹/里程计/ROGMap 超时、输出坐标系与可视化。 |
| [`config/planner_params.yaml`](config/planner_params.yaml) | 全局搜索、ROGMap、局部路径、MINCO 优化与重规划参数。 |
| [`config/mpc_params.yaml`](config/mpc_params.yaml) | MPC 步长、预测时域、速度/加速度/角速度边界与权重。 |
| [`config/region_control.yaml`](config/region_control.yaml) | 三类区域的 mode、速度/加速度及路径阶段距离。 |

整套导航的启动文件不在本包，而在 `mas2027_nav_bringup/launch/nav_executor_launch.py`。它加载本包 `config/*.yaml`，并启动地图、定位相关组件；默认也启动 `nav_serial_bridge`。仅需观察导航而不向硬件发送时，使用 `use_nav_serial:=False`。当前 CDC1 协议与区域控制的关系见区域控制说明。


### 无先验地图模式

在 `config/node_params.yaml` 中将 `node.map_source` 设为 `online`，然后使用
`ros2 launch mas2027_nav_bringup nav_executor_launch.py`。整套启动模式仅由该 YAML 选择，切换需要重启。
该模式不创建静态 `TerrainGrid`，目标接收、局部搜索、MINCO 和 MPC 直接使用实时 ROGMap；
在线图新鲜度、轨迹与指令净空检查继续生效。远目标通过已观测自由区中的子目标分段推进，
约每秒刷新路径；原始任务目标始终保留。在线坡道/隧道标签可切换 mode，没有有效标签时采用普通 mode。
默认启用 B1 累计地图：`planner.online_global_map.*` 控制分辨率、更新频率、证据和容量，
`/nav_executor/online_global_map` 发布本次启动内的历史栅格，`PathPlanner::globalMapQuery()` 提供固定版本查询。
当前任务仍采用局部窗口搜索；累计地图的任务路线接入属于后续 B2。
将 YAML 中的 `node.map_source` 改为 `prior` 可切回先验模式，该模式要求完整静态地图并使用融合查询。详见 [无先验地图导航](../docs/online_navigation.md)。
