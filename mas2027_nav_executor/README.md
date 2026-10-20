# mas2027_nav_executor：导航执行链路总览

本文讲的是本包当前代码如何把一个导航目标变成底盘速度与模式请求。它不是 Nav2 的 `controller_server` 插件：入口是独立的 `mas2027_nav_executor_node`，内部组合静态地形、在线 ROGMap、SMAC/Astar 全局搜索和 MINCO 轨迹、任务状态机、区域控制和 MPC。地图发布、定位和硬件通信分别由仓库中的其他包承担。

## 一眼看懂数据流

```text
map_server ── /cost_map、/terrain_label_map ──┐
点云/里程计 ── ROGMap（在线局部障碍、ESDF） ────────────────┤
RViz/Foxglove ── /goal_pose ────────────────────────────────┤
                                                          ▼
                目标接纳 → TaskManager → 全局搜索
                                            ↓
                     局部路径/走廊 → MINCO 轨迹 → /opt_path
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

可以把它分成两个持续运行的循环：`PathPlanner` 中的 `TaskManager` 每 50 ms 检查目标与重规划；顶层节点以 `node.control_rate_hz`（默认 20 Hz）执行一次控制和发布。节点使用 `MultiThreadedExecutor` 同时管理顶层 `nav_executor` 和内部的生命周期节点 `nav_executor_planner`；后者承载 ROGMap、SMAC/MINCO 规划和任务定时器。两个循环不是“规划一次、执行到底”：局部轨迹按有效期和强制重规划周期更新，安全性由执行前指令门逐周期检查。

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
2. `TaskManager::submitGoal()` 暂停旧目标的运动许可；新目标优先。任务状态包括 `IDLE`、`PLANNING`、`FOLLOWING`、`RECOVERING`。同一目标且仍处于 `FOLLOWING` 时可继续跟随，否则重新进行全局/局部规划。
3. `MincoPlanner::PlanGlobalPath()` 通过全局搜索器在静态地形上找路线；搜索实现可按配置选择，当前配置的 `planner_mode` 为 `EXPLORATION`。目标格被占据或净空不足时，搜索器会沿 `start → goal` 反方向最多回退 `minco_optimizer.goal_approach_max_shorten`（默认 1.5 m），取一个可通行格作为接近点；直线方向找不到时，再在目标周围找最近的、位于机器人一侧的可通行格作为排出点。起点格代价可通行但搜索净空不足时，会在 0.6 m 内找一个同时满足代价和 `smac_2d.minimum_clearance` 的邻近搜索起点。主搜索在 0.27 m 下无解且窄道续行开启时，可用 `smac_2d.relaxed_minimum_clearance`（当前 0.24 m）再做一次搜索；这只提供参考折线，执行侧仍按轨迹门和窄道地板验收。若目标格已确认不可通行，且 `goal_approach_max_shorten` 内找不到安全接近/排出点，则目标被标记为终止性无效：`TaskManager` 立即停车并清空旧路径，不再保留旧轨迹朝目标方向探索。若找到了接近/排出点，任务层会以该安全端点作为本目标的合法终点，到达后结束目标，不再追逐障碍内的原始坐标；原始目标只用于判断是否已经接近到可接受的终点。在线模式若新搜索路线突然反向且长度没有显著缩短，会先按实时地图逐段检查已提交路线的剩余段；仍满足窄道地板时保持原路线，失效或新路线明显更短时立即切换。局部轨迹仍从实测位姿开始。`/nav_executor/global_plan` 显示这条全局折线，**不是**实际跟踪的平滑轨迹。
4. `MincoPlanner::ReplanLocal()` 从全局路线取得局部引导路径，处理稀疏点、生成安全走廊、分配速度，再做 MINCO 优化并检查轨迹安全性。默认启用两次 MINCO：PRE 阶段用段平均时间 `±10%` 的软约束先稳定轨迹形状，FINELY 阶段以 PRE 结果为初值，并去掉位置梯度的沿轨迹方向分量；第二阶段失败时回退 PRE 结果，仍要过完整安全门。两次优化不替代冷/热启动：热启动时仍复用旧轨迹剩余段最近 `minco_optimizer.replan_continuity_time_s`（默认 0.3 s）作为初值前缀，且拼接跳跃超过 0.6 m 会自动缩短/放弃前缀，避免旧路线拖住新规划；若该初值优化失败，则用当前路径初值再试一次，恢复的候选仍要通过完整轨迹安全校验。稀疏折线在窄道开启时要求跳段保留不低于 `narrow_passage_clearance` 的净空，避免切角。成功后发布 `/opt_path`（`interfaces/MpcPositionCommand`）。`/nav_executor/minco_path` 和调试 Marker 用于查看优化结果。窄道续行开启时，当前实测净空在 `[narrow_passage_clearance, 有效阈值)` 内，或参考轨迹在前方 1.5 m 内进入该区间，都会按该下限继续；实测位置进入窄带后平移速度限于 `narrow_passage_speed`，入口前则按 `clearance_braking` 的制动能力和反应时间计算速度包络，到窄口降至该速度，制动参数无效时直接使用该上限；轨迹采样点在距参考折线 0.20 m 内且对应折线位置落在任一安全窄点前后 0.50 m 内时放宽，使窄口前的同一接近路线也使用该地板；偏离走廊或远离窄段仍按完整 0.26 m 阈值，参考上的单个低净空异常点不取消其余安全窄段的放宽。稠密参考线本身低于 0.24 m 时，局部路径处理器先在实时栅格内按该地板重搜参考线，再交给 MINCO。当前实测净空低于 0.24 m 时不因“前方窄”放宽；净空比较带 1 mm 浮点容差。目标接近减速带 `minco_optimizer.goal_approach.*`（`deceleration`/`reaction_time`/`stop_margin`/`min_speed`，默认 1.2 m/s²、0.20 s、0.30 m、0.15 m/s）用实测底盘刹车能力给“剩余弧长 → 允许速度”封顶：种子速度上限、弧长速度剖面的反向刹车余量和执行器对 `/opt_path` 参考速度的封顶共用同一份包络（`v·τ + v²/(2a) ≤ 剩余弧长 − stop_margin`，下限 `min_speed` 爬行）。它不改变安全门，只让参考线不再贴着 4–5 m/s² 编刹车余量；`deceleration<=0` 关闭。当前正方形底盘默认启用几何窄道对齐；执行器在前方 `narrow_passage_alignment_lookahead`（默认 2.0 m）扫描首个低净空轨迹点，把该处轨迹切线作为底盘航向目标；普通窄道在进入前先停车等航向稳定；进入几何窄道模式后，按里程计累计行驶至少 `narrow_passage_alignment_hold_distance`（默认 2.0 m），之后当实测净空足以容纳 0.302 m 旋转外廓、5 cm 目标间隙及占据格半对角线，且前方 0.50 m 没有窄点时退出；目标到达时也须满足同一旋转净空才退出，未满则跨目标保持；短暂重规划间隙继续累计里程并保持对齐；语义坡道/隧道仍按区域准备阶段对齐。
5. 顶层节点在收到 `/opt_path` 时，沿**最终轨迹**采样地形 label，形成 `RegionPlan`。运行时 `RegionController` 用当前位置在路径上的进度决定区域阶段、请求的 mode 和速度/加速度上限；普通路段也走同一控制循环。
6. `PathExecutor::computeCommand()` 检查运动许可、轨迹/里程计时效，把轨迹转为执行坐标系，构造 MPC 参考并求解。当前 YAML 已开启 `mpc.secondary_reference.enable` 和角度分支、关闭窄道分支，尚未实车 A/B；`angle_enable` 控制“首轮预测速度与 MINCO 切线夹角超过 `angle_threshold_deg` 时按夹角余弦放慢参考时间推进”，`narrow_enable` 控制“已提交轨迹进入窄带时放慢参考推进”，两个分支可独立开关。二次求解前恢复首轮加速度锚点，失败时沿用首轮指令；参考速度前馈同步缩放，`min_time_scale` 是所有缩放的下限。横向/纵向位置权重由 `mpc.weights.along/cross` 按轨迹切线旋转；`mpc.weights.state[2]` 仍作 yaw 权重，state[0]/[1] 仅在 along/cross 未配置时作为旧配置回退。区域 profile 仍收紧平移速度和加速度约束；随后 `checkCommandSafety()` 用静态地形规则与在线 ROGMap 检查下一段指令。
7. 顶层节点每周期发布 `/cmd_vel`，并把同周期的 `vx`、`vy`、`mode`、`small_yaw_ref` 和 `follow_gimbal` 打包为 `/nav_executor/chassis_cmd`。特殊地形从准备阶段请求小 yaw 对齐入口路径方向，ref 为相对下位机 auto_yaw_offset 的遥控式位置指令（deg，逆时针为正），上位机 MPC 跟踪轨迹的前进切线，并把内部角速度前馈与 LIO 轴线误差修正积分为角度 ref；LIO 航向稳定前暂停平移。`nav_serial_bridge` 消费后者，经 CDC1 发送 `vx`、`vy`、ref 和请求标志给下位机；当前串口协议不含角速度、导航状态或 mode。`/cmd_vel` 仍可用于观察。

跟随期间，`TaskManager` 检查轨迹有效期和局部重规划周期；`PathExecutor::checkCommandSafety()` 每个周期检查当前指令的地形、ROGMap 时效和净空。局部重规划失败但旧轨迹未过期时可以继续使用旧轨迹；只有旧轨迹过期时才急停重规划。在线模式的全局折线只用于引导局部规划：全局刷新失败但已提交的局部轨迹未过期时，不再立即急停，而是保留局部轨迹并等下一个刷新周期重试。目标到达且平移速度足够低后回到 `IDLE`。贴近障碍且重规划失败时，代码中还有受限逃逸 `RECOVERING` 分支；它不是常态导航路径。

`20261018_130912` 的同一帧 ROGMap 距离场显示：起点到目标在 0.25 m 搜索门槛下连通，在 0.26 m 下已经断开，瓶颈读数约 0.25 m。因此把全局搜索门槛设成 0.30 m 必然在这张地图上拒绝路线。用户确认车体为正方形、中心到角点的旋转半径为 0.302 m，故对齐直行半宽为 0.302/√2≈0.214 m；地图 ESDF 量的是车体中心到占据格中心，不是旋转半径。按 5 cm 轴向栅格且车体完全对齐估算，0.25 m 格心净空扣除半格 0.025 m 和车体半宽后，只剩约 0.011 m，定位和航向误差会继续消耗这点间隙。URDF 目前的 0.5×0.5 m 方框对应约 0.354 m 角点半径，与实测值不一致，不能用它代替实物外廓。前方净空低于旋转半径+目标间隙+半格对角线时提前进入几何对齐；到达目标或离开窄道时也要求满足这项旋转净空才退出。ROGMap 按实测半径推得的方形轮廓过滤自车回波并保留已观测墙体。

当前 ROGMap 的 ESDF 未预膨胀，净空是底盘**中心**到障碍物的距离。正方形底盘的旋转外接圆半径取 `corridor.swept_radius=0.302 m`；走廊生成用 `corridor.robot_radius=0.28 m`。本轮按用户指定的历史撞墙配置，将普通有效硬阈值回退到 0.26 m、窄道地板回退到 0.24 m，分别比旋转外廓半径小 4.2 cm 和 6.2 cm。配置显式开启 `planner.corridor.allow_undersized_clearance`，节点启动时打印两者与半径的差值；关闭此开关会恢复“至少外廓半径加 5 cm”的启动校验。`minco_optimizer.clearance_braking` 继续按朝障碍方向的速度追加反应距离和刹车距离（1.2 m/s²、0.20 s），先封顶 MPC 参考和实际指令，再由短时指令门复核；平行墙面行驶不追加纵向制动距离。低于窄道地板时仅允许朝净空增大的方向脱离。日志 `Clearance braking cap` 用于定位现场限速；这组 ESDF 阈值不能保证车体不碰墙，需结合实车量测逐步上调。

在线目标若被实时滚动图与累计图同时确认占据，且目标周围完整的接近/排出搜索范围均已观测、没有安全端点，接收阶段会拒绝；规划中才确认无效时，`TaskManager` 进入 `GOAL_INVALID`，发布一次急停并等新目标，不再每 0.5 s 重试。只有实时图阻塞、累计图尚未确认时，停止旧轨迹并保留目标，每 0.5 s 重试；实时障碍消失后重新规划。窗口外或靠近边界的目标不按“无安全排出点”终止。累计图记为自由但实时图暂时占据的远端目标先走安全的临时子目标，接近后才允许搜索排出终点；局部路径仍必须通过实时图与碰撞门。

## 代码从哪里读

| 位置 | 职责 |
|---|---|
| [`src/nav_executor_node.cpp`](src/nav_executor_node.cpp) | ROS 入口、参数与订阅发布、轨迹区域标注、20 Hz 控制、可视化。 |
| [`src/path_planner/path_planner.cpp`](src/path_planner/path_planner.cpp) | 内部规划节点、ROGMap/SMAC/MINCO 初始化和目标接纳门。 |
| [`src/task_manager/task_manager.cpp`](src/task_manager/task_manager.cpp) | 目标状态机、全局/局部规划调度、重规划与恢复。 |
| [`src/path_planner/search/`](src/path_planner/search/) | SMAC/A* 搜索及静态地形查询适配。 |
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
| `/nav_executor/minco_path` | 输出 | 局部平滑轨迹可视化。 |
| `/planning_constraints` | 输出 | RViz 中的静态规划约束栅格。 |
| `/cmd_vel` | 输出 | 最终平移速度与观测用角速度；跟随时为 MPC omega，未跟随时为 +2 rad/s，串口均不发送 w。 |
| `/nav_executor/chassis_cmd` | 输出至 `nav_serial_bridge` | 同一条消息里的 `vx`、`vy`、`mode`、`small_yaw_ref`、`follow_gimbal`；串口发送平移速度、ref 和请求标志。 |
| `/nav_executor/region_status` | 诊断输出 | 请求的 label、mode、阶段、轨迹进度与速度上限；不是底盘确认回执。 |

导航只在特殊地形的 `PREPARING`、`ARMED`、`COMMITTED`、`INSIDE`、`RELEASING` 阶段请求跟随，释放后清除。`base_link` 的 +X 是云台正方向；`gimbal_follow.allow_reverse=true` 时选择较近的正向/反向轨迹轴线，只要求 +X 或 -X 与轨迹共线，符号不表示行驶方向。整个 MPC 预测窗保持同一轴线分支，MINCO 轨迹和世界系速度不变，倒车由车体系负 vx 自然实现。方向选择使用 10° 滞回。跟随角度指令在 MPC 角速度前馈上叠加实测 LIO 轴线误差修正（默认 1/s、0.5° 死区、总角速度上限 120°/s），所选轴线误差需在 5° 内稳定 0.3 s 才放行平移。失效输入、无轨迹/许可和安全门拒绝仍停车平移。健康 MPC 控制周期沿用上一拍命令的加速锚点，避免下位机响应延迟让每拍速度重回起步量。

`force_follow_gimbal` 参数已删除。`config/region_control.yaml` 中 `allow_reverse=false` 可恢复只正向轴线对齐；`heading_error_gain_s=0` 可恢复只按 MPC omega 积分；`hold_last_alignment=false` 可恢复“无有效轨迹就清除跟随并让下位机自由自转”的旧行为。修改需重启上位机导航，下位机及通信格式不变。

## 区域控制在总链路中的位置

本车关注上坡 `label=mode=5`、隧道 `label=mode=6`、起伏路 `label=mode=7`。三类区域各有最大速度、最大加速度以及进入前准备/激活/提交、离开后释放距离。MINCO 先按区域调整路径速度；最终轨迹标注后，执行器按路径进度切换内部模式并平滑过渡 MPC 约束。当前 mode 不发给下位机；跟随时 MPC 沿现有 MINCO 前进切线提供角速度前馈，叠加 LIO 实测轴线误差后积分为小 yaw 角度指令，由下位机现有控制环实现跟随。

完整的阶段、label、协议边界和参数说明见[区域控制说明](docs/region_control.md)。当前配置为普通路段 `mode=4`、坡道 `5`、隧道 `6`、起伏路 `7`。当前 CDC1 接收协议没有 mode 字段，因此底盘不会随导航的区域状态切换模式；`/nav_executor/region_status` 只表示导航内部状态。

## 线速度停车、安全与排障

`PathExecutor` 在无运动许可、缺轨迹/里程计、数据过期、参考无法转换、MPC 求解失败或指令安全门拒绝时返回默认零速度。安全门会检查地形与 TF、ROGMap 时效，以及局部障碍净空；顶层控制循环仍按周期发布命令，避免只发一次停机。普通路段最终 `w=+2 rad/s`；特殊地形跟随时 ROS `w` 表示 MPC 内部角速度，仅供观测；串口只发角度 ref，其变化率为 MPC 角速度前馈加 LIO 实测轴线误差修正，由下位机现有控制环执行。该 `w` 仅保留在 ROS 观测消息中。特殊地形等待 LIO 航向稳定时将 `vx/vy=0`。MPC/安全门刹停期间，如区域和输入仍有效则保持跟随请求；同一目标仍在规划/重试且已有上一有效角度时，节点保持 `follow_gimbal=1` 和最后 `small_yaw_ref`、平移清零，避免下位机回到自由自转；目标结束、输入过期或姿态无效时才清除。

几何窄道对齐锁存期间，以上“目标结束或输入过期清除”不适用：节点保持 `follow_gimbal=1` 和最后角度、停止平移，跨相邻目标及短暂无轨迹空档累计有效里程；至少行驶 2 m 后在当前位置与近前方恢复宽路时退出；目标完成且已满 2 m 时也退出。`Chassis alignment transition` 日志记录每次实际模式切换和累计距离。局部参考线会采样整条折线，端点合格而弦中间切入低净空时也尝试按窄道地板绕行。

排查时建议按数据流顺序看：目标是否被接纳 → `/nav_executor/global_plan` 是否出现 → `/opt_path` 是否持续更新 → `/Odometry`、ROGMap 与 TF 是否新鲜 → `/nav_executor/chassis_cmd` 的速度 → `nav_serial_bridge` 的连接和收发日志 → 下位机 `minipc_nav` 在线状态。底盘 mode 当前不经 CDC1 发送。

## 配置与启动

| 文件 | 主要配置 |
|---|---|
| [`config/node_params.yaml`](config/node_params.yaml) | 话题、控制频率、轨迹/里程计/ROGMap 超时、输出坐标系与可视化。 |
| [`config/planner_params.yaml`](config/planner_params.yaml) | 全局搜索、ROGMap、局部路径、MINCO 优化与重规划参数。 |
| [`config/mpc_params.yaml`](config/mpc_params.yaml) | MPC 步长、预测时域、速度/加速度/角速度边界、`weights.along/cross` 权重与二次参考采样参数。 |
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
