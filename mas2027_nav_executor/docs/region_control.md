# 区域控制：隧道、坡道、起伏路

本文解释当前导航侧的区域控制实现。导航内部沿规划轨迹切换区域速度参数和 `mode` 状态；CDC1 串口下发 `vx`、`vy`、`small_yaw_ref` 和 `follow_gimbal`，不下发角速度、导航状态或 `mode`。下位机在自动模式且导航在线、速度有效时固定 +4 rad/s 自转；特殊地形的底盘朝向闭环尚未接入相对角度 `δ`。

## 1. 数据流

```text
terrain msgpack → map_server: /cost_map + /terrain_label_map
实时点云 → ROGMap: /rog_map/terrain_label
  → 区域标签选择（在线坡道/隧道优先，其他格子回退静态标注）
  → TerrainGrid 快照与在线语义图
  → 全局搜索 / MINCO 速度规划
  → /opt_path（最终轨迹）
  → annotateRegions()（沿轨迹识别区域）
  → RegionPlan / RegionSegment
  → RegionController（按路径进度切换阶段与 mode）
  → PathExecutor（区域速度、加速度约束下的 MPC）
  → /nav_executor/chassis_cmd（同一消息内的 vx、vy、mode、small_yaw_ref、follow_gimbal）
  → nav_serial_bridge → CDC1 导航专用串口 → 下位机
```

`/cmd_vel` 仍发布，供现有工具观察速度；`nav_serial_bridge` 实际消费的是 `/nav_executor/chassis_cmd`，只取 `vx`、`vy`、`small_yaw_ref` 和 `follow_gimbal`。区域控制的状态另发到 `/nav_executor/region_status`，但其中的 mode 不出串口。

## 2. 地图如何表示区域

| 原始 label | 名称 | 本车用途 |
|---:|---|---|
| 0 | `FLAT` | 普通地面 |
| 1 | `OBSTACLE` | 障碍 |
| 5 | `SLOPE` | 上坡速度约束；导航内部 mode 5 |
| 6 | `TUNNEL` | 隧道速度约束；导航内部 mode 6 |
| 7 | `UNDULATING` | 起伏路速度约束；导航内部 mode 7 |

有效标签仅有表中五个值；2～4 和 8 不再接受。terrain msgpack 只保存标签通道；`map_server` 从中生成 `mono8` 的 `/terrain_label_map`。静态占据图与在线 ROGMap 仍负责障碍安全判断。

区域控制订阅 ROGMap 的 `/rog_map/terrain_label`。新鲜的在线 `5`（坡道）或 `6`（隧道）优先于地图标注；在线 `0`、`-1`、滑窗以外、坐标变换失败或超过 `node.online_terrain_label_timeout_s`（默认 0.8 秒）时使用静态标签。在线语义图改变执行侧区域 mode 和 MPC 速度约束，不改变静态占据图、MINCO 生成轨迹时依据静态标签的速度规划，或 ROGMap 的碰撞安全门。在线图以 5 Hz 发布，轨迹标注会随新图刷新；同一轨迹刷新时保留已行进的路径进度。若在线标签造成不同 mode 控制窗重叠，prior 模式回退整条轨迹的静态标注；online 模式回退到无特殊区段的普通 profile，不再因语义冲突挂起整条轨迹。识别阈值仍需实车点云标定，细节见 ROGMap README。

当前 terrain 地图已将一块平地测试区标为 `SLOPE=5`，供 mode 切换联调；地图里没有对应 label 的其他区域不会产生特殊区域段。

涉及文件：`map_server/include/map_server/utils.hpp`、`map_server/src/utils.cpp`、`map_server/src/map_server_node.cpp`、`common/environment/terrain_grid.*`。

## 3. 为什么要沿最终轨迹标注

全局搜索路径与 MINCO 优化后的轨迹可能不完全重合。因此，导航节点收到 `/opt_path` 后，`annotateRegions()` 才沿最终轨迹采样地图。采样间距为半个地图栅格；连续命中同类区域的样本组成一个 `RegionSegment`。轨迹 ID、轨迹位置、累计弧长和区域段保存在同一个 `RegionPlan` 中，控制周期从同一份计划读取。

每个区域段记录以下路径弧长，单位是米：

```text
普通路径       预备         请求 mode       区域本体          释放
──────────────|──────────────|──────────────|██████████████|────────|──
           prepare_s       active_s      enter_s       exit_s  release_s
                                   commit_s ↑
```

- `prepare_s = enter_s - prepare_distance`：开始将 MPC 的速度、加速度约束向区域目标值过渡，同时请求小 yaw 对齐。
- `active_s = enter_s - activation_distance`：导航内部开始请求该区域的 mode；当前不经串口发送。
- `commit_s = enter_s - commit_distance`：到这里较低的区域上限会被钳到目标值；坡道上限高于普通路段时仍按混合速率逐步提升。它仅是导航内部的路径阶段标记，不代表底盘已切换。
- `enter_s` / `exit_s`：轨迹进入、离开区域本体的位置。
- `release_s = exit_s + release_distance`：越过后才恢复普通 mode，给车体离开区域留余量。

各距离满足 `prepare_distance >= activation_distance >= commit_distance >= 0`。轨迹结束时如果仍在区域内，该段的出口与释放位置保持为无穷大，避免因为轨迹到头就恢复自转。同 mode 的重叠控制窗会合并；不同 mode 的控制窗重叠会拒绝整条轨迹，防止两个区域同时争夺底盘模式。

实现文件：`src/common/environment/region_control.cpp` 中的 `annotateRegions()`。

## 4. 执行时如何确定当前模式

`RegionController` 把机器人当前位置投影到当前轨迹，求得累计弧长 `path_progress`。进度只允许前进，并限制每周期可前进的距离，减少定位抖动或轨迹交叉造成的段跳变。

| 阶段 | 进度范围 | 输出与约束 |
|---|---|---|
| `NORMAL` | 未进入准备窗 | 普通 mode，普通速度参数 |
| `PREPARING` | `prepare_s ≤ s < active_s` | 普通 mode；较低的区域上限开始过渡，较高的坡道上限暂受普通 mode 约束 |
| `ARMED` | `active_s ≤ s < commit_s` | 请求区域 mode，继续过渡速度参数 |
| `COMMITTED` | `commit_s ≤ s < enter_s` | 保持区域 mode；较低上限钳到目标值，较高上限继续平滑提升 |
| `INSIDE` | `enter_s ≤ s < exit_s` | 保持区域 mode 和区域速度参数 |
| `RELEASING` | `exit_s ≤ s < release_s` | 继续保持区域 mode；释放后恢复普通模式 |

进入一个区域段后，控制器会锁定该段直到释放，避免边界附近反复切换。收到新轨迹时会重新标注并绑定新轨迹；如果标注失败或轨迹失效，导航停止发非零速度，先前请求的特殊 mode 会被保留。有效的新轨迹如果不含该区域段，则会按新计划恢复普通 mode。

`/nav_executor/region_status` 包含轨迹 ID、label、mode、阶段、当前进度和速度上限。它仅表示导航内部状态；当前底盘既收不到 mode，也没有模式回执。

实现文件：`common/environment/region_control.*`、`src/nav_executor_node.cpp`。

## 5. 各 mode 的速度参数

`config/region_control.yaml` 为坡道、隧道、起伏路分别提供：

| 参数 | 含义 |
|---|---|
| `mode` | 导航内部区域模式编号；当前 CDC1 帧不发送它 |
| `max_speed` | 区域平移合速度上限，m/s |
| `max_acceleration` | 区域平移加速度上限，m/s² |
| `normal_max_speed`、`normal_max_acceleration` | 普通路段的平移速度、加速度上限 |
| `prepare_distance`、`activation_distance`、`commit_distance`、`release_distance` | 上述路径阶段的距离，m |
| `speed_blend_rate` | 最大速度上限每秒允许变化的量 |
| `acceleration_blend_rate` | 加速度上限每秒允许变化的量 |

MINCO 在稀疏路径上读取原始区域标签，为坡道影响窗提供更高的分段速度上限，并对隧道、起伏路按各自上限约束，再做速度传播和轨迹优化。最终轨迹进入执行器后，`PathExecutor` 还会按当前区域 profile 更新 MPC 的 `vx/vy`、`ax/ay` 约束，并对最终平移合速度做上限校验。进入坡道时，上限在导航内部请求坡道 mode 后按 `speed_blend_rate`、`acceleration_blend_rate` 逐步提高；坡道外立即受普通路段上限约束。当前串口发送平移速度和小 yaw 对齐请求，不发送底盘角速度或 mode；下位机自行生成固定自转，小 yaw ref 以角度制发送，跟随控制由下位机应用层处理。

当前映射为普通路段 `mode=4`、上坡 `label=mode=5`、隧道 `label=mode=6`、起伏路 `label=mode=7`，均为 `uint8`；特殊区域若配置了与标签不同的 mode，导航节点会拒绝启动。普通路段上限为 `3.0 m/s`、`4.0 m/s²`；坡道上限为 `4.0 m/s`、`5.0 m/s²`，隧道及起伏路仍为普通路段上限。MINCO 和 MPC 的总上限允许坡道值，普通路段由 `normal_max_speed`、`normal_max_acceleration` 约束；MINCO 按静态标签使用坡道速度窗口，执行器还可按在线标签切换 mode 和约束。坡道准备、请求 mode、提交距离分别为 0.3/0.2/0.1 m；较高上限仅在请求坡道 mode 后逐步生效。实际速度仍由轨迹曲率、障碍净空、控制器与底盘能力决定；阶段距离仍需实车标定。

## 6. base_link 轴线跟随轨迹，允许正反共线，最终只发送角度

`base_link` 的 +X 就是云台正方向。用户只要求该轴与轨迹共线，不要求车头朝轨迹前进方向，因此 +X 与 -X 都可作为对齐目标。`gimbal_follow.allow_reverse` 默认 true：先求 MINCO 当前前进切线相对 LIO 的误差，再在正向与反向中选转角较小的轴线。原需转 -118° 时可选反向轴线，只转 +62°；已经反向对齐时按对齐处理，不要求掉头 180°。初次选较近方向，随后使用 `reverse_switch_hysteresis_deg=10` 滞回：正向误差绝对值超过 100° 才换反向，反向状态下小于 80° 才换回正向。设 allow_reverse=false 恢复只正向轴线对齐。

方向选择同时用于对齐判定、原地 MPC 和整个预测窗。反向时各预测点 yaw 在 MINCO XY 速度方向上统一加 pi，yaw_rate 曲率前馈不变；不逐预测点重新选方向，避免预测窗内出现不必要的 180° 翻转。MINCO 的位置、世界系速度/加速度与原路线不变，车体系转换自然得到倒车负 vx。当前不新增居中规划。

PathExecutor 仍使用同一个 MPC 求解 vx/vy/omega。健康连续控制沿用上一拍 MPC 命令作为加速度差分约束起点，不再每个跟随周期都用滞后的实测速度重置；过期输入、参考/求解失败或安全门拒绝仍停车并锚回实测速度。原角速度 ±2 rad/s、角加速度 ±4 rad/s²、控制周期 0.05 s 保留，普通路段平移上限仍为 3 m/s。

### 特殊地形跟随

在 PREPARING、ARMED、COMMITTED、INSIDE、RELEASING 阶段持续请求 follow_gimbal=1；进度达到 release_s 后切回 NORMAL 并清除请求。所选轴线误差需在 5° 内持续 0.3 s 才放行平移，超过 8° 重新等待；等待时 MPC 位置参考固定当前点、平移前馈为零。无有效轨迹/许可但同一目标仍在规划/重试、已有上一有效角度时，保持 `follow_gimbal=1` 和最后 `small_yaw_ref`、平移清零；目标结束、输入过期或姿态无效时才清除。地形、TF、ROGMap 时效和净空安全门保留。

`force_follow_gimbal` 参数已删除。NORMAL 阶段 follow_gimbal=0、角度为 0，ROS w=+2 rad/s；串口不发送 w，下位机在导航在线且速度有效时自行生成固定 +4 rad/s 自转。

### 规划间隙保持对齐

规划器短暂失败时，若此前已有有效跟随角度，节点继续发送 `follow_gimbal=1`、保持最后 `small_yaw_ref`，同时把 `vx/vy` 清零；新轨迹到达后从保持角度继续积分。普通语义区域的间隙保持由 `gimbal_follow.hold_last_alignment`（默认 true）控制，需要同一目标仍有效且里程计姿态有效。几何窄道对齐一旦锁存，则跨越无轨迹空档与相邻目标切换继续保持角度和模式，直到里程计累计行驶至少 2 m 且当前位置与近前方恢复宽路；目标结束时若已满 2 m 也释放；里程计过期期间停止平移并保持最后角度，不把定位跳变计入行驶距离。控制日志中的 `Chassis alignment transition` 给出实际下发的 `follow`、锁存状态和累计里程。

### 角度输出和观察

`GimbalFollowAdapter` 仍做方向选择、稳定判断、积分和 ±180° 限幅，不跨界回绕。角度变化率不再只取 omega_MPC，而是：

```text
rate = clamp(omega_MPC * 180/pi
             + heading_error_gain_s * selected_axis_error_deg,
             -max_heading_rate_deg_s, +max_heading_rate_deg_s)
ref_angle_deg += rate * control_dt
```

`selected_axis_error_deg` 已包含正/反轴选择，符号只表示转到哪一边，不表示行驶方向。误差在 `heading_deadband_deg` 内时不加修正。这样即使安全门只拒绝平移、MPC 输出 omega 为零，云台仍会继续把 base_link 轴线转向轨迹；无有效轨迹但同一目标仍在重试时冻结并保持最后角度，目标结束、输入过期或姿态无效时才清零。通信字段仍叫 small_yaw_ref，单位 deg，相对下位机已有 auto_yaw_offset；现有下位机 target_rad = auto_yaw_offset + small_yaw_ref * DEGREE_2_RAD 不变。该角度不是底盘对齐回执。

CDC1 仍为 16 字节：0xB5 + float32 vx + float32 vy + float32 small_yaw_ref + uint8 follow_gimbal + CRC16；不发送 w、nav_state 或 mode。跟随时 ROS w 仅用于 MPC 角速度观测。串口桥仍为 50 Hz、300 ms 超时；启动、超时和退出前最后一帧清除跟随标志/角度并清零平移。

Foxglove 中 gimbal_heading_error_deg 表示 LIO 云台轴线到较近轨迹轴线的误差，符号只表示转到哪一侧，不代表前进/倒车；gimbal_heading_ready 表示该误差稳定。两者都不是底盘对齐回执。`gimbal_follow.allow_reverse=true` 允许正反轴对齐，`reverse_switch_hysteresis_deg=10` 防止方向抖动；关闭 `allow_reverse` 才要求正向轴线。`heading_error_gain_s`、`heading_deadband_deg`、`max_heading_rate_deg_s` 控制实测误差修正，修改后需重启导航。尚未通过实车验证云台和底盘的对齐精度。

## 7. 使用与限制

云台达妙 H7、底盘 DJI C，本次下位机不修改。用户现有切换基准、云台/底盘跟随控制及 12° 底盘跟随死区保留；缺少底盘相对角度和模式回传，LIO 切线方向对齐不能证明底盘也达到 5°，轴向、安装零点与实际跟随响应仍需实车核对。

当前 ESDF 软代价倾向远离墙面，并非特殊区域中线硬约束；用户确认本次不新增居中规划。MPC 跟踪现有 MINCO 路线，不能修正参考路线本身偏离特殊路段中线的问题。

两端包类型和偏移不变。需重启上位机导航、串口桥和 Foxglove bridge；未刷写、未做实车通过验证，朝向参数、提前距离和稳定时间尚需实车标定。
