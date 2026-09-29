# 区域控制：隧道、坡道、起伏路

本文解释当前导航侧的区域控制实现。目标是让车沿规划轨迹接近特殊区域时，提前请求对应的底盘 `mode`，并使用该区域的速度参数；离开后恢复普通模式。底盘是否自转由下位机按 `mode` 决定。导航侧不向底盘传 `omega`。

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
  → /nav_executor/chassis_cmd（同一消息内的 vx、vy、mode）
  → ros2_comm → mas_vision → 下位机
```

`/cmd_vel` 仍发布，供现有工具观察速度；`ros2_comm` 实际消费的是 `/nav_executor/chassis_cmd`。区域控制的状态另发到 `/nav_executor/region_status`。

## 2. 地图如何表示区域

| 原始 label | 名称 | 本车用途 |
|---:|---|---|
| 0 | `FLAT` | 普通地面 |
| 1 | `OBSTACLE` | 障碍 |
| 5 | `SLOPE` | 上坡模式与限速；底盘 mode 5 |
| 6 | `TUNNEL` | 隧道模式与限速；底盘 mode 6 |
| 7 | `UNDULATING` | 起伏路模式与限速；底盘 mode 7 |

有效标签仅有表中五个值；2～4 和 8 不再接受。terrain msgpack 只保存标签通道；`map_server` 从中生成 `mono8` 的 `/terrain_label_map`。静态占据图与在线 ROGMap 仍负责障碍安全判断。

区域控制订阅 ROGMap 的 `/rog_map/terrain_label`。新鲜的在线 `5`（坡道）或 `6`（隧道）优先于地图标注；在线 `0`、`-1`、滑窗以外、坐标变换失败或超过 `node.online_terrain_label_timeout_s`（默认 0.8 秒）时使用静态标签。在线语义图改变执行侧区域 mode 和 MPC 速度约束，不改变静态占据图、MINCO 生成轨迹时依据静态标签的速度规划，或 ROGMap 的碰撞安全门。在线图以 5 Hz 发布，轨迹标注会随新图刷新；同一轨迹刷新时保留已行进的路径进度。若在线标签造成不同 mode 控制窗重叠，本轮改用整条轨迹的静态标注。识别阈值仍需实车点云标定，细节见 ROGMap README。

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

- `prepare_s = enter_s - prepare_distance`：开始将 MPC 的速度、加速度约束向区域目标值过渡。
- `active_s = enter_s - activation_distance`：开始持续发送该区域的 mode。
- `commit_s = enter_s - commit_distance`：到这里较低的区域上限会被钳到目标值；坡道上限高于普通路段时仍按混合速率逐步提升。当前实现没有底盘回执，因此它是路径阶段标记，不能证明底盘已切换。
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

`/nav_executor/region_status` 包含轨迹 ID、label、mode、阶段、当前进度和速度上限。它表示导航侧**请求**的状态，不是下位机已执行的回执。

实现文件：`common/environment/region_control.*`、`src/nav_executor_node.cpp`。

## 5. 各 mode 的速度参数

`config/region_control.yaml` 为坡道、隧道、起伏路分别提供：

| 参数 | 含义 |
|---|---|
| `mode` | 下位机识别的底盘模式编号，必须与下位机协议一致 |
| `max_speed` | 区域平移合速度上限，m/s |
| `max_acceleration` | 区域平移加速度上限，m/s² |
| `normal_max_speed`、`normal_max_acceleration` | 普通路段的平移速度、加速度上限 |
| `prepare_distance`、`activation_distance`、`commit_distance`、`release_distance` | 上述路径阶段的距离，m |
| `speed_blend_rate` | 最大速度上限每秒允许变化的量 |
| `acceleration_blend_rate` | 加速度上限每秒允许变化的量 |

MINCO 在稀疏路径上读取原始区域标签，为坡道影响窗提供更高的分段速度上限，并对隧道、起伏路按各自上限约束，再做速度传播和轨迹优化。最终轨迹进入执行器后，`PathExecutor` 还会按当前区域 profile 更新 MPC 的 `vx/vy`、`ax/ay` 约束，并对最终平移合速度做上限校验。进入坡道时，上限在请求坡道 mode 后按 `speed_blend_rate`、`acceleration_blend_rate` 逐步提高；坡道外立即受普通路段上限约束。`omega` 的下发和区域控制不耦合，当前 `ros2_comm` 不转发它。

当前映射为普通路段 `mode=4`、上坡 `label=mode=5`、隧道 `label=mode=6`、起伏路 `label=mode=7`，均为 `uint8`；特殊区域若配置了与标签不同的 mode，导航节点会拒绝启动。普通路段上限为 `3.0 m/s`、`4.0 m/s²`；坡道上限为 `4.0 m/s`、`5.0 m/s²`，隧道及起伏路仍为普通路段上限。MINCO 和 MPC 的总上限允许坡道值，普通路段由 `normal_max_speed`、`normal_max_acceleration` 约束；MINCO 按静态标签使用坡道速度窗口，执行器还可按在线标签切换 mode 和约束。坡道准备、请求 mode、提交距离分别为 0.3/0.2/0.1 m；较高上限仅在请求坡道 mode 后逐步生效。实际速度仍由轨迹曲率、障碍净空、控制器与底盘能力决定；阶段距离仍需实车标定。

## 6. `ros2_comm` 与 `mas_vision` 的协议边界

导航节点发布的 `interfaces/ChassisCommand` 把 `vx`、`vy`、`mode` 放在同一条 ROS 消息中，避免速度和模式来自不同控制周期。`ros2_comm` 订阅它，固定发送 10 字节 UDP 载荷：`float vx`、`float vy`、`uint8 mode`、`uint8 nav_state`。帧头/长度/帧尾为 `0xBB, 10, payload, 0x5B`。不再提供 `legacy` 或把 mode 复用为 `nav_state` 的选项。

桌面工程 `/home/mas/桌面/mas_vision` 的 `ROS2_RECV_PACKET` 按同样顺序解析，再由 `sendNav()` 放入串口 `SendPacket`；串口中的 mode 同样紧挨着 `nav_state` 前面。串口包因此比旧版多 1 字节，接收长度、字段偏移和校验须由下位机同步修改，否则不能直接上车。`nav_state` 仍表示导航运动状态，不能替代 mode。ROS 指令超时后 `ros2_comm` 发送零速度与 `nav_state=0`，保留最近的 mode，避免车辆停在特殊区域时立即恢复自转。

`normal_mode` 是 `mas2027_nav_bringup/launch/nav_executor_launch.py` 的启动参数，默认 4；三类区域的 mode 和速度参数在 `config/region_control.yaml`。两处的普通 mode 编号必须一致。

## 7. 当前验证与限制

导航与桥接源码已更新；仍须编译、联调并把新版 `mas_vision` 与下位机串口解析同时部署。当前尚无实车验证，也没有下位机“模式已生效”的反馈，所以 `COMMITTED` 不能用于模式确认。地图仍需标注区域，速度与距离参数仍需标定。新版与旧版 UDP/串口包不兼容，切勿只升级链路的一端。
