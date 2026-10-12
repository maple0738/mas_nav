# 区域控制：隧道、坡道、起伏路

本文解释当前导航侧的区域控制实现。导航内部沿规划轨迹切换区域速度参数和 `mode` 状态；当前 CDC1 串口协议只接收速度与 `nav_state`，尚不能把 `mode` 传给底盘。底盘自转行为由下位机自身决定。导航侧也不向底盘传 `omega`。

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
  → nav_serial_bridge → CDC1 导航专用串口 → 下位机
```

`/cmd_vel` 仍发布，供现有工具观察速度；`nav_serial_bridge` 实际消费的是 `/nav_executor/chassis_cmd`，只取 `vx`、`vy`。区域控制的状态另发到 `/nav_executor/region_status`，但其中的 mode 不出串口。

## 2. 地图如何表示区域

| 原始 label | 名称 | 本车用途 |
|---:|---|---|
| 0 | `FLAT` | 普通地面 |
| 1 | `OBSTACLE` | 障碍 |
| 5 | `SLOPE` | 上坡速度约束；导航内部 mode 5 |
| 6 | `TUNNEL` | 隧道速度约束；导航内部 mode 6 |
| 7 | `UNDULATING` | 起伏路速度约束；导航内部 mode 7 |

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

MINCO 在稀疏路径上读取原始区域标签，为坡道影响窗提供更高的分段速度上限，并对隧道、起伏路按各自上限约束，再做速度传播和轨迹优化。最终轨迹进入执行器后，`PathExecutor` 还会按当前区域 profile 更新 MPC 的 `vx/vy`、`ax/ay` 约束，并对最终平移合速度做上限校验。进入坡道时，上限在导航内部请求坡道 mode 后按 `speed_blend_rate`、`acceleration_blend_rate` 逐步提高；坡道外立即受普通路段上限约束。当前串口既不发送 mode，也不发送 `omega`。

当前映射为普通路段 `mode=4`、上坡 `label=mode=5`、隧道 `label=mode=6`、起伏路 `label=mode=7`，均为 `uint8`；特殊区域若配置了与标签不同的 mode，导航节点会拒绝启动。普通路段上限为 `3.0 m/s`、`4.0 m/s²`；坡道上限为 `4.0 m/s`、`5.0 m/s²`，隧道及起伏路仍为普通路段上限。MINCO 和 MPC 的总上限允许坡道值，普通路段由 `normal_max_speed`、`normal_max_acceleration` 约束；MINCO 按静态标签使用坡道速度窗口，执行器还可按在线标签切换 mode 和约束。坡道准备、请求 mode、提交距离分别为 0.3/0.2/0.1 m；较高上限仅在请求坡道 mode 后逐步生效。实际速度仍由轨迹曲率、障碍净空、控制器与底盘能力决定；阶段距离仍需实车标定。

## 6. 导航专用 CDC1 协议

导航节点发布的 `interfaces/ChassisCommand` 仍把 `vx`、`vy`、`mode` 放在同一条 ROS 消息中。`nav_serial_bridge` 只取速度，向下位机 CDC1 接口发送 12 字节帧：`0xB5`、小端 `float vx`、小端 `float vy`、`uint8 nav_state`、小端 CRC16。CRC 使用 `crc_rm` 的初值 `0xFFFF`、反射多项式 `0x8408`。速度非零且指令新鲜时 `nav_state=1`；速度为零、没有指令或超过 300 ms 未收到新指令时发送零速度与 `nav_state=0`。发送周期 20 ms，串口为 115200 8N1。

下位机回传为 11 字节帧：`0x5B`、8 字节 `NavSendPacket` 裁判数据、CRC16。桥目前校验并统计回传帧，没有将它转成 ROS 消息。当前接收结构 `NavReceivePacket` 只有速度与 `nav_state`，没有 mode；原视觉 UDP/串口转发不参与这条链路。

`normal_mode` 是 `mas2027_nav_bringup/launch/nav_executor_launch.py` 的导航内部启动参数，默认 4；三类区域的 mode 和速度参数在 `config/region_control.yaml`。两处的普通 mode 编号必须一致，但当前都不作为串口字段发送。

## 7. 当前验证与限制

CDC1 的零速度报文与回传 CRC 已在硬件接口上测试；非零速度仍需先通过虚拟串口校验，再做实车联调。底盘不会收到 mode，所以在下位机协议扩展前，区域 `COMMITTED` 不能用于底盘模式确认。地图仍需标注区域，速度与距离参数仍需标定。
