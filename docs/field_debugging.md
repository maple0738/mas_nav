# 现场记录与调试

## 现场录包

下一次实车复跑使用 `record_nav_diagnostics.sh` 收一份可离线分析的完整证据。记录器只订阅现有话题，不参与规划或控制，不改变导航行为。

### 启动

终端 1：

```bash
cd /home/mas/mas_nav_2027_native
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./src/mas2027_nav_bringup/scripts/record_nav_diagnostics.sh
```

记录器会打印 `RUN_DIR`，默认是：

```text
.scratch/field_runs/<YYYYMMDD_HHMMSS>/
```

看到 `Topics: ...` 和 `Now start the normal navigation launch...` 后，在终端 2 按原方式启动导航：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mas2027_nav_bringup nav_executor_launch.py <原来的参数>
```

跑完后回到终端 1 按 `Ctrl-C`。脚本会停止 bag、写 MCAP 索引，然后自动收集日志、参数和性能 CSV。

### 记录内容

MCAP bag 包含：

- 状态与命令：`/tf`、`/tf_static`、`/Odometry`、`/goal_pose`、`/cmd_vel`、`/nav_executor/chassis_cmd`、`/nav_executor/region_status`、`/parameter_events`
- 规划输出：`/opt_path`（执行器实际消费的 `MpcPositionCommand`）、`/nav_executor/minco_path`、`/nav_executor/global_plan`、`/nav_executor/debug/minco_trajectory`、`/nav_executor/debug/global_plan`、`/nav_executor/debug/safe_corridor`
- 累计地图：`/nav_executor/online_global_map`、`/nav_executor/online_terrain_map`、`/nav_executor/online_terrain_uphill`
- ROGMap 证据：`/rog_map/layer_type`、`/rog_map/layer_value`、`/rog_map/terrain_label`、`/rog_map/layer_height_delta`、`/rog_map/field`、`/rog_map/occupied`、`/rog_map/raw_occupied`
- 默认还录 `/cloud_registered_full`，用于回看点云投影

同时收集：

- `logs/`：本次运行新增的 ROS 节点日志（含 `launch.log`）
- `csv/`：本次运行新增的 `minco_perf_*`、`rog_map_perf_*`
- `params_planner.yaml`、`params_executor.yaml`：导航节点运行期间抓取，节点未出现或抓取失败时会告警；运行中参数变更可对照 bag 内的 `/parameter_events`
- `topics.txt`、`nodes.txt`、`bag_info.txt`、`manifest.txt`
- `config/`：本次启动使用的执行器、bringup `*.yaml` 和机器人 URDF 快照
- `README.txt`：本次运行摘要和排查提示

### 容量与选项

`/cloud_registered_full` 约占 1–3 GB / 5 分钟（取决于点云和压缩率）。磁盘紧张时：

```bash
FIELD_RECORD_CLOUD=0 ./src/mas2027_nav_bringup/scripts/record_nav_diagnostics.sh
```

只做短时自检，可让记录器自动停止：

```bash
FIELD_RECORD_SECONDS=30 ./src/mas2027_nav_bringup/scripts/record_nav_diagnostics.sh
```

指定输出目录：

```bash
./src/mas2027_nav_bringup/scripts/record_nav_diagnostics.sh /path/to/run_dir
```

### 分析入口

跑完后把 `RUN_DIR` 告诉分析侧即可。建议优先看：

- `Region status transition`：平地是否切入 `label=5/6`，以及切入位置
- `open set exhausted before goal` 中 `iterations>1` 的记录：全局搜索真实耗尽
- `bag_info.txt`：确认 `/rog_map/*`、累计地图和 `/cloud_registered_full` 均有消息
- `csv/rog_map_perf_detailed-*.csv` 的 `terrain_label_flat/slope/tunnel` 与分类计数
- 失败时刻附近的 `/rog_map/layer_type`、`/rog_map/layer_height_delta`、`/nav_executor/online_terrain_map`

### 注意

- 记录器应比导航启动得早；`transient_local` 话题即使晚订阅也可能只拿到之后发布的更新。
- `Ctrl-C` 后请等脚本打印 `Recorded run: ...` 再关闭终端，避免 MCAP 索引未落盘。
- 该脚本不启动、不重启、不配置导航节点；原有 launch 参数和安全逻辑不变。

## Foxglove 远程可视化

在另一台机器上通过 SSH 远程使用 Foxglove 观察/调试本仓库 nav 栈的操作手册。

现场环境（2026-09-17 实测，IP 仅为当日记录，连接前需重新查询）：

- 机器人：Ubuntu 24.04 + ROS 2 Jazzy，主机名 `mas-intel-1`，用户 `mas`
  - `wlo1`：`192.168.77.79`（SSH 从这里进来）
  - `enp2s0`：`192.168.1.50`
  - 机器上装了 `FlClash`（TUN，`198.18.0.0/30`）
- 上位机/笔记本：`192.168.77.15`（与机器人同一网段）
- 机器人已装 `ros-jazzy-foxglove-bridge 3.5.0` 与 Foxglove Studio 桌面版 `3.1.1`

机器人 Wi-Fi 地址会变化。连接前在机器人上运行 `ip -4 -brief addr show wlo1`，
把输出中的 IPv4 地址填入下面的 `ROBOT_IP`。2026-09-24 实测为 `192.168.77.141`。
在机器人上运行 `ss -lntp | grep 8766` 确认 bridge 正在监听；若没有输出，先启动 nav 栈或独立 bridge。

### 1. 链路

```text
笔记本 Foxglove ──ws://127.0.0.1:8766──▶ SSH 隧道 ──▶ 机器人 127.0.0.1:8766
                                                        foxglove_bridge
                                                              │ DDS
                                                              ▼
                                        nav_executor / ROGMap / map_server / ...
```

`foxglove_bridge` 已默认挂进 `nav_executor_launch.py`（`use_foxglove:=True`，
`foxglove_address:=127.0.0.1`，`foxglove_port:=8766`）。bridge 只负责把 DDS 图暴露成
WebSocket；`8766` 用于避开 mapping Web UI 已占用的 `8765`。关掉 bridge 时 nav 栈照常跑，
单独起 bridge、nav 没起时 Foxglove 能连上但没数据。

### 2. 机器人端

#### 随 nav 栈一起起（推荐）

```bash
source install/setup.bash
# 默认已起 bridge，绑 127.0.0.1:8766（配合 SSH 隧道）
ros2 launch mas2027_nav_bringup nav_executor_launch.py

# 直连（笔记本装桌面版 Foxglove，直接连机器人 IP）
ros2 launch mas2027_nav_bringup nav_executor_launch.py foxglove_address:=0.0.0.0

# 不需要可视化时关掉
ros2 launch mas2027_nav_bringup nav_executor_launch.py use_foxglove:=False
```

#### 单独起 bridge（nav 栈已用 use_foxglove:=False，或只想先开可视化）

```bash
# 方式 A：只绑回环 + SSH 隧道（推荐：端口不外露，不用动防火墙）
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=8766

# 方式 B：直连（笔记本装桌面版 Foxglove，直接连机器人 IP）
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8766
```

**不要**在 nav 栈已默认起 bridge 的同时再单独起一份，会
`Failed to bind port: Address already in use (os error 98)`。两种 address 也不能并存。

确认活着：

```bash
ss -tln | grep 8766        # 看到 127.0.0.1:8766 或 0.0.0.0:8766
```

正常启动日志：

```text
[foxglove_bridge-1] [INFO] [foxglove_bridge]: Server listening on port 8766
[foxglove_bridge-1] [INFO] [foxglove_bridge]: Advertising new channel 46 for topic "/cmd_spin"
```

停止：Ctrl-C 整个 nav launch 会一并停掉 bridge；若只想杀 bridge：

```bash
pkill -f foxglove_bridge
```

#### 限流白名单（点云太多、走 WiFi 卡时用）

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=8766 \
  topic_whitelist:="['^/cloud_registered$','^/nav_executor/minco_path$','^/nav_executor/debug/minco_trajectory$','^/nav_executor/global_plan$','^/Odometry$','^/tf.*']"
```

### 3. 笔记本端

#### 方式 A：SSH 隧道（配合 `address:=127.0.0.1`）

```bash
ROBOT_IP=192.168.77.141  # 换成机器人当前 wlo1 地址
ssh -N -L 8766:127.0.0.1:8766 mas@"$ROBOT_IP"
```

这条命令要一直挂着。建议写进 `~/.ssh/config` 免得每次敲；`HostName`
也要在机器人 Wi-Fi 地址变化后更新：

```sshconfig
Host mas-robot
    HostName 192.168.77.141
    User mas
    LocalForward 8766 127.0.0.1:8766
    ServerAliveInterval 30
    ServerAliveCountMax 3
```

之后 `ssh -N mas-robot` 即可。

#### 方式 B：直连（配合 `address:=0.0.0.0`）

Foxglove 桌面版直接填 `ws://<机器人当前 wlo1 IPv4>:8766`；例如
2026-09-24 的地址是 `ws://192.168.77.141:8766`。此方式还需以
`foxglove_address:=0.0.0.0` 启动 bridge。

#### 连接

Foxglove → `Open connection` → 选 **Foxglove WebSocket** → 填 URL → `Open`。

- 方式 A 填 `ws://127.0.0.1:8766`
- 方式 B 填 `ws://<机器人当前 wlo1 IPv4>:8766`

### 4. 面板配置

3D 面板的 **Fixed frame 要按地图模式选**：当前 `node.map_source: online` 时填 `odom`
（在线模式只发布 `odom → base_link → lidar_link`，没有 `map`）；切到 `prior` 模式时填 `map`
（`map → odom → base_link → lidar_link`，由 `tf_maintainer` 统一发布）。Fixed frame 不存在时 3D 里基本没东西可看，先查 TF。

| 面板类型 | Topic | 看什么 |
|---|---|---|
| TF | — | online 看 `odom→base_link`；prior 看 `map→odom→base_link` |
| PointCloud2 | `/cloud_registered_full` | ROGMap 使用的完整配准点云；`/cloud_registered` 为相同内容的兼容话题 |
| PointCloud2 | `/rog_map/occupied` | ROGMap 动态占据 |
| PointCloud2 | `/rog_map/field` | 2D 距离场 |
| MarkerArray | `/rog_map/terrain_markers` | 在线坡道（橙）/隧道（青）候选区域 |
| OccupancyGrid | `/rog_map/terrain_label` | 在线地形原始标签：5=坡道、6=隧道 |
| OccupancyGrid | `/cost_map`、`/planning_constraints` | 静态地形代价图 / 导航实际使用的静态约束图 |
| Path | `/nav_executor/minco_path` | **当前 MINCO 局部轨迹** |
| Marker | `/nav_executor/debug/minco_trajectory` | 同一局部轨迹的彩色线，按速度着色 |
| Path | `/nav_executor/global_plan` | 全局搜索折线（默认 SMAC 2D，可配置 Astar） |
| Marker | `/nav_executor/debug/global_plan` | 全局折线的粗线 + 目标点 |
| Path | `/opt_path_vis` | 旧可视化入口，内容与当前局部轨迹对应 |
| Path | `/backup_path_vis`、`/minco_candidate_path_vis` | 备份/候选轨迹 |
| MarkerArray | `/nav_executor/debug/safe_corridor` | 安全走廊 |
| Marker | `/nav_executor/debug/dynamic_obstacles` | 当前帧动态障碍 |
| Odometry | `/Odometry` | 位姿 |

#### 在 3D 面板查看轨迹

连接机器人后，打开 Foxglove 的 **3D** 面板：

1. online 模式 Fixed frame 选 `odom`，prior 模式选 `map`；
2. 在 Topics 中启用 `Path` → `/nav_executor/minco_path`，这是当前局部轨迹；
3. 需要按速度着色时，同时启用 `Marker` → `/nav_executor/debug/minco_trajectory`；
4. 想看搜索折线时启用 `Path` → `/nav_executor/global_plan`，它与局部轨迹是两条线，不重合是正常的。

`/opt_path` 是自定义 `interfaces/msg/MpcPositionCommand`，3D 面板不认；画线请用上面的 `Path` 话题。若话题存在但画面为空，先在机器人上执行 `ros2 topic echo /nav_executor/minco_path --once`，确认有 pose；再看 trajectory header 是否为 `odom`。

#### 在 3D 面板查看在线坡道与隧道

连接机器人后，打开 Foxglove 的 **3D** 面板，将 Fixed frame 按模式设为 `odom`（online）或 `map`（prior）；在面板的 Topics 列表中找到并开启 `/rog_map/terrain_markers`。橙色方块是 ROGMap 在线识别的坡道候选格，青色方块是隧道候选格。这个显示只包含在线识别，不包含人工地图标注。需要核对原始标签时，再在同一个 3D 面板开启 `/rog_map/terrain_label`（`5`=坡道、`6`=隧道）；普通 OccupancyGrid 配色不会像彩色标记那样醒目。

若彩色话题没有出现，在机器人上检查 `ros2 topic info /rog_map/terrain_markers --verbose`：应能看到 ROGMap 发布器和 Foxglove 连接后产生的订阅。若曾按本文“限流白名单”示例单独启动 bridge，要把 `^/rog_map/terrain_markers$` 加进白名单，否则 Foxglove 看不到该话题。话题可见但画面为空时，先确认 `/rog_map/terrain_label` 当前确实出现 `5/6`，online 检查 `odom→base_link`、prior 检查 `map→odom`；没有识别到特殊地形时不会显示色块。3D 面板支持 ROS 2 `visualization_msgs/msg/MarkerArray`，无需另装面板插件。

Plot 面板排查"慢/卡顿"最直观：`/cmd_vel` 的 `twist.linear.x` / `twist.angular.z`，
`/Odometry` 的 `twist.twist.linear.x`。

`Layout → Save` 可把布局导出成 json，下次直接导入，不用重配。

### 5. 已知坑

#### 5.1 握手子协议改了（本次卡最久的一条）

`foxglove_bridge 3.5.0` 基于新版 Rust `foxglove-sdk`，WebSocket 握手**只接受子协议
`foxglove.sdk.v1`**，不再接受老名字 `foxglove.websocket.v1`。旧客户端只发
`foxglove.websocket.v1` 时会被 400 掉，服务端日志是：

```text
ERROR foxglove::websocket::server] Dropping client 127.0.0.1:58700: handshake failed
```

HTTP 响应体是 `Missing expected sec-websocket-protocol header`（该字符串在
`/opt/ros/jazzy/lib/libfoxglove.so` 里）。

实测（向 bridge 端口发原始握手请求）：

| 客户端声明的子协议 | 结果 |
|---|---|
| `foxglove.websocket.v1` | `400 Bad Request` |
| `foxglove.sdk.v1` | `101 Switching Protocols` |
| 两个都声明 | `101`，服务端选 `foxglove.sdk.v1` |

**所以客户端必须够新。** Foxglove Studio 3.1.1 的客户端代码是
`SUPPORTED_SUBPROTOCOLS=["foxglove.websocket.v1","foxglove.sdk.v1"]`，两个都认；
版本旧就升级，或改用网页版 `https://studio.foxglove.dev`。

#### 5.2 绑定地址

`address:=127.0.0.1` 时只有本机能连，远端必然连不上（也不会有明显报错）。远端要连
就必须 `address:=0.0.0.0`，或者老实用 SSH 隧道。

#### 5.3 网页版的 mixed content

`https://studio.foxglove.dev` 是 https 页面，浏览器只允许它连 **localhost** 的
`ws://`。所以网页版**必须走 SSH 隧道**（隧道把服务映射到笔记本本机），不能填
`ws://<机器人当前 wlo1 IPv4>:8766`。

#### 5.4 自定义消息类型在 3D 面板里加不了

`/opt_path`、`/backup_path` 是 `interfaces/msg/MpcPositionCommand`，3D 面板不认。
要画线优先用 `/nav_executor/minco_path`、`/nav_executor/global_plan`（都是 `nav_msgs/Path`）；
`/opt_path_vis`、`/backup_path_vis` 仍可作为兼容入口。要看数值用 Raw Messages 面板。

#### 5.5 带宽

点云很吃带宽。`/cloud_registered_full` 与 `/cloud_registered` 内容相同，远程调试只选一个；
`/rog_map/*` 那六七个层全开，走 WiFi 会明显卡。必要时用 `topic_whitelist` 限流。

#### 5.6 代理

机器人上有 `FlClash`（TUN 模式）。如果笔记本也开代理，记得把 `localhost` / `127.0.0.1`
加进 bypass，否则隧道到本机的那段可能被代理截走。

#### 5.7 防火墙

机器人 `/etc/ufw/ufw.conf` 里是 `ENABLED=no`，所以方式 B 直连不需要额外放行。
若哪天启用了 ufw，直连需要：

```bash
sudo ufw allow from 192.168.77.0/24 to any port 8766 proto tcp
```

#### 5.8 SSH 转发

`sshd_config` 没有显式设 `AllowTcpForwarding`（默认 `yes`），`X11Forwarding yes`。
隧道方案开箱可用。

### 6. 排障速查

| 现象 | 查什么 |
|---|---|
| Foxglove 连不上、无报错 | `ss -tln \| grep 8766`；核对当前机器人 IP；是不是绑了 `127.0.0.1` 又没开隧道；nav launch 是否已退出 |
| 连上立刻断、日志 `handshake failed` | 客户端太旧，不认 `foxglove.sdk.v1`（见 5.1） |
| 启动报 `Address already in use` | 已有一个 bridge 在跑，`pkill -f foxglove_bridge` 后再起 |
| 连上但没数据 | nav 栈没起，或 topic 被白名单滤掉 |
| 3D 里一片空白 | Fixed frame 与当前模式不符：online 用 `odom`、prior 用 `map`；或对应 TF 链缺失 |
| 卡成幻灯片 | 点云太多，收窄白名单（见 5.5） |

### 7. 远程调试常用动作

- **发目标点**：3D 面板里点 `Publish` → 选 `/goal_pose` → 在地图上点一下即发布导航目标。
  （bridge 默认 capabilities 含 `clientPublish`。）
- **手动重定位**：同理可发 `/initialpose`（`geometry_msgs/PoseWithCovarianceStamped`）。
- **录包留证据**：

  ```bash
  ros2 bag record -o /tmp/debug_bag /cloud_registered /opt_path /cmd_vel /Odometry /tf
  ```

## 窄道碰撞与轨迹拒绝

本文适用于 `mas2027_nav_executor` 使用 ROGMap 在线距离场通过窄道时的实车排查。每次只改一组参数，保存对应的配置快照与录包；发生实体接触后先停车，确认碰撞阶段和地图证据，再进行下一次低速验证。

### 当前几何与通过边界

- 已确认车体为正方形，中心到角点的旋转半径为 `0.302 m`，对齐直行的半宽约为 `0.302 / √2 = 0.214 m`。`planner.corridor.swept_radius` 记录前者，不能把它直接设为整条直行轨迹的中心净空门槛。
- `20261018_130912` 的同一帧 ROGMap 距离场中，起点到目标按 `0.25 m` 最低净空搜索时连通，按 `0.26 m` 时断开；瓶颈约在 `(2.54,-1.24)`，读数约 `0.25 m`。因此统一要求 `0.30 m` 会在地图上判无路。`20261018_135419` 的历史版本用 `0.27/0.25 m` 主/回退搜索、`0.24 m` 窄道地板完成 4/4 目标；该记录不验证后来修改的版本。
- ROGMap 距离场到占据**格心**，栅格为 `0.05 m`。若墙边界沿栅格轴向且车体完全对齐，`0.25 - 0.025 - 0.214 ≈ 0.011 m` 是一侧的名义间隙。实际墙面、定位偏差和航向偏差未包含在内。
- 车体在偏角 `θ` 下，朝直墙的半宽近似为 `0.214 × (|cos θ| + |sin θ|)`。`5°` 时约 `0.231 m`，已超过上面粗算的 `0.225 m` 格心至墙边界距离。因此当前 `ready_tolerance_deg=5`、`ready_release_deg=8` 不能作为窄道安全保证。

### 先判断碰撞发生在哪一阶段

| 现场现象 | 优先核对 | 调整位置与试验值 | 影响和边界 |
|---|---|---|---|
| 入窄道时车体仍在转、角点擦墙 | `/nav_executor/region_status` 的 `gimbal_heading_error_deg`、`gimbal_heading_ready`；`/nav_executor/chassis_cmd` 的 `follow_gimbal`、`vx/vy`、`small_yaw_ref`。确认实际车体航向，而不只看角度命令。 | `mas2027_nav_executor/config/region_control.yaml`：`gimbal_follow.ready_tolerance_deg` 先由 `5` 试到 `1`，`ready_release_deg` 由 `8` 试到 `2`，`settle_time_s` 由 `0.3` 试到 `0.5`。若对齐启动太晚，再检查 `planner_params.yaml` 的 `narrow_passage_alignment_lookahead=2.0 m` 与实际可用的入口空间。 | 更严格的对齐会使车在入口等待；若航向噪声或下位机响应无法达到要求，应查反馈链路，不要仅靠放宽角度让车起步。`heading_ready` 基于 LIO 航向，不是下位机到位确认。 |
| 航向已稳定，沿墙直行时车侧擦墙 | 同时对齐 `/Odometry`、`/opt_path`、`/rog_map/field` 与 `/rog_map/layer_value`，看实车相对轨迹的横向偏差，以及碰撞位置的地图净空。 | `planner_params.yaml`：`minco_optimizer.narrow_passage_speed` 先由 `0.5` 试到 `0.2 m/s`。若最终轨迹本身贴墙，再将 `narrow_passage_clearance` 由 `0.24` 试到 `0.25 m`。 | 降速有助于减少跟踪偏差，但不会改变静态间隙；提高地板会拒绝低于 `0.25 m` 的轨迹。历史地图在 `0.26 m` 已断开，继续提高可能只能停车。`safe_dist=0.33` 是软目标，不能代替硬门。 |
| 到达目标或离开窄道时开始自转并擦墙 | 核对 `Chassis alignment transition`、`follow_gimbal` 切换瞬间的位置和 ROGMap 净空；确认是否已离开狭窄区。 | `planner_params.yaml`：`corridor.extra_margin` 可由 `0.05` 试到 `0.07 m`，使自由旋转释放更晚。 | 它作用于旋转空间要求，不会为直行轨迹增加侧向间隙。保持实测 `swept_radius=0.302 m`，不要靠改小半径换取通过。 |
| 车已在窄道内且航向不正，需要原地转向 | 检查当前位置是否具备旋转外廓所需空间。 | 停止当前自动尝试，从可旋转的位置重新对齐后再进入。 | 这个场景不能靠降低 `narrow_passage_clearance` 解决；窄道直行可通过不代表能原地旋转。 |
| 实车撞到墙，但二维地图将该墙显示为自由或净空明显偏大 | 对比 `/cloud_registered_full`、`/rog_map/raw_occupied`、`/rog_map/layer_value`、`/rog_map/field`，并核对点云与里程计时间；先判断是点云未观测、姿态错位还是地图过滤。 | `planner_params.yaml` 中 `projection.body_exclusion_radius=0.302 m` 是入图前自车回波过滤轮廓；设 `0` 仅作为重新建图的对照实验。 | 设 `0` 可能引入自车回波造成误停，不能把它当作常规通行调参。若观测墙被漏掉，先修地图/时序，继续提高轨迹门槛也无法可靠判断真实实体间隙。 |

上述角度和速度是**排查试验值**，不是已通过实车认证的安全设置。若收紧到约 `1°`、降到 `0.2 m/s` 后仍擦墙，就要量测实体间隙、轨迹跟踪偏差和地图/定位误差；在约 `0.25 m` 的地图瓶颈上继续降低净空门槛不能制造物理空间。

### 哪些参数不要混着调

- `planner.smac_2d.minimum_clearance=0.27 m` 是主搜索门槛；`relaxed_minimum_clearance=0.25 m` 在主搜索无解时提供参考线。提高回退门槛到 `0.26 m`，历史那张地图会失去连通性。
- `minco_optimizer.collision_dist=0.28 m` 对应普通轨迹约 `0.26 m` 的有效硬门；窄道续行可使用独立的 `narrow_passage_clearance=0.24 m`。只提高 `collision_dist`，不能等同于提高窄道绝对地板。
- `corridor.robot_radius=0.28 m` 用于安全走廊形状；`corridor.swept_radius=0.302 m` 是实测旋转外廓；`corridor.extra_margin=0.05 m` 作用于旋转释放。三者不互相替代。
- `narrow_passage_alignment_hold_distance=2.0 m` 是对齐模式的最短保持里程，不是撞墙时的直行净空门槛。
- 配置文件不是运行时参数的唯一来源：`mas2027_nav_bringup/launch/nav_executor_launch.py` 在 `online` 模式会覆盖 YAML，将 `planner.exploration.unknown_as_occupied` 和 `planner.rog_map.projection.unknown_as_occupied` 都设为 `true`。此时 ROGMap 仍会针对 `ray_range` 下限 `0.3 m` 的三维近距盲区执行强制 free 更新；机器人附近显示为 free 的格需要结合原始点云核实，不能只看 YAML 或二维净空。以本次运行抓取的 `params_planner.yaml`、`params_executor.yaml` 和录包为准。

### 一次可复核的现场试验

1. 先用 [现场导航运行记录](#现场录包) 启动 `record_nav_diagnostics.sh`，再按原方式启动导航。记录器保存 bag、配置快照、日志和性能 CSV。
2. 只选择一处可重复的窄道，记录入口位置、行驶方向、墙体实体间距和具体接触点。第一次按当前配置低速观察；若发生接触，停车并保存该次 `RUN_DIR`。
3. 在同一时间轴上核对 `/Odometry`、`/opt_path`、`/nav_executor/chassis_cmd`、`/nav_executor/region_status`、`/rog_map/field` 和点云。先分类为“转向擦碰”“直行贴墙”“退出时自转”或“地图漏墙”，再按上表只改对应的一组参数。
4. 修改 YAML 后重启导航；同时核对记录器抓取的 `params_planner.yaml`、`params_executor.yaml`，确认 launch 覆盖后的实际参数。对比改动前后：最小地图净空、最大航向误差、最大横向跟踪偏差、是否有实体接触、是否出现轨迹拒绝或持续停车。只有重复通过且实体间隙经量测仍足够，才可认为该路段完成标定。
