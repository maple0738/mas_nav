# Foxglove 远程可视化与调试

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

## 1. 链路

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

## 2. 机器人端

### 随 nav 栈一起起（推荐）

```bash
source install/setup.bash
# 默认已起 bridge，绑 127.0.0.1:8766（配合 SSH 隧道）
ros2 launch mas2027_nav_bringup nav_executor_launch.py

# 直连（笔记本装桌面版 Foxglove，直接连机器人 IP）
ros2 launch mas2027_nav_bringup nav_executor_launch.py foxglove_address:=0.0.0.0

# 不需要可视化时关掉
ros2 launch mas2027_nav_bringup nav_executor_launch.py use_foxglove:=False
```

### 单独起 bridge（nav 栈已用 use_foxglove:=False，或只想先开可视化）

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

### 限流白名单（点云太多、走 WiFi 卡时用）

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=8766 \
  topic_whitelist:="['^/cloud_registered$','^/nav_executor/minco_path$','^/nav_executor/debug/minco_trajectory$','^/nav_executor/global_plan$','^/Odometry$','^/tf.*']"
```

## 3. 笔记本端

### 方式 A：SSH 隧道（配合 `address:=127.0.0.1`）

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

### 方式 B：直连（配合 `address:=0.0.0.0`）

Foxglove 桌面版直接填 `ws://<机器人当前 wlo1 IPv4>:8766`；例如
2026-09-24 的地址是 `ws://192.168.77.141:8766`。此方式还需以
`foxglove_address:=0.0.0.0` 启动 bridge。

### 连接

Foxglove → `Open connection` → 选 **Foxglove WebSocket** → 填 URL → `Open`。

- 方式 A 填 `ws://127.0.0.1:8766`
- 方式 B 填 `ws://<机器人当前 wlo1 IPv4>:8766`

## 4. 面板配置

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

### 在 3D 面板查看轨迹

连接机器人后，打开 Foxglove 的 **3D** 面板：

1. online 模式 Fixed frame 选 `odom`，prior 模式选 `map`；
2. 在 Topics 中启用 `Path` → `/nav_executor/minco_path`，这是当前局部轨迹；
3. 需要按速度着色时，同时启用 `Marker` → `/nav_executor/debug/minco_trajectory`；
4. 想看搜索折线时启用 `Path` → `/nav_executor/global_plan`，它与局部轨迹是两条线，不重合是正常的。

`/opt_path` 是自定义 `interfaces/msg/MpcPositionCommand`，3D 面板不认；画线请用上面的 `Path` 话题。若话题存在但画面为空，先在机器人上执行 `ros2 topic echo /nav_executor/minco_path --once`，确认有 pose；再看 trajectory header 是否为 `odom`。

### 在 3D 面板查看在线坡道与隧道

连接机器人后，打开 Foxglove 的 **3D** 面板，将 Fixed frame 按模式设为 `odom`（online）或 `map`（prior）；在面板的 Topics 列表中找到并开启 `/rog_map/terrain_markers`。橙色方块是 ROGMap 在线识别的坡道候选格，青色方块是隧道候选格。这个显示只包含在线识别，不包含人工地图标注。需要核对原始标签时，再在同一个 3D 面板开启 `/rog_map/terrain_label`（`5`=坡道、`6`=隧道）；普通 OccupancyGrid 配色不会像彩色标记那样醒目。

若彩色话题没有出现，在机器人上检查 `ros2 topic info /rog_map/terrain_markers --verbose`：应能看到 ROGMap 发布器和 Foxglove 连接后产生的订阅。若曾按本文“限流白名单”示例单独启动 bridge，要把 `^/rog_map/terrain_markers$` 加进白名单，否则 Foxglove 看不到该话题。话题可见但画面为空时，先确认 `/rog_map/terrain_label` 当前确实出现 `5/6`，online 检查 `odom→base_link`、prior 检查 `map→odom`；没有识别到特殊地形时不会显示色块。3D 面板支持 ROS 2 `visualization_msgs/msg/MarkerArray`，无需另装面板插件。

Plot 面板排查"慢/卡顿"最直观：`/cmd_vel` 的 `twist.linear.x` / `twist.angular.z`，
`/Odometry` 的 `twist.twist.linear.x`。

`Layout → Save` 可把布局导出成 json，下次直接导入，不用重配。

## 5. 已知坑

### 5.1 握手子协议改了（本次卡最久的一条）

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

### 5.2 绑定地址

`address:=127.0.0.1` 时只有本机能连，远端必然连不上（也不会有明显报错）。远端要连
就必须 `address:=0.0.0.0`，或者老实用 SSH 隧道。

### 5.3 网页版的 mixed content

`https://studio.foxglove.dev` 是 https 页面，浏览器只允许它连 **localhost** 的
`ws://`。所以网页版**必须走 SSH 隧道**（隧道把服务映射到笔记本本机），不能填
`ws://<机器人当前 wlo1 IPv4>:8766`。

### 5.4 自定义消息类型在 3D 面板里加不了

`/opt_path`、`/backup_path` 是 `interfaces/msg/MpcPositionCommand`，3D 面板不认。
要画线优先用 `/nav_executor/minco_path`、`/nav_executor/global_plan`（都是 `nav_msgs/Path`）；
`/opt_path_vis`、`/backup_path_vis` 仍可作为兼容入口。要看数值用 Raw Messages 面板。

### 5.5 带宽

点云很吃带宽。`/cloud_registered_full` 与 `/cloud_registered` 内容相同，远程调试只选一个；
`/rog_map/*` 那六七个层全开，走 WiFi 会明显卡。必要时用 `topic_whitelist` 限流。

### 5.6 代理

机器人上有 `FlClash`（TUN 模式）。如果笔记本也开代理，记得把 `localhost` / `127.0.0.1`
加进 bypass，否则隧道到本机的那段可能被代理截走。

### 5.7 防火墙

机器人 `/etc/ufw/ufw.conf` 里是 `ENABLED=no`，所以方式 B 直连不需要额外放行。
若哪天启用了 ufw，直连需要：

```bash
sudo ufw allow from 192.168.77.0/24 to any port 8766 proto tcp
```

### 5.8 SSH 转发

`sshd_config` 没有显式设 `AllowTcpForwarding`（默认 `yes`），`X11Forwarding yes`。
隧道方案开箱可用。

## 6. 排障速查

| 现象 | 查什么 |
|---|---|
| Foxglove 连不上、无报错 | `ss -tln \| grep 8766`；核对当前机器人 IP；是不是绑了 `127.0.0.1` 又没开隧道；nav launch 是否已退出 |
| 连上立刻断、日志 `handshake failed` | 客户端太旧，不认 `foxglove.sdk.v1`（见 5.1） |
| 启动报 `Address already in use` | 已有一个 bridge 在跑，`pkill -f foxglove_bridge` 后再起 |
| 连上但没数据 | nav 栈没起，或 topic 被白名单滤掉 |
| 3D 里一片空白 | Fixed frame 不是 `map`；或 TF 链缺 `map`/`odom` |
| 卡成幻灯片 | 点云太多，收窄白名单（见 5.5） |

## 7. 远程调试常用动作

- **发目标点**：3D 面板里点 `Publish` → 选 `/goal_pose` → 在地图上点一下即发布导航目标。
  （bridge 默认 capabilities 含 `clientPublish`。）
- **手动重定位**：同理可发 `/initialpose`（`geometry_msgs/PoseWithCovarianceStamped`）。
- **录包留证据**：

  ```bash
  ros2 bag record -o /tmp/debug_bag /cloud_registered /opt_path /cmd_vel /Odometry /tf
  ```
