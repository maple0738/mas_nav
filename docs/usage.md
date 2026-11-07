# 导航使用说明

以下路径均相对仓库根目录 `mas_nav/`。参数在启动时读取，修改后重启导航。默认使用无图模式。

## 编译与启动

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to mas2027_nav_bringup --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch mas2027_nav_bringup nav_executor_launch.py use_rviz:=True
```

默认启用底盘串口；仅观察时加 `use_nav_serial:=False`。RViz 中用 2D Goal Pose 下目标。无图模式目标使用 `odom`，有图模式使用 `map`。

启动默认读取安装目录中的配置。调试源码参数时，可直接指定配置路径，修改 YAML 后重启即可，无需为 YAML 改动重新编译：

```bash
ros2 launch mas2027_nav_bringup nav_executor_launch.py \
  navigation_config:="$PWD/mas2027_nav_bringup/config/navigation_map.yaml" \
  config_dir:="$PWD/mas2027_nav_executor/config" use_rviz:=True
```

程序改动需要重新编译。`navigation_config` 管模式、地图和日志；`config_dir` 管下表六份执行器参数。启动日志打印本次采用的配置、模式和地图绝对路径。

## 有图与无图

唯一默认入口：`mas2027_nav_bringup/config/navigation_map.yaml` 的 `map_source`。

| 值 | 工作方式 | 地图要求 |
| --- | --- | --- |
| `online` | LIO + 在线 ROGMap；不启动先验定位和静态地图服务 | 不需要地图文件；RViz 固定坐标系为 `odom` |
| `prior` | PCD 定位 + 静态全局地形，ROGMap 提供实时局部障碍 | PCD、占据 YAML/图像、地形 msgpack 必须齐全；固定坐标系为 `map` |

也可以临时传 `map_source:=online` 或 `map_source:=prior`，覆盖配置文件。不要在 `node_params.yaml` 再写一份模式；启动器会统一注入。无图模式强制未知区域不可通行，不能通过修改普通搜索参数放开。

地图分为三个职责：ROGMap 生成实时局部障碍、地面几何证据和地形标签；在线累计图负责历史确认、纠错和记忆；先验静态图仅在 `prior` 模式提供全局约束。二维距离场是各自障碍层的派生缓存。局部边界检测与历史纠错使用不同证据条件，台阶阈值和碰撞门保持一致的配置口径。

`prior` 模式的 `terrain_map_server` 在启动时加载并发布 `/cost_map`、`/terrain_label_map`、`/terrain_uphill_map`，采用持久化 QoS，后启动的导航订阅者也能收到。静态图不周期重发；接收端只在内容变化时更新版本和派生缓存。先验约束统一由 `TerrainGrid → TerrainMapQuery → FusedMapQuery` 接入，ROGMap 不再单独加载或融合二维先验图。`online` 模式不启动该服务。ROGMap 实时二维障碍统一查看 `/rog_map/layer_value`，不再发布重复的 `layer_value_dynamic/static`。

## 地图放哪里

默认地图在 `mas2027_nav_bringup/pcd/` 和 `mas2027_nav_bringup/map/`，具体文件由 `navigation_map.yaml` 选择。也可将同一场地的一套地图放到外部目录，例如：

```text
/data/navigation/maps/lab/
  lab.pcd
  lab.yaml
  lab.pgm
  lab_terrain.msgpack
```

对应配置：

```yaml
map_source: prior
map_root: /data/navigation/maps/lab
log_dir: ~/.local/state/mas_nav
map_files:
  localization_pcd: lab.pcd
  occupancy_yaml: lab.yaml
  terrain_msgpack: lab_terrain.msgpack
```

`map_root` 的相对路径以此配置文件所在目录为基准；`map_files` 的相对路径以 `map_root` 为基准；YAML 的 `image` 以该地图 YAML 所在目录为基准。以上均支持绝对路径和 `~`。四个文件应来自同一次建图、使用同一坐标系、原点和分辨率。有图启动前会检查文件存在性；不检查地图是否来自同一次导出。外部地图不会复制进 `install/`，运行机器上需保留该目录。

默认 `map_root: ..` 指向 bringup 包根目录。构建时将清单选中的包内地图安装到 share 目录；无图构建允许地图缺失。先验定位的 PCD 和静态地形服务均由这份清单绑定。

## 修改哪个参数文件

执行器配置目录：`mas2027_nav_executor/config/`。

| 文件 | 内容 |
| --- | --- |
| `node_params.yaml` | 话题、坐标系、控制/规划频率、超时与显示 |
| `perception.yaml` | ROGMap 分辨率、范围、点云、衰减、障碍/台阶、ESDF 与性能统计 |
| `terrain_recognition.yaml` | 坡面 5、隧道 6、起伏路 7 的在线识别阈值 |
| `planner_params.yaml` | 全局搜索、MINCO、探索恢复、在线累计图的确认与清除 |
| `region_control.yaml` | 各地形模式、速度/加速度、准备/激活距离、航向对齐与起伏定长锁存 |
| `mpc_params.yaml` | 跟踪控制的约束、预测窗口与权重 |

雷达/LIO 参数在 `mas2027_nav_bringup/config/small_point_lio_params.yaml`；先验定位算法参数在 `mas2027_perception/Localization/odom_localizer/config/params.yaml`。启动器按固定文件清单加载，额外放入的 YAML 不会自动覆盖参数；清单内重复定义同一参数会在启动前报错。命令行 `normal_mode`、`output_topic` 仅在显式填写时覆盖 YAML。

## 地形调参

`terrain_recognition.yaml` 中的长度单位为米。坡度为“高差/水平距离”，不是角度；峰谷差是最高点到最低点的差，不是单边振幅。默认阈值保持当前算法行为，不能代表现场标定结果。

| 地形 | 优先调整 | 作用 |
| --- | --- | --- |
| 坡面 | `terrain.slope.min_gradient/max_gradient` | 默认 0.14～0.70，约 8°～35° |
| 坡面 | `fit_radius_cells`、`min_samples` | 默认 11×11 窗口、30 个有效地面格；窗口变大计算更多，也可能混入邻近物体 |
| 坡面 | `min_height_change_m`、`max_plane_rms_m`、`max_plane_residual_m` | 高差下限和拟合误差上限；后两项减小更严格 |
| 隧道 | `projection.tunnel_height_delta_min/max`、`tunnel_occupancy_ratio_max` | 柱上下占据回波的跨度和竖直占据率，用于柱分类；不是底盘可通行净空 |
| 隧道 | `terrain.tunnel.min_free_gap_ratio`、`min_neighbor_support` | 中间明确为空闲的比例、3×3 邻域支持格数；提高更严格 |
| 起伏路 | `terrain.undulating.min_peak_to_trough_m/max_peak_to_trough_m` | 默认峰谷差 0.045～0.12；提高下限可抑制小扰动 |
| 起伏路 | `min_wavelength_m/max_wavelength_m` | 默认同类极值间距 0.18～0.34 |
| 起伏路 | `max_trend_gradient` | 去趋势前的整体坡度上限，默认 0.12，必须小于坡面下限 0.14；起伏路只认接近水平的路段 |
| 起伏路 | `min_delta_m`、`min_neighbor_correlation`、`min_common_samples` | 反转噪声门、邻条带一致性和共同实测覆盖；相关系数默认 0.85，提高更严格 |
| 起伏路 | `entry_window_enable` | 关闭后仅使用完整窗口，识别更晚；默认保留入口短窗口 |
| 起伏路 | `front_signature_enable` | 正面观测签名；关闭后只能靠剖面出现反转来识别 |
| 起伏路 | `front_min_face_height_m`、`front_min_face_cells` | 上坡段最小升高（默认 0.045 m）和最短连续格数；提高更严格 |
| 起伏路 | `front_approach_cells`、`front_approach_tolerance_m` | 坡底后方必须是接近水平的地面（默认 5 格、0.03 m 容差）；长坡后方仍在升高，因此不会被当成起伏 |
| 起伏路 | `front_min_hidden_cells`、`front_max_hidden_cells` | 峰顶后被遮挡、无地面回波的格数范围；用于区分“波谷被挡住”和台阶/箱顶 |
| 起伏路 | `front_crest_height_tolerance_m`、`front_min_crest_cells`、`front_max_crest_cells` | 遮挡段后地面与第一峰顶的同高容差、最短/最长长度；持续同高平台不算起伏 |
| 起伏路 | `front_min_lateral_support` | 上坡方向两侧条带支持数（1..3，默认 3）；降低可提高斜向通过率，也更易误报 |

`terrain.*.enable` 仅关闭对应在线语义识别，不会关闭障碍分类，也不会关闭静态地图里已有的标签。完整剖面判据必须有精细地面高度，粗体素高度不能单独生成标签；正面签名则允许坡面使用地面参考高度，只对遮挡段后的同高峰顶做 0.04 m 容差比较。四方向、三邻条带、17 点完整窗口/13 点入口窗口是算法结构；覆盖不足时拒识。

起伏路和坡面靠趋势分开：完整剖面先拟合并扣除整体坡度，剩余坡度必须不超过 `max_trend_gradient`（默认 0.12，低于坡面下限 0.14），再看峰谷差、波长和邻条带一致性。因此长坡不会仅因为路面噪声被反推成起伏；坡面上的小起伏也会先被 0.045 m 的峰谷差下限挡掉。只有一处起伏路、坡面多时，优先保持 `max_trend_gradient` 低于坡面下限，不要为了更早识别去放宽它。

从正面接近时，第一个峰顶会挡住后面的波谷，剖面上只剩一段单调上坡，没有反转可供 17 点判据使用。此时 `front_signature_enable` 沿坡面拟合出的上坡方向检查“水平起点 + 上坡段 + 无地面回波的遮挡段 + 与第一峰顶同高的下一段地面”，并要求上坡方向两侧条带同时成立。起点后方必须是水平地面，这是为了拒绝长坡：长坡从任意坡格起扫时后方仍在升高，不会满足。该签名只补充标签 7，不改变碰撞掩码、台阶门或净空判定；遮挡段内出现占据格会直接拒绝。

平地误报起伏路时，先检查精细高度、峰谷差和邻条带一致性，再逐项调整阈值。已经记忆的标签受 `planner.online_global_map.terrain_clear_confirmations` 影响；起伏对齐保持距离在 `region_control.undulating.alignment_latch_distance`，调识别阈值不会立即解除已有锁存。需要重新评估一次识别时重启进程清空本次累计状态。

隧道标签 `6` 从准备窗开始请求沿当前 MINCO 路径轴线对齐，达到激活窗后进入 mode 6；未就绪时平移保持为零。准备、激活、释放距离在 `region_control.tunnel` 调整，航向容差和稳定时间在 `gimbal_follow` 调整。规划短暂无有效航向时，已有对齐角度按 `hold_last_alignment` 保持并停车；退出区域或任务结束后解除。普通几何窄处不再触发对齐，仍保留净空、限速和制动判定；坡面 `5`、起伏路 `7` 沿用语义对齐。

## 查看状态

Path 和 Marker 按话题订阅情况生成；无人查看时跳过可视化构造。RViz 后打开会自动补发当前数据，局部轨迹和静态约束 Marker 的补发检查频率由 `node.visualization.global_plan_publish_hz` 控制。规划使用的地图话题不受可视化订阅开关影响；静态代价、标签和方向图按任意顺序到达后，约束栅格均会在地图就绪时发布。

```bash
ros2 param get /nav_executor node.map_source
ros2 param get /nav_executor planner.rog_map.projection.terrain.undulating.min_peak_to_trough_m
ros2 topic echo /nav_executor/region_status --once
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered_full
```

`/rog_map/terrain_markers` 中橙色方块为坡面 `5`、青色为隧道 `6`、紫色为起伏路 `7`；`/rog_map/terrain_label` 提供逐格原始数值。两者只在有订阅者时计算。

性能 CSV 默认位于 `~/.local/state/mas_nav/`，由清单中的 `log_dir` 修改，启动时自动创建。`terrain_geometry_time_ms` 看地形识别总耗时；`terrain_slope_fit_count` 看平面求解次数；`terrain_wave_profile_check_count` 看未命中缓存的条带检测次数；`terrain_wave_profile_cache_hits` 看同帧复用次数。Detailed CSV 为每次更新，Summary CSV 的 `last_*` 为最近一次更新值。统计开关位于 `perception.yaml` 的 `performance`。

出现 `rog_map_stale`/`tf_unavailable` 先检查数据和 TF，`clearance` 检查净空，`terrain_transition` 检查地形方向。识别标签不代替碰撞、台阶和制动门。
