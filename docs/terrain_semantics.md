# 实时与先验地形语义

导航的坡道约束共用 `buildSlopeSideMask()`：以坡道标签 `5` 的局部轮廓估算边界法向，并用上升向量或先验图的长轴区分两端和两侧。坡面两侧紧邻的普通格成为一格宽的硬障碍带，坡道入口和出口保持开放。侧边障碍进入先验规划约束或在线累计占据图；在线 ROGMap 搜索、局部路径处理和最终轨迹校验也检查同一障碍带，斜穿角点时连同两个相邻格一起检查。已有障碍和净空规则仍适用。

在线全局搜索使用实时 ROGMap 栅格，搜索边检查实时坡道侧边与累计地图中已确认的坡道侧边；斜穿角点时也检查两个相邻格。局部路径处理和发布前轨迹校验继续逐段检查侧边约束；MINCO 优化器、走廊及执行前指令门使用实时 ROGMap 距离场。侧边约束未并入 MINCO 的距离场。该方案不计算隧道顶部的三维净空，也不改变 `collision_dist` 阈值。

## 在线建图

ROGMap 在薄地面邻域拟合高度平面，得到坡道标签及指向高度增加方向的单位向量，目前自动识别坡道 `5` 和隧道候选 `6`。坡道拟合除 RMS 外还要求任一样本残差不超过 0.05 m，避免桌面/箱顶等单个抬升薄面把平地拟合成假坡；2026-10-18 的录制运行中，`(2.73,-0.07)` 附近一个高出地面约 0.2 m 的薄面即可在 11×11 窗口内造出 20°～30° 的假坡。隧道标签只从投影层已经判为 `PASSABLE/HOLLOW_TUNNEL` 的柱生成，并沿用 `projection.tunnel_height_delta_min/max` 与 `tunnel_occupancy_ratio_max`；投影层判为 `OCCUPIED` 的稀疏高柱不会再因为旧的 0.6～1.8 m 固定窗口而得到 `6`。`OnlineGlobalMap` 可累计观测到的特殊地形 `5/6/7`（`7` 的在线识别尚未实现）到 odom 坐标系。特殊标签需要连续 3 份新快照确认后才写入累计图；离开局部窗口或收到未知观测不会清除它。原位置连续 10 份完整普通地面观测会清除已记忆标签，供误识别纠错。两个次数由 `planner.online_global_map.terrain_confirmations` 和 `terrain_clear_confirmations` 配置，观测间隔受 `confirmation_max_gap_s` 约束。实时局部占据障碍仍即时生效；累计标签确认前，区域 mode 可能暂按普通地面处理，需要在实车坡道入口检查识别提前量。ROGMap 性能 CSV 的 `terrain_label_slope` / `terrain_label_tunnel` 是实际标签格数；执行器在 `RegionStatus` 的 label 或 mode 变化时打印 `Region status transition`，可直接核对是否在平地切入 `5/6`。在线累计占据图 `/nav_executor/online_global_map` 将已确认坡道的侧边格显示为障碍，`/nav_executor/online_terrain_map` 用于底盘区域模式，并发布 `/nav_executor/online_terrain_uphill`（`32FC2`，每格两个 `float32`，单位向量）供检查。RViz 默认显示 `/nav_executor/online_terrain_markers` 的累计区域，旧的 ROGMap 局部标记仍可用于核对当前视野。累计数据属于本次进程，重启后重新建图；不能把它当作跨次运行的持久地图。

## 先验地图

`map_server` 仍接受原有仅含 `width`、`height`、`resolution`、`terrain` 的 msgpack。可以额外提供 `uphill`：长度为 `2 × width × height` 的一维数值数组，按行优先排列，每格依次为 `x, y` 两个分量。坡道格填入指向高度增加方向的单位向量，其他格填 `0, 0`。地图服务检查数组长度、有限性、向量长度和非坡道格的零向量要求，并通过 `/terrain_uphill_map`（`32FC2`）发布。数组与地形标签、代价图共用尺寸、分辨率、原点和 frame。

旧地图没有 `uphill` 时，地图服务从每个至少 16 格且明显狭长的坡道连通区域计算长轴。这个轴线可用于生成两侧障碍带，**不能区分上坡与下坡**。近方形、断裂或分叉区域没有可靠长轴时，方向为零，这些格不会启用侧边障碍；需要准确上升方向时，应从高程数据或人工标注补充 `uphill`。坡道范围和向量应与地图坐标系一致；实车应核验入口边界、侧边障碍和车辆转弯半径。
