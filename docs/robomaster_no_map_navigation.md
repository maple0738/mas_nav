# RoboMaster 无图导航优化方案：从点目标探索到战场区域导航

> 状态：方案设计文档。本文描述建议的目标、规划和验证方式，不代表当前仓库已经实现。
> 当前代码仍是“精确点目标 + 实时滚动窗口 + 累计地图辅助”的实现，不能把本文当作现行行为说明。

## 1. 结论

RoboMaster 赛场不适合把通用未知环境 frontier 探索作为主策略。更合适的目标是：

> **固定赛场拓扑 + 近似目标区域 + 目标导向探索 + 绕障分支锁 + 实时局部安全 + 比赛时间回退。**

纯 frontier 探索的问题不是“不能走”，而是：

- 场地固定、目标语义明确，重新探索整张图没有必要；
- 比赛时间短，一次 40 s 的来回探索就可能吃掉 10% 的比赛时间；
- 目标通常不是精确点，而是敌方基地、高地入口、隧道口、巡逻/防守区域或移动目标；
- 目标可能被高地、基地或隧道遮挡，LiDAR 也可能因为目标材质、安装高度或距离没有稳定回波；
- 对手是高速动态障碍，导航需要保留退路，不能为了覆盖未知区域进入死胡同。

因此建议把导航问题拆成两层：

```text
战术层：GoalRegion / Zone Graph / 比赛时间
                 │
                 ▼
全局层：累计地图上的区域路线、目标状态、分支选择
                 │
                 ▼
局部层：实时 ROGMap 连通性、净空、MINCO、MPC、指令安全门
```

累计地图可以用未知格做“乐观方向搜索”，但 **实时执行仍然把未知格当作不可通行**。不能把 `unknown_as_occupied=false` 当作探索提速手段。

---

## 2. 现场运行暴露的问题

以 `20261020_165514` 这次现场运行为例：

| 指标 | 结果 |
|---|---|
| 运行时长 | 129.7 s |
| 目标数 | 2 |
| 目标 1 | 请求 `(8.23, -1.20)`，该点位于膨胀障碍内；到达安全弹出点 `(7.43, -1.83)`，距原目标约 1.02 m |
| 目标 1 耗时 | 64.9 s |
| 目标 2 | 返回 `(-0.02, 0.05)`，实际终点 `(0.01, 0.06)` |
| 目标 2 耗时 | 36.6 s |
| 总路径 | 约 79.1 m |
| 运动构成 | 约 54.6% 平移、42.3% 原地旋转、3.1% 完全停车 |

最突出的不是规划耗时，而是目标 1 前的局部探索：

- 13.2～56.5 s 约 43.3 s 内走了约 23.8 m；
- 净位移只有约 0.13 m；
- 共提交 13 次探索子目标；
- 7 次因实时连通性丢失而换点，3 次报告目标推进受阻；
- 子目标在南北、东西方向反复跳转。

规划器本身仍有约 450 次重规划、393 次成功、57 次失败（29 次 `COLLISION`、28 次 `OPTIMIZER_FAILED`），但总重规划耗时 p95 约 14.5 ms。也就是说，这次的主要瓶颈是 **探索决策和路径方向反复**，不是算力。

如果 RM 一场按 7 分钟计，43 s 的局部来回接近 10% 的比赛时间；目标 1 的 64.9 s 超过 15%。这类路径在赛场上不可接受。

---

## 3. 目标定义：从 PoseStamped 到 GoalRegion

### 3.1 目标结构

建议新增近似目标结构：

```yaml
goal:
  zone_id: enemy_base
  center: [8.2, -1.2]
  radius: 1.5
  approach_direction: any
  require_visibility: false
  deadline_s: 60
  success_mode: enter_region   # enter_region | visible | closest
```

字段含义：

| 字段 | 含义 |
|---|---|
| `zone_id` | 战术区域 ID，例如敌方基地、中央高地、隧道口 |
| `center` | 近似坐标，不要求是精确可通行格 |
| `radius` | 进入该半径即算近似到达 |
| `approach_direction` | 允许从哪个方向进入，可避免从高台边缘或墙侧接近 |
| `require_visibility` | 是否必须看见目标才算成功；移动目标通常需要 |
| `deadline_s` | 超过后转备用区域或最近接近点 |
| `success_mode` | 进入区域、看见目标、或 best-effort 最近点 |

### 3.2 建议的区域半径

| 目标类型 | 建议半径 |
|---|---:|
| 敌方基地/战场区域 | 1.5～2.0 m |
| 中央高地入口 | 0.8～1.2 m |
| 隧道入口 | 0.8～1.0 m |
| 巡逻/防守点 | 1.0 m |
| 敌方机器人（移动） | 由视觉/裁判系统跟踪，不按固定点到达 |
| 己方基地/复位点 | 0.15～0.5 m，可保留精确模式 |

精确点模式仍然保留，用于基地复位、狭窄对接等必须精确的场景；战场目标默认使用区域模式。

### 3.3 目标状态

规划器不要只判断“可达/不可达”，而应维护以下状态：

```text
UNKNOWN       从未观测到，或超出可靠感知范围
FREE          累计图已知可通行
VISIBLE       在传感器范围内，且视线可达
OCCLUDED      目标方向被已知障碍遮挡
BLOCKED       累计图确认目标格被占用
UNREACHABLE   超时或安全搜索确认无法到达
```

处理原则：

- `UNKNOWN` / `OCCLUDED` 不等于 `BLOCKED`；
- 只有累计图确认目标格长期占用时，才允许进入 ejection/失败逻辑；
- 目标在实时窗口外但累计图曾见过时，继续使用累计路线；
- 目标在未知区域时，先走向目标导向的观察点，不直接对未知点做精确搜索。

---

## 4. 战场 Zone Graph

### 4.1 节点

RM 场地固定，建议建立轻量语义拓扑，而不是全图 frontier 搜索。节点数量控制在 8～14 个，例如：

```text
own_base
own_highland_entry
own_tunnel_entry
own_defense_point
center_north_entry
center_south_entry
center_highland
enemy_highland_entry
enemy_tunnel_exit
enemy_base_approach
enemy_base_region
patrol_point
return_point
```

每个节点包含：

```yaml
zone_id: center_north_entry
center: [x, y]
radius: 1.0
approach_direction: south
terrain_mode: normal
min_clearance: 0.26
expected_traverse_s: 4.0
allow_reverse: false
fallback_zone: center_south_entry
```

### 4.2 边

边的属性至少包含：

- 预计通行时间；
- 最小净空；
- 是否坡道/高地/隧道；
- 是否单向；
- 是否允许倒车；
- 动态阻塞时的绕行代价。

全局规划使用累计图和 zone graph 的混合方式：

1. 当前 zone 到目标 zone 先在拓扑图上求路线；
2. 相邻 zone 之间用累计地图上的栅格路径补全；
3. 局部执行仍只接受实时 ROGMap 中连通的段；
4. 实时障碍导致某条边失效时，拓扑图暂时封锁该边，重新选路。

### 4.3 无图规则的处理

如果比赛规则允许标准场地尺寸和固定设施的先验，直接使用 zone graph 是收益最高的方案。

如果规则禁止任何场地先验：

- 不加载 `rmuc.pgm` / `rmul.pgm` 作为运行地图；
- 仍然维护在线拓扑节点（边界、坡道口、隧道口、高地入口）；
- 用在线检测和访问历史生成边；
- 不要退回“全字段 dense frontier”作为唯一策略。

仓库里已有 `rmuc.yaml` / `rmul.yaml`，可以离线用于 zone 坐标设计、仿真和回归测试；运行时是否加载必须由规则决定。

---

## 5. 目标导向探索

### 5.1 现有评分的问题

当前在线探索主要按到目标的欧氏距离、路径长度和方向惩罚评分。目标在墙后时，墙两侧的 frontier 都可能在某一帧“看起来更近”，导致机器人来回切换。

建议把评分改为：

```text
U(f) = w_goal   × Δd_goal(f)
     + w_info   × I_unknown(f)
     + w_vis    × V_goal(f)
     - w_path   × d_path(f)
     - w_revisit× R_visit(f)
     - w_switch× S_branch(f)
     - w_turn  × T_turn(f)
```

| 项 | 含义 |
|---|---|
| `Δd_goal` | 累计图上到目标区域的 geodesic 距离减少量，不用欧氏距离 |
| `I_unknown` | 从候选点可见的未知格/未知体素数量 |
| `V_goal` | 候选点对目标区域或目标附近未知区域的可见性增益 |
| `d_path` | 从当前位置到候选点的已知自由路径长度 |
| `R_visit` | 最近访问过的区域、走廊、失败区域的惩罚 |
| `S_branch` | 从当前绕障分支切换到另一侧的代价 |
| `T_turn` | 大角度转向和原地旋转的代价 |

在 RM 中，`w_revisit` 和 `w_switch` 应该比“覆盖更多未知区域”更重要。

### 5.2 绕障分支锁

当目标被高地、基地或墙体遮挡时：

1. 在累计图上找左右两条候选绕行分支；
2. 选代价较小的一侧；
3. 锁定该分支 2～3 m 或 8～10 s；
4. 只有以下情况允许切换：
   - 实时障碍使该分支不可通行；
   - 到达死路；
   - 目标进入视线；
   - 分支代价显著变差，例如超过 2 倍；
5. 切换后记录原因，避免下一帧又切回去。

这次运行中，目标在墙后，子目标在 `(3.48,2.97)`、`(3.78,-1.62)`、`(4.18,3.48)`、`(4.12,-2.92)` 等位置南北来回跳。分支锁可以直接消除这类大范围折返。

### 5.3 访问历史

建议在累计地图上增加：

- 每格最近访问时间；
- 每格访问次数；
- 失败区域和失败原因；
- 已完成观察点；
- 已确认死胡同边。

现有的“失败点 1.20 m 内 30 s 不重选”保留，但只作为底层保护。上层需要按走廊/区域惩罚，而不是只按点距离去重。

---

## 6. 目标不可见时的处理

### 6.1 目标超出可靠感知范围

增加传感器模型参数：

```yaml
sensor:
  max_range: 30.0
  min_range: 0.2
  horizontal_fov_deg: 360
  vertical_fov_deg: 60
```

目标距离大于 `max_range` 时：

- 状态设为 `UNKNOWN_FAR`；
- 不执行 ejection；
- 选择“减少目标距离 + 增加目标方向未知覆盖”的观察点；
- 进入可靠范围后再切换精确目标搜索。

RM 场地尺寸较小，LiDAR 通常能覆盖场地，但目标可能是低矮、黑色、反光或没有稳定回波的物体，不能假定一定有点云命中。

### 6.2 目标被遮挡

在累计图上做视线检查：

```text
候选点 → 目标区域采样 → 是否穿过已知占用格
```

- 被占用格遮挡：选择能绕到障碍端部的观察点；
- 只被未知格遮挡：选择能增加该方向覆盖的 frontier；
- 视线打开：切换精确接近或进入目标区域；
- 遮挡无法解除：按 deadline 转 best-effort。

### 6.3 目标完全扫不到

这是 RM 中最需要区分的情况：

| 情况 | 处理 |
|---|---|
| 目标是区域/基地，只要求到达 | 不要求 LiDAR 回波，按 GoalRegion 到达 |
| 目标是敌方机器人，要求跟踪 | 使用相机/裁判系统，LiDAR 只做障碍 |
| 目标是高地/隧道内的点 | 先导航到入口，再由区域 mode 接替 |
| 目标材质导致无回波 | 使用其他传感器或操作员确认，不能只靠点云 |
| 完全不可达 | deadline 后选择最近安全点或备用 zone |

如果比赛规则要求“看到目标”才得分，而目标本身没有稳定 LiDAR 回波，单靠点云方案无法可靠完成，需要视觉、反光标记或比赛系统信息。这一点应在方案评审时明确，不能靠调 frontier 参数解决。

### 6.4 观察动作

不要默认持续以 `+2 rad/s` 旋转。建议把观察做成显式状态：

```text
OBSERVE:
  停车或低速
  旋转一圈，约 1.0 rad/s
  更新累计图
  重新分类目标状态
  继续、返回或转 best-effort
```

触发观察的时机：

- 到达岔路口；
- 到达高地/隧道入口；
- 目标方向被遮挡且没有可用 frontier；
- 子目标连续失败；
- 到达目标区域但尚未看到目标。

---

## 7. 规划状态机

```text
IDLE
  │ 接收 GoalRegion
  ▼
CLASSIFY_GOAL
  ├─ FREE / VISIBLE ──────────► APPROACH_GOAL
  ├─ BLOCKED ─────────────────► APPROACH_OR_EJECT
  ├─ OCCLUDED ────────────────► OBSERVE_AROUND_OBSTACLE
  ├─ UNKNOWN / UNKNOWN_FAR ───► GOAL_DIRECTED_EXPLORE
  └─ UNREACHABLE ─────────────► CLOSEST_APPROACH / FALLBACK_ZONE

GOAL_DIRECTED_EXPLORE
  ├─ 找到目标视线 ────────────► APPROACH_GOAL
  ├─ 到达观察点 ──────────────► OBSERVE
  ├─ 分支锁超时 ──────────────► 重新选分支
  └─ deadline ────────────────► CLOSEST_APPROACH / FALLBACK_ZONE

APPROACH_GOAL
  ├─ 进入 GoalRegion ─────────► ARRIVED
  ├─ 需要精确复位 ────────────► PRECISE_DOCK
  └─ 超时/不可达 ─────────────► CLOSEST_APPROACH
```

任务层保留现有精确模式，仅对战术目标启用区域模式。这样不会破坏己方基地复位、狭窄对接等需要精度的行为。

---

## 8. 比赛时间与战术回退

导航不应只按目标距离优化，还要按比赛时钟：

```text
T-120 s：允许继续探索主要目标
T-60 s：放弃不可达目标，转防守/巡逻 zone
T-30 s：回己方基地或安全区
```

建议给每个目标设置：

```yaml
deadline_s: 60
fallback_zone: own_defense_point
return_zone: own_base
```

当目标不可达时：

1. 尝试备用 zone；
2. 仍不可达则返回安全区；
3. 记录 best-effort 最近点和原因；
4. 不在比赛末段继续做无收益探索。

如果允许接入比赛系统/裁判系统，应把比赛阶段、剩余时间和目标信息输入战术层，而不是只靠点云自主判断。

---

## 9. 代码落点

### 9.1 任务层

文件：

- `mas2027_nav_executor/src/task_manager/task_manager.cpp`
- `mas2027_nav_executor/src/path_planner/trajectory/minco_planner.cpp`

建议：

- `GoalRegion` 替换纯 `PoseStamped` 的到达判定；
- 增加 `zone_id`、`radius`、`deadline`、`success_mode`；
- 保留精确模式，用于基地/复位；
- `checkGoalReached` 支持区域判断；
- 目标在未知区时不提前 latch 为 invalid。

### 9.2 全局探索层

文件：

- `mas2027_nav_executor/src/path_planner/search/global_path_searcher.cpp`
- `mas2027_nav_executor/include/mas2027_nav_executor/path_planner/search/global_path_searcher.hpp`

建议：

- 当前 `remaining` 欧氏距离评分改为累计图 geodesic；
- 保留现有直接目标、累计路线和 frontier 回退；
- 增加 zone waypoint 选择和分支锁；
- 增加目标可见性检查；
- 日志输出每个候选的各项 utility，便于现场调参。

### 9.3 累计地图

文件：

- `mas2027_nav_executor` 的 `OnlineGlobalMap` 相关实现。

建议增加：

- 访问次数/最近访问时间层；
- 失败区域层；
- 目标方向未知覆盖统计；
- 可选的拓扑节点/边缓存。

### 9.4 配置

文件：

- `mas2027_nav_executor/config/planner_params.yaml`
- 新增 zone graph 配置文件，建议放在 `mas2027_nav_bringup/config/` 或执行器 config 下。

初始参数建议：

```yaml
exploration:
  goal_region_radius: 1.5
  goal_visibility_required: false
  goal_search_timeout_s: 120
  goal_direction_cone_deg: 90
  frontier_commit_time_s: 8.0
  frontier_commit_distance_m: 2.0
  frontier_switch_penalty: 0.8
  revisit_penalty: 2.0
  info_gain_weight: 0.5
  visibility_gain_weight: 1.0
```

这些只是起点，必须用现场 bag 回放和实车调参，不应直接作为比赛最终值。

### 9.5 测试

建议增加：

- 目标位于 L 形墙后，累计图未知；
- 目标超出滚动窗口但累计图曾有通路；
- 目标完全不可见，只能 best-effort；
- 目标在中央高地内，需要先到坡道入口；
- 目标被隧道遮挡，需要选择入口；
- 分支锁后不因单帧 frontier 变化而反向；
- 目标区域到达和精确模式切换；
- 动态对手阻断当前分支后的重选；
- 窄道 0.24 m 地板和硬安全门不回归。

现场回归指标：

```text
目标区域到达时间
路径长度 / 净位移比
子目标切换次数
非收益折返距离
原地旋转时间占比
最小净空
规划失败率
比赛剩余时间
```

验收建议：

- 目标 1 的局部探索时间从 43 s 降到 15～20 s；
- 10 m 目标推进的子目标切换不超过 2～3 次；
- 没有新信息时，不出现超过 5 m 的净收益为零的往返；
- 任何情况下不降低 0.24/0.26 m 硬净空门。

---

## 10. 分阶段实施

### P0：先解决“来回跳”和“精确点”

1. GoalRegion 与区域到达；
2. zone graph 雏形；
3. 累计图 geodesic 评分；
4. 绕障分支锁；
5. 比赛 deadline 和备用 zone。

这一版不依赖新传感器，能直接改善这次现场运行的 23.8 m 净零位移问题。

### P1：解决目标不可见

1. 累计图视线检查；
2. 目标方向未知覆盖评分；
3. 观察点和限时 360° 扫描；
4. 相机/裁判系统目标融合；
5. 移动目标预测拦截。

### P2：长期稳定性

1. 在线拓扑节点和边维护；
2. 长距离 LIO 漂移评估；
3. 回环/关键帧；
4. 动态对手预测和让行；
5. 多目标战术调度。

---

## 11. 明确不做的事

- 不把 `unknown_as_occupied` 改为 `false` 来“提速”；
- 不为探索方便降低 0.24 m 窄道地板或 0.26 m 普通净空；
- 不继续扩大纯 frontier 探索的权重；
- 不用 1.2 m 点去重替代分支级决策；
- 不假设 LiDAR 一定能扫到目标；
- 不把移动敌方机器人的旧坐标当成静态终点；
- 不在比赛末段继续无 deadline 的探索。

---

## 12. 现场数据前置条件

最近一次运行 `20261020_165514` 在停止录制后系统异常重启，导致运行目录中的日志、CSV 和 `bag/metadata.yaml` 为 0 字节；bag 本身可通过 `ros2 bag reindex -s mcap` 恢复，源日志和顶层 CSV 仍在。
后续现场验证必须：

1. 录制前 source 工作空间 overlay，确保 `/nav_executor/chassis_cmd`、`/opt_path`、`/nav_executor/region_status` 等自定义消息进入 bag；
2. 停止录制后执行 `sync`，确认日志、CSV、bag metadata 非空；
3. 尽量正常关机，避免直接断电；
4. 每次运行保留 `minco_perf_detailed`、`rog_map_perf_detailed`、`rog_map_perf_summary` 和完整日志。

否则“方案是否有效”无法用 bag 复盘。

---

## 13. 一句话总结

RM 无图导航的核心不是“探索得更多”，而是：

> **用规则允许的场地语义减少探索自由度，用区域目标容忍坐标误差，用分支锁消除来回折返，用实时局部安全门保证不撞。**

优先级建议：**GoalRegion + Zone Graph + 分支锁** 先落地，再叠加目标可见性和视觉/比赛系统融合。
