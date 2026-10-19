# 窄道碰撞排查与调参

本文适用于 `mas2027_nav_executor` 使用 ROGMap 在线距离场通过窄道时的实车排查。每次只改一组参数，保存对应的配置快照与录包；发生实体接触后先停车，确认碰撞阶段和地图证据，再进行下一次低速验证。

## 当前几何与通过边界

- 已确认车体为正方形，中心到角点的旋转半径为 `0.302 m`，对齐直行的半宽约为 `0.302 / √2 = 0.214 m`。`planner.corridor.swept_radius` 记录前者，不能把它直接设为整条直行轨迹的中心净空门槛。
- `20261018_130912` 的同一帧 ROGMap 距离场中，起点到目标按 `0.25 m` 最低净空搜索时连通，按 `0.26 m` 时断开；瓶颈约在 `(2.54,-1.24)`，读数约 `0.25 m`。因此统一要求 `0.30 m` 会在地图上判无路。`20261018_135419` 的历史版本用 `0.27/0.25 m` 主/回退搜索、`0.24 m` 窄道地板完成 4/4 目标；该记录不验证后来修改的版本。
- ROGMap 距离场到占据**格心**，栅格为 `0.05 m`。若墙边界沿栅格轴向且车体完全对齐，`0.25 - 0.025 - 0.214 ≈ 0.011 m` 是一侧的名义间隙。实际墙面、定位偏差和航向偏差未包含在内。
- 车体在偏角 `θ` 下，朝直墙的半宽近似为 `0.214 × (|cos θ| + |sin θ|)`。`5°` 时约 `0.231 m`，已超过上面粗算的 `0.225 m` 格心至墙边界距离。因此当前 `ready_tolerance_deg=5`、`ready_release_deg=8` 不能作为窄道安全保证。

## 先判断碰撞发生在哪一阶段

| 现场现象 | 优先核对 | 调整位置与试验值 | 影响和边界 |
|---|---|---|---|
| 入窄道时车体仍在转、角点擦墙 | `/nav_executor/region_status` 的 `gimbal_heading_error_deg`、`gimbal_heading_ready`；`/nav_executor/chassis_cmd` 的 `follow_gimbal`、`vx/vy`、`small_yaw_ref`。确认实际车体航向，而不只看角度命令。 | `mas2027_nav_executor/config/region_control.yaml`：`gimbal_follow.ready_tolerance_deg` 先由 `5` 试到 `1`，`ready_release_deg` 由 `8` 试到 `2`，`settle_time_s` 由 `0.3` 试到 `0.5`。若对齐启动太晚，再检查 `planner_params.yaml` 的 `narrow_passage_alignment_lookahead=2.0 m` 与实际可用的入口空间。 | 更严格的对齐会使车在入口等待；若航向噪声或下位机响应无法达到要求，应查反馈链路，不要仅靠放宽角度让车起步。`heading_ready` 基于 LIO 航向，不是下位机到位确认。 |
| 航向已稳定，沿墙直行时车侧擦墙 | 同时对齐 `/Odometry`、`/opt_path`、`/rog_map/field` 与 `/rog_map/layer_value`，看实车相对轨迹的横向偏差，以及碰撞位置的地图净空。 | `planner_params.yaml`：`minco_optimizer.narrow_passage_speed` 先由 `0.5` 试到 `0.2 m/s`。若最终轨迹本身贴墙，再将 `narrow_passage_clearance` 由 `0.24` 试到 `0.25 m`。 | 降速有助于减少跟踪偏差，但不会改变静态间隙；提高地板会拒绝低于 `0.25 m` 的轨迹。历史地图在 `0.26 m` 已断开，继续提高可能只能停车。`safe_dist=0.33` 是软目标，不能代替硬门。 |
| 到达目标或离开窄道时开始自转并擦墙 | 核对 `Chassis alignment transition`、`follow_gimbal` 切换瞬间的位置和 ROGMap 净空；确认是否已离开狭窄区。 | `planner_params.yaml`：`corridor.extra_margin` 可由 `0.05` 试到 `0.07 m`，使自由旋转释放更晚。 | 它作用于旋转空间要求，不会为直行轨迹增加侧向间隙。保持实测 `swept_radius=0.302 m`，不要靠改小半径换取通过。 |
| 车已在窄道内且航向不正，需要原地转向 | 检查当前位置是否具备旋转外廓所需空间。 | 停止当前自动尝试，从可旋转的位置重新对齐后再进入。 | 这个场景不能靠降低 `narrow_passage_clearance` 解决；窄道直行可通过不代表能原地旋转。 |
| 实车撞到墙，但二维地图将该墙显示为自由或净空明显偏大 | 对比 `/cloud_registered_full`、`/rog_map/raw_occupied`、`/rog_map/layer_value`、`/rog_map/field`，并核对点云与里程计时间；先判断是点云未观测、姿态错位还是地图过滤。 | `planner_params.yaml` 中 `projection.body_exclusion_radius=0.302 m` 是入图前自车回波过滤轮廓；设 `0` 仅作为重新建图的对照实验。 | 设 `0` 可能引入自车回波造成误停，不能把它当作常规通行调参。若观测墙被漏掉，先修地图/时序，继续提高轨迹门槛也无法可靠判断真实实体间隙。 |

上述角度和速度是**排查试验值**，不是已通过实车认证的安全设置。若收紧到约 `1°`、降到 `0.2 m/s` 后仍擦墙，就要量测实体间隙、轨迹跟踪偏差和地图/定位误差；在约 `0.25 m` 的地图瓶颈上继续降低净空门槛不能制造物理空间。

## 哪些参数不要混着调

- `planner.smac_2d.minimum_clearance=0.27 m` 是主搜索门槛；`relaxed_minimum_clearance=0.25 m` 在主搜索无解时提供参考线。提高回退门槛到 `0.26 m`，历史那张地图会失去连通性。
- `minco_optimizer.collision_dist=0.28 m` 对应普通轨迹约 `0.26 m` 的有效硬门；窄道续行可使用独立的 `narrow_passage_clearance=0.24 m`。只提高 `collision_dist`，不能等同于提高窄道绝对地板。
- `corridor.robot_radius=0.28 m` 用于安全走廊形状；`corridor.swept_radius=0.302 m` 是实测旋转外廓；`corridor.extra_margin=0.05 m` 作用于旋转释放。三者不互相替代。
- `narrow_passage_alignment_hold_distance=2.0 m` 是对齐模式的最短保持里程，不是撞墙时的直行净空门槛。
- 配置文件不是运行时参数的唯一来源：`mas2027_nav_bringup/launch/nav_executor_launch.py` 在 `online` 模式会覆盖 YAML，将 `planner.exploration.unknown_as_occupied` 和 `planner.rog_map.projection.unknown_as_occupied` 都设为 `true`。此时 ROGMap 仍会针对 `ray_range` 下限 `0.3 m` 的三维近距盲区执行强制 free 更新；机器人附近显示为 free 的格需要结合原始点云核实，不能只看 YAML 或二维净空。以本次运行抓取的 `params_planner.yaml`、`params_executor.yaml` 和录包为准。

## 一次可复核的现场试验

1. 先用 [现场导航运行记录](field_run_recording.md) 启动 `record_nav_diagnostics.sh`，再按原方式启动导航。记录器保存 bag、配置快照、日志和性能 CSV。
2. 只选择一处可重复的窄道，记录入口位置、行驶方向、墙体实体间距和具体接触点。第一次按当前配置低速观察；若发生接触，停车并保存该次 `RUN_DIR`。
3. 在同一时间轴上核对 `/Odometry`、`/opt_path`、`/nav_executor/chassis_cmd`、`/nav_executor/region_status`、`/rog_map/field` 和点云。先分类为“转向擦碰”“直行贴墙”“退出时自转”或“地图漏墙”，再按上表只改对应的一组参数。
4. 修改 YAML 后重启导航；同时核对记录器抓取的 `params_planner.yaml`、`params_executor.yaml`，确认 launch 覆盖后的实际参数。对比改动前后：最小地图净空、最大航向误差、最大横向跟踪偏差、是否有实体接触、是否出现轨迹拒绝或持续停车。只有重复通过且实体间隙经量测仍足够，才可认为该路段完成标定。
