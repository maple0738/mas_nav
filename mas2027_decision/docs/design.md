# 决策层设计

## 1. 结构

```
/ Odometry ──┐
裁判数据 ────┤──> Observation ──> DecisionCore::select() ──> Intent
点表 ────────┘                        │
                                      └─> DecisionCore::step() ──> Goal ──> /goal_pose
```

- `DecisionCore` 不依赖 ROS，可单测、可离线回放；
- 节点只负责：取位姿、填 `Observation`、10 Hz 调 core、去重后发布。

## 2. 为什么是优先级链而不是状态转移表

传统 FSM 需要为"进攻 → 回血"、"巡逻 → 回防"等每一对状态写转移边，任务一多转移表就爆炸。
这里 `select()` 每帧重新判断：

```cpp
if (末段低血)        return RUSH_ENEMY_BASE;
if (回血 latch)      return HEAL;
if (基地受击事件)     return DEFEND_BASE;
if (弹药不足)        return HEAL;
if (敌方前哨站激活)   return ATTACK_OUTPOST;
if (己方前哨站需支援) return HELP_OUTPOST;
return PATROL;
```

**中断**天然成立：优先级一变，这一帧就换了意图。跨帧只保留两样东西：

| 持久状态 | 作用 |
|---|---|
| `heal_latched_` | 回血回差：0.30 进入、0.60 退出，避免阈值抖动 |
| `base_under_attack_` + 计时 | 基地事件保持 5 s，避免一帧噪声导致来回切 |

## 3. 与导航的接口

只发 `geometry_msgs/PoseStamped` 到 `/goal_pose`：

- `acceptGoal` 会校验地形/ROG/odom 新鲜度，并 TF 到 `odom`；
- 同目标（≤ 0.15 m）不足以触发重规划，但会让执行器短暂停顿约 0.2 s；
- 所以决策层必须去重：意图变化、目标移动 > 0.3 m、或超过 3 s 才重发。

## 4. 点表与坐标系

- 地图锚定模式：`goal_frame = map`，点表是地图坐标，用 `tools/teach_points.py` 示教；
- 纯无图模式：`goal_frame = odom`，点表是相对起点的坐标，`StartAnchor` 只做坐标换算；
- 红蓝方：`mirror_points = true` 时按 `y -> -y`、`yaw -> -yaw` 镜像。

## 5. 扩展点

- **新意图**：在 `Intent` 加枚举 → `select()` 加一条优先级 → `step()` 加一个分支；
- **需要并行/更深嵌套**时再转行为树：保持 `Observation` / `Goal` 不变，只替换 `select()` 内部；
- **裁判数据**：在节点里加订阅，填 `Observation`，并置 `referee_valid = true`；
  `debug_*` 参数退化为初始值，调试注入仍可用。

## 6. 已知限制

- `nav_executor` 没有任务状态反馈，无法判断"是否到达/规划失败"；
- 目前没有"到达后触发下一步"的逻辑，巡逻用时间切换；
- 基地受击检测依赖 `ally_base_hp`，裁判数据没接通前只能手动注入。
