# debug/ 目录约定

这里放**不进产品代码**的现场材料：录包、日志、CSV、截图、分析笔记。
大文件已在 `.gitignore` 里排除，不要提交进仓库。

## 目录

| 目录 | 放什么 |
|---|---|
| `notes/` | 每次现场调试的结论笔记（推荐一段运行一个 `YYYYMMDD_HHMMSS_场景名.md`） |
| `logs/` | 节点日志、`ros2 topic echo` 输出、诊断行 |
| `bags/` | 现场 rosbag（`*.db3` / `*.mcap` / `*.bag`） |
| `csv/` | 决策/导航导出的时间序列 |
| `plots/` | 由 `csv/` 生成的图（轨迹、意图时间线、血量曲线） |

## 推荐流程

1. 每次运行新建一个统一前缀：`20261023_153000_半场巡逻`。
2. 同时保存：bag（至少 `/Odometry`、`/goal_pose`、`/cmd_vel`、`/debug/decision_intent`）、
   `ros2 param dump /mas2027_decision`、点表 `points.yaml`。
3. 在 `notes/<前缀>.md` 里写：日期、场地、点表版本、参数、期望行为、实际行为、结论。
4. 需要复现时，用 bag 回放 + 同一点表 + 同一份参数 dump。

## 调试用命令

```bash
# 注入裁判数据（裁判系统接通前）
ros2 param set /mas2027_decision debug_hp_ratio 0.25
ros2 param set /mas2027_decision debug_game_time_s 310.0
ros2 param set /mas2027_decision debug_ally_base_hp 4000.0
ros2 param set /mas2027_decision debug_enemy_outpost_active true

# 观察意图与目标
ros2 topic echo /debug/decision_intent
ros2 topic echo /goal_pose
```

注意：调试参数在节点每次 tick 都会重新读取，改完立即可见，不需要重启。
