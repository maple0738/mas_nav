# 假决策联调步骤

目标：不接裁判系统，用固定脚本验证「决策层发目标 -> 导航执行 -> 到达 -> 下一个点」
整条链路，以及低血返回 home / 回血恢复巡逻的分支。

脚本节点是 `mas2027_decision_test_node`，和正式节点 `mas2027_decision_node` 完全独立，
不参与实战。

## 1. 编译并 source

```bash
colcon build --packages-select mas2027_decision --symlink-install
source install/setup.bash
```

`--symlink-install` 下修改源码里的 `config/test_points.yaml` 会直接生效；
不加这个参数时，示教请指向 `install/mas2027_decision/share/mas2027_decision/config/test_points.yaml`，
或者改完源码后重新 `colcon build`。

## 2. 示教测试点

先把车摆到 `home`，启动示教：

```bash
ros2 run mas2027_decision teach_points.py \
  --file ~/gyx_nav/mas_nav/mas2027_decision/config/test_points.yaml \
  --parent map
```

依次输入 `home`、`a1`、`a2`、`a3`，每次摆好车再回车。
同名点会覆盖原坐标，新名字会追加；其他键保持不变。

`--parent` 要和决策的 `goal_frame` 一致：有先验地图用 `map`，纯无图用 `odom`。

## 3. 启动顺序

终端 A：导航（`nav_executor`、定位、TF），保持和平时一样。
终端 B：

```bash
ros2 launch mas2027_decision decision_test.launch.py
```

常用覆盖参数：

```bash
# 纯无图
ros2 launch mas2027_decision decision_test.launch.py goal_frame:=odom
# 改时序
ros2 launch mas2027_decision decision_test.launch.py start_delay_s:=3.0 dwell_s:=8.0
# 用别的点表
ros2 launch mas2027_decision decision_test.launch.py points_file:=/abs/path/test_points.yaml
```

## 4. 预期时间轴

| 时间 | 阶段 | 行为 |
| --- | --- | --- |
| 0-5 s | `WAIT_START` | 不发目标，原地不动 |
| 5 s | `GO` | 发 `a1` 目标 |
| 到达 a1 | `DWELL` | 停 5 s |
| +5 s | `GO` | 发 `a2` 目标 |
| ... | | a3 -> a1 循环 |

观察：

```bash
ros2 topic echo /debug/test_state        # WAIT_START / GO / DWELL / HEAL
ros2 topic echo /goal_pose
ros2 topic hz /Odometry
```

## 5. 掉血 / 回血

```bash
# 触发返回 home
ros2 param set /mas2027_decision_test debug_hp_ratio 0.2
# 恢复巡逻（回到掉血前正在去的巡逻点）
ros2 param set /mas2027_decision_test debug_hp_ratio 0.7
```

阈值来自 `heal_enter`（0.30）/ `heal_exit`（0.60），可在 launch 或 YAML 里改。

## 6. 常见问题

- **`/goal_pose` 一直没消息**：确认 `enabled=true`；确认 `points_file` 能读到且包含
  `home` 和至少一个巡逻点；`goal_frame != odom` 时必须有 `map -> odom` TF。
  节点启动日志会打印 `test decision up: enabled=... points=...`。
- **到了却不切下一个点**：到达判定是「当前位姿到目标点距离 <= `arrive_tolerance_m`」，
  默认 0.4 m。地图漂移大时可适当放大，或改用 `goal_frame:=odom` 验证链路。
- **每 3 秒顿一下**：`/goal_pose` 的兜底重发会重新提交目标，导航会短暂清空运动许可。
  调大 `goal_republish_s` 可减少重发。
- **60 秒还没到**：`arrive_timeout_s` 触发后会跳过该点并打 WARN，方便继续测后面的点。
