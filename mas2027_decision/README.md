# mas2027_decision

哨兵决策层：把比赛状态翻译成导航目标，只通过 `/goal_pose` 与 `mas2027_nav_executor` 交互。

当前实现是**优先级链 + 每意图持久状态**的状态机（参考 `WUST-RM/awakening` 的 `sentry_brain`），
不引入行为树。设计说明见 `docs/design.md`，调参见 `docs/tuning.md`。

## 目录

```
mas2027_decision/
├── include/mas2027_decision/
│   ├── types.hpp            # Vec2 / Observation / Intent / Goal
│   ├── decision_params.hpp  # 阈值参数 + 点表
│   ├── decision_core.hpp    # 纯逻辑决策核心（无 ROS）
│   ├── point_table.hpp      # 点表读取 / 镜像
│   └── start_anchor.hpp     # 纯无图模式的起点锚定
├── src/
│   ├── decision_core.cpp
│   ├── point_table.cpp
│   ├── decision_node.cpp    # ROS 节点：odom -> 决策 -> /goal_pose
│   └── test_decision_node.cpp  # 假决策脚本节点（只用于联调）
├── config/
│   ├── decision.yaml        # 阈值与话题
│   ├── points.yaml          # 示教点表
│   ├── test_decision.yaml   # 假决策测试参数
│   └── test_points.yaml     # 假决策测试点表（home / a1 / a2 / a3）
├── launch/
│   ├── decision.launch.py
│   └── decision_test.launch.py
├── test/test_decision_core.cpp
├── tools/teach_points.py    # 实车示教点表
├── docs/                    # 设计与调参说明
└── debug/                   # 现场录包/日志/笔记（不入库，见其 README）
```

## 构建

```bash
colcon build --packages-select mas2027_decision --symlink-install
source install/setup.bash
```

## 运行

```bash
ros2 launch mas2027_decision decision.launch.py \
  enabled:=true goal_frame:=map
```

- `enabled:=false`（默认）时节点只订阅、不发布目标，用于观察。
- `goal_frame`：`map` = 地图锚定模式；`odom` = 纯无图模式。
- `mirror_points:=true`：点表按场地中线镜像，红蓝方复用。

## 示教点表

```bash
ros2 run mas2027_decision teach_points.py --file <points.yaml> --parent map
```

把车摆到目标位置，输入点名（`home`、`heal`、`patrol_a` ...），回车即写入。
同名点覆盖原值，新名字追加到文件末尾；未修改的键保留。

## 假决策联调

不接裁判数据，按固定脚本走一遍 `决策 -> /goal_pose -> 导航` 链路：

```bash
# 1. 先起导航（nav_executor / 定位 / TF）
# 2. 再起测试决策
ros2 launch mas2027_decision decision_test.launch.py
```

默认流程：原地等 5 s -> `a1` 停 5 s -> `a2` 停 5 s -> `a3` 停 5 s -> 回 `a1` 循环。
点表来自 `config/test_points.yaml`，用示教工具改写：

```bash
ros2 run mas2027_decision teach_points.py \
  --file <源码目录>/mas2027_decision/config/test_points.yaml --parent map
```

模拟掉血 / 回血：

```bash
ros2 param set /mas2027_decision_test debug_hp_ratio 0.2   # 回 home
ros2 param set /mas2027_decision_test debug_hp_ratio 0.7   # 恢复巡逻
ros2 topic echo /debug/test_state                          # 观察当前阶段
```

说明：测试节点用"当前位姿到目标点的距离"判断到达（默认 0.4 m），
因为 `nav_executor` 没有任务状态话题；单点超时（默认 60 s）会跳过该点。
详细步骤见 `docs/test_playbook.md`。

## 调试

```bash
ros2 param set /mas2027_decision debug_hp_ratio 0.25
ros2 param set /mas2027_decision debug_game_time_s 310.0
ros2 topic echo /debug/decision_intent
```

更多约定见 `debug/README.md`。

## 测试

```bash
colcon test --packages-select mas2027_decision
colcon test-result --verbose
```

`decision_core` 不依赖 ROS，可用任意 gtest 环境单独跑。

## 已知缺口

- 裁判数据（血量、弹药、基地血量、比赛时间）尚未接入，目前由 `debug_*` 参数注入。
- `nav_executor` 没有任务反馈，决策层只能用固定驻留时间、不能确认"到达/失败"。
  后续建议增加 `/nav_executor/mission_status`。
