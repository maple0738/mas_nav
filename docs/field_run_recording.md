# 现场导航运行记录

下一次实车复跑使用 `record_nav_diagnostics.sh` 收一份可离线分析的完整证据。记录器只订阅现有话题，不参与规划或控制，不改变导航行为。

## 启动

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

## 记录内容

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

## 容量与选项

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

## 分析入口

跑完后把 `RUN_DIR` 告诉分析侧即可。建议优先看：

- `Region status transition`：平地是否切入 `label=5/6`，以及切入位置
- `open set exhausted before goal` 中 `iterations>1` 的记录：全局搜索真实耗尽
- `bag_info.txt`：确认 `/rog_map/*`、累计地图和 `/cloud_registered_full` 均有消息
- `csv/rog_map_perf_detailed-*.csv` 的 `terrain_label_flat/slope/tunnel` 与分类计数
- 失败时刻附近的 `/rog_map/layer_type`、`/rog_map/layer_height_delta`、`/nav_executor/online_terrain_map`

## 注意

- 记录器应比导航启动得早；`transient_local` 话题即使晚订阅也可能只拿到之后发布的更新。
- `Ctrl-C` 后请等脚本打印 `Recorded run: ...` 再关闭终端，避免 MCAP 索引未落盘。
- 该脚本不启动、不重启、不配置导航节点；原有 launch 参数和安全逻辑不变。
