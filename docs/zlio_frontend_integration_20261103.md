# ZLIO 前端接入与 CPU 对比（2026-11-03）

**后续审计更正：此前实包测试将加速度按 SI 输入，但 Small Point-LIO 的加速度饱和阈值未同步换算，可能触发不必要的最短批次；其中 Small Point-LIO 的 CPU 数值不应作为正常饱和门配置下的结论。当前已在测试专用配置中修正，重新对比见 [自适应前后](small_lio_adaptation_comparison_20261103.md)。**

**历史记录：ZLIO 前端及其专用接口已按用户要求从 active 删除，本文中的 ZLIO 启动/构建命令不再适用。当前仅使用带时间、空间自适应的 Small Point-LIO。**

后续已使用用户提供的实车录包继续测试，见 [实车录包 CPU 对比](realbag_lio_cpu_20261103.md)。下文保留当时的合成场景结果。

## 接入结果

接入时曾将主导航默认前端改为 `mas2027_zlio`；现按用户要求恢复 Small Point-LIO 并启用时间/空间自适应。
ZLIO 后续已删除，不再作为可选启动项。保留现有双 MID360 驱动、`odom_localizer`、
`tf_maintainer`、ROGMap 和规划控制。完整 ZLIO 前端来自本地 z-localization 提交
`76e126328b670adc92981d9b4900cc059a6f2343`；核心、原生输入工作线程、高频预测、
初始化、连续轨迹去畸变、概率平面地图、相关观测处理及时间/空间自适应均保留。
没有引入 Z-Localization 后端。第三方代码和许可证保存在包内，原文件校验信息见 UPSTREAM.json。

本地包为 `mas2027_perception/Odometry/mas2027_zlio` 和
`third_party/localization_interfaces`；不再运行 Small Point-LIO 时，它仍可独立回退。

```bash
# source 维护环境后，再 source 当前 active/install/local_setup.bash
ros2 launch mas2027_nav_bringup nav_executor_launch.py lio_frontend:=zlio use_foxglove:=false
# 只回退 LIO，其他组件不切换
ros2 launch mas2027_nav_bringup nav_executor_launch.py lio_frontend:=small_point_lio use_foxglove:=false
```

## 接口适配

- 读取现有双雷达融合后的 PointCloud2，`timestamp` 是 FLOAT64 绝对秒；没有再将两个雷达重复融合。
- 话题、雷达到 IMU 外参、车体偏心盲区和 IMU 标度来自已有 `small_point_lio_params.yaml`。
  原始加速度按 `9.81 / norm(gravity配置)` 转为 SI，保留现有实测标度校正。
  原生估计节点和适配节点使用相同外参覆盖，避免重复维护标定。
- ZLIO 的内部 `zlio_odom / zlio_imu` 消息经过适配：姿态/位置移到 base_link 原点，
  速度含杆臂补偿，位姿和速度协方差同步变换。首次输出固定 odom 的初始位置与航向。
- `/Odometry` 为 odom 中的 base_link 位姿、base_link 轴下的 twist，上限 100 Hz。
- `output_resolution=-1` 导出所有有效的新去畸变回波，而非用定位降采样点冒充完整点云。
  适配节点将回波转换到 odom 并按约 50 ms 累积发布 `/cloud_registered_full` 及兼容话题
  `/cloud_registered`；未发布的短窗口回波会累积，避免节流时丢掉中间窗口。
- 动态 TF 仍由 tf_maintainer 唯一发布。ZLIO 的原生后验、预测和 session 话题仍可观测。
- active 的 ZLIO 观测间隔为 10–50 ms，空间采样使用完整原生反馈控制；这些配置与
  Small Point-LIO 的 1–5 ms 批量时间适应不同，并非用相同观测频率运行两个算法。
- 输入故障、reset 请求或已建立输出后的 session 换代会锁存停止兼容输出，避免新 odom 原点
  被旧全局定位误用。现有全局定位器没有实现 ZLIO 的后端协调重置；此时需重启整套导航。
  保留原生 reset 服务不等于已经实现 active 导航中的无缝恢复。
- 可选 Foxglove 关闭时不再提前解析其安装路径，本机未安装该包也能构造关闭它的启动配置。

## 同输入 ROS 进程基准

没有找到包含原始 LiDAR+IMU 的实车录包。已有录包主要为仿真导航遥测，不能作为
完整 LIO 回放。本次采用确定性的三平面合成房间，不将其描述为实车或 Gazebo IMU 测试。

每帧 5043 点、20 Hz，逐点时间覆盖 50 ms；IMU 200 Hz。两算法接收同一组 float XYZ
和逐点时间。原始 IMU 比力为 1 g，合成标定使用恒等雷达/IMU/车体外参，两边均换算到
9.81 m/s²。各算法保留自己的地图、滤波、采样和线程设置。每次输入 12 s，前 5 s
作为预热，后 7 s 计时；每场景交替运行各算法 3 次，表格取中位数。

CPU 使用前端工作进程的 user+system 时间 / 真实经过时间，100% 表示占满一个逻辑核。
ZLIO 统计原生节点和兼容节点之和；Small Point-LIO 统计其单节点。发布器、静态 TF、
导航、ROGMap、全局定位和可视化不计入。OMP_NUM_THREADS=2，OMP_WAIT_POLICY=PASSIVE。
内存为各工作进程 RSS 之和的采样峰值，可能重复计算共享页，不等于独占物理内存。
主机、二进制和脚本校验值记录在本地 `tested-elf-provenance.json`。

| 场景 / 指标 | Small Point-LIO（时空自适应） | ZLIO＋适配节点 |
|---|---:|---:|
| 静止：CPU / 单核 | 5.57% | 14.14% |
| 静止：CPU 3 次范围 | 5.57–5.71% | 14.00–14.14% |
| 静止：合计峰值 RSS | 37.15 MiB | 76.15 MiB |
| 静止：里程计频率 | 80.43 Hz | 100.00 Hz |
| 静止：里程计延迟 P95 | 36.96 ms | 1.02 ms |
| 静止：完整点云延迟 P95 | 53.02 ms | 5.17 ms |
| 旋转：CPU / 单核 | 5.86% | 13.86% |
| 旋转：CPU 3 次范围 | 5.86–6.00% | 13.71–14.00% |
| 旋转：合计峰值 RSS | 37.07 MiB | 76.23 MiB |
| 旋转：里程计频率 | 78.28 Hz | 100.00 Hz |
| 旋转：里程计延迟 P95 | 45.43 ms | 1.00 ms |
| 旋转：完整点云延迟 P95 | 53.17 ms | 5.14 ms |

旋转场景前 3 s 静止，之后以 1 rad/s 绕竖直轴旋转；每个点按自身时刻的真实姿态生成，
IMU 与此一致。两种算法均完成初始化并持续输出有限位姿、正确坐标系及约 20 Hz 完整点云。
ZLIO 每次有 214 个接受激光更新的窗口、约 37 万次有效匹配；Small Point-LIO 静止输入的
独立核心健康复核在 15 s 中有 205601 个接受匹配，避免把仅 IMU 输出当作有效估计。
直接核心复核不用于表格 CPU 数值；表格均来自 ROS 节点测量。旋转期间最大位置漂移约
Small Point-LIO 3.52 mm、ZLIO 0.335 mm，仅作合成场景健康检查，不能代表实车精度。

本场景中 ZLIO CPU 约为 Small Point-LIO 的 2.4–2.5 倍，内存也更高。
它的优势在于独立高频预测、输出时效及完整的估计模型；不能宣称这次替换降低 CPU。
两边实际里程计输出频率不同，表格也不能视为完全等价输出下的算法速度排名。
仍需实车 LiDAR+IMU 录包验证真实 CPU、初始化成功率、高动态/遮挡/坡道表现及定位精度。

## 验证和复跑

- Release 构建并安装；ZLIO 9 组共 59 项测试、Small Point-LIO 4 组共 16 项功能测试通过。
  包括原生初始化/估计/地图/输入/消息适配，以及完整回波逐窗口无丢失、杆臂速度与协方差变换。
- 两种前端启动分支均通过实际 launch 对象构造检查；ROS 基准实际启动两种节点并验证输出。
  没有启动实车驱动、串口或整机导航，没有将此前真值里程计仿真称作本次完整 LIO 验收。
- ROS 参数、Python 编译和 git diff --check 检查通过。性能采集后的 C++ 格式整理没有改变算法；
  格式整理后重新构建、安装并重跑上述功能测试。

```bash
source /opt/ros/jazzy/setup.bash
source /home/jsy/Projects/environment/build/ros/jazzy/navigation/active/mas_nav/install/setup.bash
source /home/jsy/Projects/navigation/active/mas_nav/install/local_setup.bash
cd /home/jsy/Projects/navigation/active/mas_nav
BENCH_TAG=stationary python3 tools/benchmark_lio_cpu.py
BENCH_TAG=rotating BENCH_YAW_RATE=1.0 python3 tools/benchmark_lio_cpu.py
```

脚本要求 psutil、numpy、PyYAML 和 ROS Python 包；结果默认保存在 `.scratch/lio-cpu-benchmark/`。
本次逐次 JSON、节点日志、汇总和校验信息位于 `.scratch/zlio-integration-20261103/` 的
`final-static/`、`final-rotating/`、`summary.json` 和 `tested-elf-provenance.json`。
