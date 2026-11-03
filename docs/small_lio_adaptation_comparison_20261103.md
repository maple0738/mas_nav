# Small Point-LIO 时间、空间自适应前后对比（2026-11-03）

**最终固化状态：时间、空间自适应已固定启用，两个 enabled 参数及基准模式切换已删除。本文前后对比记录对应删除开关之前的同一二进制；原始结果留存，以下旧变体命令仅作历史实验说明。当前录包脚本只测试固化后的自适应前端。**

## 当前状态

active 已删除接入的 `mas2027_zlio` 前端、专用 `localization_interfaces`、本地构建/安装
产物以及 ZLIO 启动分支和依赖。主导航仅启动 Small Point-LIO。生产配置仍同时启用
`adaptive_sampling.enabled` 与 `adaptive_timing.enabled`，不再有 `lio_frontend` 选择参数。
ZLIO 相关历史报告和测量证据保留并标注已失效的启动命令，参考组件仓库未纳入此次删除。

## 方法与单位修正

同一 Small Point-LIO 二进制，通过两个开关做特性对比，避免其他源码/标定变化混入：

- baseline：固定 0.5 m 体素、固定 5 ms 批量调度；
- spatial：仅开启空间采样反馈；
- temporal：仅开启时间调度反馈；
- adaptive：两者同时开启（当前生产配置）。

使用用户提供的 462.858 s Livox 原始录包，6942 帧、92578 IMU。两边均收到同样的
90,588,646 个统一回波筛选点，使用真实逐点时间、相同外参假定和 0.38–100 m 范围。
可靠输入和统一 4000/200 队列仅为基准覆盖设置；传输完整性按接收计数逐项核对。
工作进程 CPU 包含点云处理、滤波和输出，转换/发布器及导航其他模块不计入。

**后续审计发现，先前实包基准将加速度按 SI 输入，却未将饱和阈值从 g 换算为 SI。**
原 3g 配置在该测试中被误用为 3 m/s²：92578 条 IMU 中有 92573 条会被误判饱和，
可导致时间调度常态使用最短批次。修正为 3×9.81 m/s² 后仅 4 条达到阈值。
此前实包的 Small Point-LIO CPU 不能作为正确饱和门下的结论，本报告全部采用重跑结果。
修正仅发生在测试专用配置；生产 g 单位输入及其 3g 参数保持原值。
新增配置回归验证 SI 阈值和各变体仅改变两个开关，避免再次混用单位。

## 完整录包：关闭与同时开启

每个变体完整回放一次，**4× 速率**、前 20 记录秒为预热。100% 表示一个逻辑核；
4× 占用不能直接当作正常 1× CPU，也不与旧的 2× 测试混比。

| 指标 | baseline | adaptive |
|---|---:|---:|
| 稳态 CPU / 单核 | 30.46% | 34.54% |
| 总 CPU 秒（含启动处理） | 35.14 s | 39.79 s |
| 峰值采样 RSS | 41.35 MiB | 40.66 MiB |
| 批次总数 | 114047 | 140278 |
| 接受匹配点总数 | 2231982 | 4281959 |
| 里程计输出 / 记录秒 | 79.00 Hz | 82.82 Hz |
| 完整点云输出 / 记录秒 | 15.00 Hz | 15.00 Hz |
| 完整点云输出点数 | 36753875 | 36753875 |
| 回放消息年龄 P95 / 墙钟 | 23.70 ms | 23.73 ms |
| 末尾输出落后 / 记录时间 | 80.27 ms | 74.70 ms |

该录包中，两项同时开启使稳态 CPU **增加 13.4%**，批次增加 **23.0%**，接受匹配点
增加 **91.8%**。内存基本持平，完整点云数量相同，输出时效变化很小。因此这套配置
主要增加定位观测和更新量，不能宣称节省 CPU；匹配点更多也不能直接等同于精度更高。

两个变体都通过完整接收、有效匹配、有限位姿、末尾跟随检查。两个轨迹按同一时间
插值、略过启动段后，位置差的 RMS 约 0.128 m、P95 0.205 m、最大 0.289 m；终点差
约 0.0166 m。这是算法输出之间的差异，**没有外部真值，不能解释为精度改善或恶化**。
轨迹图和原始轨迹保存在结果目录的 `trajectory_comparison.png` 和各运行 `trajectory.json`。

## 正常速率短段：分项检查

前 30 s、1×、预热 10 s；baseline / spatial / temporal / adaptive 各一次。
四种模式均收到相同的 449 帧、6001 条 IMU、5862747 个筛选点，并通过健康检查。

| 变体 | 稳态 CPU / 单核 | 峰值 RSS | 批次 | 接受匹配 | 回放年龄 P95 |
|---|---:|---:|---:|---:|---:|
| 关闭两项 | 8.31% | 38.55 MiB | 6893 | 110068 | 84.65 ms |
| 仅空间 | 9.32% | 38.35 MiB | 8362 | 236722 | 84.56 ms |
| 仅时间 | 8.51% | 38.70 MiB | 6987 | 110388 | 84.04 ms |
| 两项开启 | 9.17% | 38.21 MiB | 8443 | 237967 | 84.11 ms |

本短段空间自适应增加了可用定位观测，主要开销变化也与此相关；仅时间适应的变化较小。
空间采样目标会根据场景调整，本包近场几何下反而保留更多点，不能泛化成“自适应一定减少点数”。
两项开启比仅空间的 CPU 略低，但每项只运行一次、差距很小，不能当作稳定优势。
这些短段不替代完整录包或重复统计基准。

## 复跑与验证

```bash
source /opt/ros/jazzy/setup.bash
source /home/jsy/Projects/environment/build/ros/jazzy/navigation/active/mas_nav/install/setup.bash
source /home/jsy/Projects/navigation/active/mas_nav/install/local_setup.bash
cd /home/jsy/Projects/navigation/active/mas_nav
python3 tools/benchmark_lio_bag.py \
  --bag /home/jsy/Projects/navigation/rosbag2_mapping \
  --output .scratch/adaptation-rerun --rate 4 --warmup 20 \
  --modes baseline adaptive
python3 tools/benchmark_lio_bag.py \
  --bag /home/jsy/Projects/navigation/rosbag2_mapping \
  --output .scratch/adaptation-ablation --duration 30 --rate 1 --warmup 10 \
  --modes baseline spatial temporal adaptive
```

结果位于 `.scratch/small-adaptation-comparison-20261103/`：正式整包为
`corrected-full-4x/`，分项为 `corrected-ablation-1x/`。参数、逐秒资源采样、接收计数、
轨迹和日志均可检查。旧单位配置的中间运行不纳入新结论。


最终核验：Small Point-LIO 4 组共 16 项功能回归和录包解码/配置 6 项回归通过。
安装后的启动对象仅包含 Small Point-LIO，两项自适应均开启，已删除包在 active 安装中
不可发现；脚本编译和 `git diff --check` 通过。测试进程已退出，未启动实车硬件。
