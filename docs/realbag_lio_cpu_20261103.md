# 实车录包 LIO CPU 对比（2026-11-03）

**后续审计更正：此前实包测试将加速度按 SI 输入，但 Small Point-LIO 的加速度饱和阈值未同步换算，可能触发不必要的最短批次；其中 Small Point-LIO 的 CPU 数值不应作为正常饱和门配置下的结论。当前已在测试专用配置中修正，重新对比见 [自适应前后](small_lio_adaptation_comparison_20261103.md)。**

**历史对比记录：ZLIO 前端已删除。现有录包脚本已改为 Small Point-LIO 的自适应开关对比，当前结果见 [自适应前后对比](small_lio_adaptation_comparison_20261103.md)。下列旧模式命令仅作历史说明。**

使用用户提供的 `/home/jsy/Projects/navigation/rosbag2_mapping`，替代仅依赖合成场景的判断。
前端执行文件与 active 安装保持一致；未启动真实雷达驱动、串口、全局定位、规划或 RViz。

## 输入审计与统一适配

- SQLite 文件 `rosbag2_2025_07_24-11_36_07_0.db3`，1,894,232,064 字节；记录跨度
  462.858 s，包含 6942 条 Livox CustomMsg 和 92578 条 IMU。
- IMU 已是 m/s²，约 200 Hz；没有重复/回退时间戳，最大采样间隔 6.914 ms。
  两前端均按 SI 输入，不使用当前实车驱动的 g 单位转换倍率。
- 依据仓库中的 CustomMsg/CustomPoint 定义解码小端 CDR1：处理 Header 字符串和
  8 字节对齐，保留 `timebase + offset_time` 的真实逐点绝对时间。统一使用
  `(tag & 0x3f) == 0` 的严格回波掩码，然后发布相同 FLOAT32 XYZ + FLOAT64 秒时间。
  不是伪造扫描内时间，也不需要 Livox SDK。文件始终以只读模式打开。
- 录包没有提供实际车体安装标定。基准仅假定 base_link 到 lidar_link 为恒等变换，
  两边使用相同的 LiDAR→IMU 平移假定（沿用前端默认值），盲区以雷达原点为中心、半径 0.38 m，
  最大距离 100 m。没有将当前车辆的偏心盲区/安装姿态错误套到未知录包设备。
  因此输出用于资源和估计连续性检查，不作为车体坐标或绝对定位精度验收。

## 测量方法

`tools/benchmark_lio_bag.py` 在独立 ROS 域依次启动两个前端，回放相同数据。
ZLIO 的 CPU/RSS 包含原生节点和兼容节点，Small Point-LIO 包含其单节点。
转换、发布、静态 TF 和监测器不计入前端 CPU。

工作进程 user+system CPU / 回放真实耗时 ×100%，100% 表示占满一个逻辑核。
RSS 为进程采样 RSS 之和，可能重复计入共享页。OMP_NUM_THREADS=2，
OMP_WAIT_POLICY=PASSIVE；两者保留自己的地图/滤波/时空自适应配置，不人为限制为
相同的有效点数。初始预热时间不纳入稳态 CPU，另保留含启动处理的总 CPU 秒。

两个前端都开启默认关闭的 `publish_processing_stats`：每秒报告接收扫描、IMU、点数；
Small Point-LIO 还报告批次和接受匹配数。只有接收计数与发布计数全部一致、持续输出
有限位姿、有接受的激光匹配、无 ZLIO 故障并且末尾里程计落后不足 0.5 记录秒的运行，
才作为有效比较。发布前先等待订阅匹配及 1 s 数据链路建立，避免初始发现丢包。
基准另开启可靠输入传输，统一 IMU/点云队列为 4000/200；原先最佳努力输入的
全量预检出现 1/92578 条 IMU 丢失，因此未将该轮纳入正式同输入结果。
这些选项默认关闭，生产输入仍沿用原先 best effort 和队列设置。

输出频率区分真实墙钟秒和录包秒；回放消息年龄由输出 stamp 相对回放时间轴计算，
包含记录/传输时间差和扫描等待，并不是纯算法内核耗时。全量倍速结果不能直接当作
正常 1 倍速下的 CPU 百分比，也不能与此前合成场景数值直接比较。

## 结果

整包各运行一次，回放倍率均为 2×，前 20 记录秒为预热。正式结果均通过完全接收和估计健康检查。
以下是同一份实测数据，不沿用之前合成场景的 CPU 数字。

| 整包 2× 指标 | Small Point-LIO | ZLIO＋适配节点 |
|---|---:|---:|
| 稳态 CPU，单逻辑核 100% | 19.60% | 56.37% |
| 稳态 CPU 秒 / 墙钟秒 | 43.40 / 221.40 | 124.80 / 221.40 |
| 含启动处理的总 CPU 秒 | 45.15 | 130.17 |
| 总 CPU 秒 / 每记录秒 | 0.0975 | 0.2812 |
| 峰值合计 RSS | 40.57 MiB | 112.33 MiB |
| 里程计频率 / 墙钟秒 | 181.38 Hz | 157.73 Hz |
| 里程计频率 / 记录秒 | 90.69 Hz | 78.87 Hz |
| 完整点云频率 / 记录秒 | 15.00 Hz | 15.00 Hz |
| 回放消息年龄 P95（墙钟） | 45.45 ms | 2.83 ms |
| 末尾输出落后（记录时间） | 80.21 ms | 0.19 ms |
| 首次有效输出（墙钟） | 0.54 s | 1.19 s |
| 有效匹配总数 | 4,315,697 | 14,821,935 |

两者接收数均为：6942 帧、92578 条 IMU、90,588,646 个统一筛选后的点。
原始 CustomMsg 共 92,565,216 点，回波掩码在进入任一前端前统一应用。
Small Point-LIO 共处理 344816 个批次；ZLIO 接受 13846 个激光窗口更新。
二者无 session 故障，位姿/速度有限，末尾输出均跟上录包；没有用空闲进程或
初始化失败的低占用充当结果。最大速度约 2.33 / 2.18 m/s，最大离起点距离
约 23.52 / 23.43 m，仅用于检查估计没有明显发散，不能替代真值精度评价。

该整包中 ZLIO 稳态 CPU 约为 Small Point-LIO 的 2.88 倍，合计 RSS 约为 2.77 倍。
ZLIO 的回放输出更及时；实际里程计频率则较低，不能把此前合成输入下的
“100 Hz 对 80 Hz”直接推广到这份有采样抖动的实测输入。有效匹配数和批次定义不同，
不能将匹配数量直接作为精度排名。

### 正常 1× 短段交叉检查

运行相同前 40 记录秒、预热 10 秒，顺序改为 ZLIO→Small Point-LIO。两者均完整收到
599 帧、8001 条 IMU 和 7,818,071 个筛选后的点，并通过估计健康检查。

| 前 40 s，正常 1× | Small Point-LIO | ZLIO＋适配节点 |
|---|---:|---:|
| 稳态 CPU / 单核 | 9.61% | 27.27% |
| 峰值合计 RSS | 38.31 MiB | 86.46 MiB |
| 里程计频率 | 90.58 Hz | 78.89 Hz |
| 完整点云频率 | 14.98 Hz | 14.98 Hz |
| 回放消息年龄 P95 | 85.53 ms | 2.92 ms |
| 首次有效输出 | 1.07 s | 2.01 s |

该短段在正常速率下同样表现为 ZLIO 约 2.84 倍 CPU，输出年龄更小。
短段 RSS 不能替代全包峰值；短段 CPU 也不代表全包所有运动阶段的 1× CPU。
本次整包各一次、正常速率短段各一次，没有将不同阶段或倍率混为重复测量的中位数。
没有外部真值，不能根据资源与连续性检查宣称哪种算法实车精度更好。

## 验证与证据

- 两前端 Release 重建、安装，原有 75 项功能回归通过；另有 4 项 CDR 解码回归通过，
  覆盖字符串对齐、严格回波掩码、绝对逐点时间、截断/字节序拒绝和空消息。
- `publish_processing_stats`、`reliable_input` 默认关闭，Small Point-LIO 的新增队列参数
  默认保持原有 5；仅基准临时配置启用可靠输入及 4000/200 队列。实车配置未改为 SI，
  未将本录包的假定外参写回生产配置。
- 原始包只读。所有测试工作进程已退出，没有启动实车硬件或导航执行。
- `full-reliable-2x/` 为正式整包结果，`prefix-reliable-1x/` 为正常速率短段；
  `full-2x/` 那轮少 1 条 IMU，保留日志但不计入正式结果。
- 本地证据目录 `.scratch/realbag-cpu-20261103/`，含逐秒资源采样、节点日志、
  输入审计、各运行参数、`full-summary.json`、`prefix-summary.json` 和二进制校验信息。
  `git diff --check` 与脚本 Python 编译通过。

## 复跑

```bash
source /opt/ros/jazzy/setup.bash
source /home/jsy/Projects/environment/build/ros/jazzy/navigation/active/mas_nav/install/setup.bash
source /home/jsy/Projects/navigation/active/mas_nav/install/local_setup.bash
cd /home/jsy/Projects/navigation/active/mas_nav
python3 tools/benchmark_lio_bag.py \
  --bag /home/jsy/Projects/navigation/rosbag2_mapping \
  --output .scratch/realbag-cpu-rerun --rate 2 --warmup 20
```

可用 `--rate 1 --duration 40 --warmup 10` 检查正常速率短段，`--modes zlio small`
调换顺序。未传 duration 时回放整包。原始结果和日志保存在
`.scratch/realbag-cpu-20261103/`；合成测试文档保留为历史参考。
