# 修改历史

本文件记录由开发任务产生的代码、配置、脚本、资源和文档变更。新记录追加在最上方，不改写旧记录。

## 2026-09-29 — 修复在线回退路径净空检查与 SMAC 拒绝诊断

- 改动：针对 17:40–17:46 运行日志中的大量 SMAC/MINCO 失败，修复 `GlobalPathSearcher` 在线子目标回退只检查终点净空、不检查途经格的问题。启用 SMAC 时，回退的途经格与斜向两侧格共用 SMAC 最低净空，距离场无效或非有限时拒绝；每轮按格缓存判定，仅保留与直接 SMAC 一致的起点已有位置豁免。`planner_params.yaml` 的最低净空由 0.25 提到 0.27 m，比当前轨迹静态有效硬阈值 0.26 m 留 1 cm 余量，不修改碰撞、动态安全或脱困保护。SMAC 目标拒绝区分栅格代价、ESDF 净空不足和查询不可用；端点诊断将 `cost_traversable` 与包含净空的 `smac_traversable` 分开，打印净空、阈值、查询状态及快照/投影/掩码/距离场序号，修正“目标可通行但不可达”的误导文案。`LocalPathProcessor` 起点负净空新增位置、格子代价和快照证据日志，保留原有制动与安全判据。更新在线导航说明，新增 7 项回归用例。
- 验证：执行器与 bringup Release 构建通过，最终执行器增量构建与 CTest 5 个套件共 39 项测试全部通过。新增用例覆盖低净空区域绕行、不能跨越净空屏障、净空角点阻断、空闲但低净空目标的安全子目标回退、无效距离场与低净空起点离开、无 SMAC 的 A* 失败诊断、SMAC 空闲目标的净空/距离场拒绝。实际测试日志确认目标净空 0.100 m、阈值 0.270 m 时，`cost_traversable=true` 且 `smac_traversable=false`，缺失距离场单独报告不可用。YAML 解析、安装参数软链接和内容一致性、`git diff --check` 通过。没有启动硬件或底盘，没有实车复测负净空来源、卡顿次数和新阈值下的窄通道通过性，也未测量新增回退距离场查询的现场耗时；平滑后轨迹仍需安全校验，不能据本次测试认定现场规划失败已全部消除。

## 2026-09-29 — 地面灰黑闪烁诊断与邻域平面校验

- 改动：只读采样当前 ROS 地图、点云、里程计和参数，确认累计图存在真实的 UNKNOWN→OCCUPIED 切换；10 秒中观测到 117 个这种切换格，运行薄表面阈值为 0.08 m，局部图中存在靠近地面且占用跨度为 0.10–0.15 m 的柱（统计不能确认每一柱都是真实地面）。新增 ROGMap `ProjectionLayer::filterGround()`，对模糊低矮柱用半径 3 格内的原始薄表面拟合邻域平面；至少 8 个样本、四侧支持、RMS≤0.025 m、坡度≤0.7，柱内跨度≤0.15 m 且上下高度均在平面±0.075 m 内才纠正为 PASSABLE。纠正柱不作为新的拟合种子，增量更新重新校验支持并在失效时恢复阻断；通过校验时同时更新 raw_type，供累计图记录自由证据。增加 `projection.ground_filter.*` 配置，保留原始 0.08 m 薄表面阈值，默认开启有限容差的邻域校验。RViz 默认关闭 Layer Type 诊断栅格，避免与累计图共面叠加；更新在线导航说明与 6 项地面回归测试。主要涉及 rog_map 投影/配置/快照生成接线、执行器 YAML、RViz 和文档。
- 验证：rog_map、执行器与 bringup 三包 Release 构建通过，仍有原 PCL 配置和 fmt 弃用告警；ROGMap 地形/地面 10 项与执行器 32 项测试全部通过，新增测试覆盖噪声地面、10 cm 垂直凸起/高墙保留、未知/非平面邻域、连续坡面、增量邻域支持失效和关闭开关。隔离 ROS 域 97/98 的相同合成噪声点云对比：关闭时测试地面区域 1784 格为占用，开启时为 0；2×2 格墙体均保持占用。夹具保留每柱两次返回并避开 float32 体素边界，日志与结果在 `.scratch/ground-flicker/`。域 96 闭环限速 1 m/s 推进至 x≈9.722 m，累计图从 82×83 扩为 175×87，历史自由格保留；点云中断后制动，最终指令为零。201×201、20% 噪声厚柱的合成二维投影基准中，中位数约 1.99→8.84 ms，开启后的 p95≈14.56 ms；不包含三维射线/距离场开销，不能代表现场实时性能。YAML 解析与 git diff --check 通过。没有重启用户正在运行的导航，尚未进行新版现场灰黑闪烁复测；有限容差并非完整地面分割，厚重影、稀疏边缘、不平地形、低矮物体区分及当前 3/4 m/s 行驶仍需现场验收。

## 2026-09-29 — 修复本机 codex app-server daemon 包损坏

- 改动：`~/.codex/packages/app-server-daemon/current` 原先指向 0.159.0 release，该目录内 `bin/codex`、`bin/codex-code-mode-host`、`codex-path/rg`、`codex-resources/bwrap` 及 `voice` 全部为 0 字节空文件，分离式启动 daemon 因此报 `Exec format error (os error 8)`。改用官方 `codex app-server daemon update --from-cli -y`，从 npm 已安装的 CLI 包（0.158.0，`@openai/codex-linux-x64` 的 vendor 树）离线复制并固定出新 release `local-6173a84c…-x86_64-unknown-linux-musl`，`current` 重新指向该完整包；删除全为 0 字节的 0.159.0 release 目录，避免按版本号被复用；保留完整的 0.157.1 release 作为回退。本次未改动仓库内代码、启动文件或参数，属于本机工具链修复。
- 验证：`current/bin/codex` 为有效 ELF（286 MB，与 CLI vendor 树一致），包内 44 个文件无 0 字节残留；`codex app-server daemon start` 与 `restart` 均返回 `status: started`/`restarted`，daemon 以 `app-server --listen unix:// --managed-daemon` 常驻（pid 34094），`~/.codex/app-server-control/app-server-control.sock` 可连通，`daemon version` 返回 `running` 且 CLI 与 app-server 均为 0.158.0；`codex doctor` 的 Background Server 由 idle 变为 `✓ app-server running (ephemeral mode)`，总评 `20 ok · 3 notes · 1 warn · 0 fail`。未执行生产更新路径 `codex app-server daemon update`，未验证 0.159.0 能否正常安装；最初的 0 字节写入失败未复现，推测由无法写入 `~/.codex` 的受限进程安装所致，尚未确证。

## 2026-09-29 — 减少短暂地面误判写入累计全局图

- 改动：ROGMap 查询快照增加 `raw_types`，保留形态学滤波与保持处理前的分类。累计地图排除原始 UNKNOWN、滤波填充及保持生成的 OCCUPIED，避免把局部安全补洞当作新的长期障碍证据；不会将这些格直接当作自由区。`online_global_map` 增加启动参数 `occupied_confirmations=2`、`confirmation_max_gap_s=1.5`，新障碍需要连续新快照确认；待确认格为 UNKNOWN 并在占用/距离查询中阻断，恢复完整自由观测即可清除未确认候选，已有确认障碍继续使用原多观测清除机制。实时局部障碍、投影地面阈值与轨迹安全检查未调整。本次针对累计图放大短暂误判的机制，尚未凭现场数据确认或解决原始投影持续地面误判。涉及 ROGMap 快照生成、累计地图、PathPlanner 参数、YAML、回归测试和在线导航说明。
- 验证：rog_map、执行器与 bringup 三包 Release 构建通过，ROGMap 构建仍有 PCL 配置与 fmt 弃用告警；执行器 5 个 CTest 套件共 32 项、ROGMap 地形分类 4 项全部通过。新增 3 项回归测试覆盖待确认格不通行及自由恢复、重复/未知中断/超时观测不确认、具有正置信度的原始自由格被局部填充/保持成障碍时不写入累计图；原有累计地图、规划与安全测试保留。隔离 ROS 域 96 重跑合成地板/理想底盘闭环，1 m/s 限速推进至 x≈9.049 m，累计图发布 34 次并从 82×84 扩为 168×84，窗口外历史自由格仍保留；点云中断后 rog_map_stale 制动且最终指令为零。脚本/日志位于 `.scratch/online-global-map/`，有初始运动学重试告警，不能代表现场误判或实时性能验收。参数 YAML 解析及 git diff --check 通过。未重启用户导航或启动硬件，未对现场地面误判做复测，未验证当前 3/4 m/s 实车配置。

## 2026-09-29 — B1 在线累计全局栅格与固定版本查询

- 改动：新增 `common/environment/online_global_map.{hpp,cpp}`，在本次启动的 odom 坐标中保留滚动 ROGMap 的历史观测，按完整覆盖保守降采样；未知及零观测置信度的滤波填充格不覆盖历史，障碍立即阻断，多次新自由观测才能清除，重复或回退时间的快照不累计证据。矩形地图按需扩展，达到容量上限整次拒绝并保留历史；发布不可变查询快照，支持占用、坐标换算、有符号距离和梯度，更新写锁与快照读锁分开。`PathPlanner` 在 online 模式以独立回调组定时累积新鲜快照，通过 `/nav_executor/online_global_map` 发布 reliable/transient-local OccupancyGrid，并提供 `globalMapQuery()`；增加启动只读 YAML 设置（默认 0.1 m、2 Hz、200 万格），RViz 显示、10 项测试及 README/在线导航说明。B1 完成地图记忆与查询接口，实际任务搜索仍为 A 方案，累计图的任务全局路线接入留待 B2；没有新增落盘、回环或跨次启动定位。
- 验证：执行器与 bringup 两包 Release 构建通过；5 个 CTest 套件共 29 项测试全部通过，其中 10 项累计图测试覆盖未知阻断代价不被记成墙、无观测滤波格排除、负坐标和半粗格滑动、历史和旧快照保留、粗格自由覆盖与障碍优先、多帧清障、容量拒绝、非法输入/时间回退、有符号距离场，以及现有搜索接口沿历史自由区返回局部窗口外。隔离 ROS 域 96 的合成地板点云与理想全向底盘闭环中，20 m 目标推进至 x≈9.162 m，输出速度最大 1.0 m/s；17 s 后停止点云，触发 rog_map_stale 制动，最终指令为零。累计图发布 33 次，从 81×83 扩为 168×89，确认已离开当前 10 m 局部窗口的 (2.05, 0.55) 历史自由格仍保留。运行有 ROGMap 丢帧与 MINCO 初始运动学重试告警，不能视为零丢帧或实车实时性能验证；未见累计图输入拒绝告警。临时脚本与日志位于工作空间 `.scratch/online-global-map/`。参数/RViz YAML 解析及 git diff --check 通过；未重启用户导航进程，未启动硬件/底盘通信，未验证当前 3/4 m/s 配置、真实动态障碍、长期地图性能、回环漂移或实际返航。

## 2026-09-29 — 提交 A 方案与后续 YAML 启动配置

- 改动：提交无先验在线局部导航 A 方案、实验室先验地图成组选择，以及工作区已有的 YAML 唯一模式选择与 YAML 速度上限配置；具体实现和历史验证见下方对应记录。本次没有新增导航行为。
- 验证：提交前重新运行四个 CTest 套件，共 19 项测试全部通过；启动文件 Python 语法与 git diff --check 通过；从实际安装配置构造 online 启动，确认模式为 online、不启动先验定位/地图服务、不读取地图文件、保留未知区阻断，且速度参数没有固定覆盖。未重启当前导航进程，未验证当前 3/4 m/s 上限下的合成闭环或实车表现。

## 2026-09-29 — 无先验导航提速并采用 YAML 速度配置

- 改动：`nav_executor_launch.py` 移除 online 模式对 MINCO 和普通/特殊区域速度、加速度的固定 1.0 覆盖，采用现有 `planner_params.yaml`、`region_control.yaml` 与 `mpc_params.yaml`：普通/隧道/起伏路段上限 3.0 m/s、4.0 m/s²，坡道与 MINCO 总上限 4.0 m/s、5.0 m/s²。更新根 README 和在线导航说明，保留未知区域阻断与关闭先验融合的在线策略。
- 验证：Python 语法检查通过；从安装目录配置构造 online launch 并按加载顺序合并参数，确认 MINCO、各区域和 MPC 的实际配置上限一致且不再被固定 1.0 覆盖；确认不启动先验定位/地图服务、不读取先验地图清单，以及未知区和先验融合策略保持有效；`git diff --check` 通过。未重启正在运行的导航、未启动硬件或做提速后合成闭环/实车验证；历史 1.0 m/s 合成运行不能代表当前较高速配置的验证结果。

## 2026-09-29 — 整套导航启动模式仅由 YAML 选择

- 改动：`nav_executor_launch.py` 从 `mas2027_nav_executor/config/node_params.yaml` 的 `node.map_source` 读取并校验 `online`/`prior`，移除独立的 `map_source` launch 参数与固定 prior 默认值；同一 YAML 取值控制先验文件加载、定位/地图服务、TF、RViz 坐标系和在线规划参数。保留用户当前 `online` 配置，更新参数注释、根 README、导航 README 与在线导航说明。模式缺失或非法时在启动构造阶段报错。
- 验证：Python 语法检查通过；临时 YAML 与模拟节点构造验证 online/prior 两个分支的节点选择、map→odom 开关、RViz 坐标系和执行器参数一致，online 分支使用抛异常的先验文件加载器确认不会读取先验清单，prior 分支使用现有地图清单成功构造；非法值与缺失字段均拒绝。确认安装目录符号链接读取当前 online 配置，`git diff --check` 通过。未重启用户当前导航进程、未启动硬件或做实车验证；本次仅修改 Python 启动与配置注释/文档，无需重新编译 C++。

## 2026-09-29 — 无先验地图在线局部导航（A 方案）与先验地图切回实验室组

- 改动：主启动新增 `map_source:=online`，通过延迟构造跳过先验地图清单读取，不启动 GICP 定位与静态地图服务；`tf_maintainer` 仅发布 odom→base_link，RViz 自动采用 odom。默认 prior 模式继续加载地图文件与静态/在线融合查询。`node.map_source` 为启动时只读参数；online 不创建 TerrainGrid，目标接收、MINCO、走廊、轨迹及 MPC 指令安全门使用在线 ROGMap，保留地图/里程计新鲜度和净空检查。未知区在在线投影和距离场中按障碍处理，初始普通/特殊区域速度与加速度上限统一为 1.0；缺失或过期的在线特殊标签使用普通 mode。局部窗口搜索增加已观测自由区中的推进子目标，要求朝任务目标进展与足够净空，禁止斜向穿越障碍角点；TaskManager 保留原始目标，在滚动窗口变化时约每秒或到子目标后重搜，规划失败阶段也刷新在线路径。新增在线搜索与命令门测试、在线区域标注测试及 `docs/online_navigation.md`，更新 README。按用户补充要求，`navigation_map.yaml` 的 PCD/YAML/terrain 成组切回 `lab_map_20260921_211523`，fengbao 资产保留。
- 验证：两包 Release 构建通过；导航四个 CTest 套件共 19 项测试通过；Python 启动文件语法检查、YAML/配套文件核对与 `git diff --check` 通过。构造 launch 时用抛异常的地图加载器证明 online 不读取清单，且不包含 map_server/odom_localizer，prior 仍包含二者。隔离 ROS 域 92 中，不提供任何先验地图或 map→odom，合成地板点云与理想全向底盘闭环完成 8 m 目标，最终 x≈7.963 m，最大输出平移速度≈1.0 m/s；另一轮 20 m 远目标持续推进至 x≈8.499 m，任务未完成时切断点云，日志在地图年龄 0.526 s 时触发 rog_map_stale 制动，最终速度为零。临时验证脚本与日志位于工作空间 `.scratch/online-navigation/`。实际加载 lab terrain/YAML 成功：772×308、0.05 m/px、原点 (-5.237015, -7.874179)，源码与安装清单均已切换。未启动硬件雷达/通信桥、未做实车或真实动态障碍行驶，未验证稀疏点云、坡道/隧道、长距离漂移、死胡同脱出或返航；A 方案没有新增长期全局地图和回环。

## 2026-09-29 — 提交无先验导航改造前的现有更改

- 改动：提交当前工作区已有的静态/在线融合查询、SMAC 墙角与净空约束、普通/坡道速度及区域控制参数、配套测试；地图选择切换为 `fengbao_map`，加入对应 PCD、PGM、YAML 和 terrain MSGPack，导航启动默认打开 RViz。此前详细开发与验证过程保留在下方 2026-09-26 的记录中。
- 验证：提交前 `git diff --check` 通过；当前构建目录 `ctest --output-on-failure` 的三个套件全部通过；安装后的 `map_server_node` 实际加载 fengbao 地图，核对为 832×460、0.05 m/px、原点 (-17.159674, -8.145816)，地图服务内部尺寸/分辨率校验通过。没有重新全量编译或实车行驶验证。

## 2026-09-26 — 融合在线 ROGMap 与静态地形净空的局部查询接口

- 改动：新增 `mas2027_nav_executor` 的 `common/environment/fused_map_query.{hpp,cpp}`，用一个 `rog_map::MapQueryInterface` 同时呈现在线滚动 ROGMap 与静态地形距离场：查询点按最新 `static_frame <- output_frame` 平面变换折算到静态地图，占用取并集（有符号净空取两者较小值），静态梯度经 `R^T` 转回动态系，网格几何沿用动态图。缺少变换、静态图外、插值失败或非有限输入时整体失败（fail-closed），`values()` 返回 `nullptr`，逐格结果经 `copyValues()` 合成。`minco_planner` 新增 `rebuildFusedQuery()`/`refreshFusedTransform()`，把 MINCO 优化器、走廊生成和安全检查的局部查询由纯动态查询换成该融合查询，并在重规划与安全定时回调中刷新变换；`CMakeLists.txt` 把 `fused_map_query.cpp`、`terrain_grid.cpp`、`terrain_map_query.cpp` 编入 `_planner` 静态库（原先只在节点目标内），节点目标改为仅保留 `region_control.cpp`，静态库链接补 `OpenCV_LIBS`。新增 `test/test_fused_map_query.cpp` 5 项用例，覆盖缺变换、静态/动态净空取小与梯度旋转、动态更近时保留动态结果、静态障碍屏蔽动态空格、静态图外 fail-closed。
- 验证：`colcon build --packages-select mas2027_nav_executor --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过（首次增量构建为空转，随后 touch 6 个改动源文件与 3 个测试文件强制重编，`fused_map_query.cpp`、`minco_planner.cpp`、`smac_planner_2d_simple.cpp`、`nav_executor_node.cpp` 及三个测试单元全部重新编译，日志 0 warning / 0 error，节点与 `test_fused_map_query` 重新链接）；`ctest --output-on-failure` 三个套件全部通过：`test_fused_map_query` 5 项、`test_region_online_priority` 4 项、`test_smac_wall_clearance` 2 项，`colcon test-result` 无失败。未做实车或仿真运行，未在真实 TF/点云下测量融合查询的耗时与在线行为，也未验证静态图外的现场退化表现。

## 2026-09-26 — 修复坡道启动参数并增强 C 型拐道外侧偏好

- 改动：`mas2027_nav_executor/config/region_control.yaml` 将坡道准备、请求 mode、提交距离设为 0.3/0.2/0.1 m，满足节点所需的递减顺序，修复原 0.3/0.8/0.2 m 导致的启动崩溃。普通路段显式限制为 3.0 m/s、4.0 m/s²，坡道保留较高的 4.0 m/s、5.0 m/s²；MINCO 修正区域标签索引为 5/6/7、按坡道窗口允许较高分段速度，MPC 总上限提升至坡道值，区域控制器在普通 mode 下仍钳住普通上限。涉及 `region_control.cpp`、`nav_executor_node.cpp`、`minco_planner.cpp`、三份参数文件、区域控制说明与切换测试。`planner_params.yaml` 另将 SMAC 2D 静态 ESDF 权重从 2.0 提到 4.0，增强 C 型拐道全局折线的外侧净空偏好；原有最小净空和轨迹碰撞检查保留。
- 验证：查看 15:22 正常运行日志，发现多次 `TERRAIN_COLLISION` 及轨迹净空低于 0.26 m；用当前地图和该次日志中的起终点作离线栅格搜索对比，一条长路线权重 2→4 时最小静态净空约 0.32→0.39 m、长度约增加 0.14 m，另一段受窄通道约束的路线无变化。`colcon build --packages-select mas2027_nav_executor --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过；区域控制与 SMAC 净空两组测试合计 7 项通过；核对 YAML 中普通/坡道/MINCO/MPC 上限及阶段距离；独立启动重编节点 5 秒，完成初始化并打印 `weight=4.000`，未再抛出区域参数异常；`git diff --check` 通过。离线搜索是对 SMAC 代价的近似复现，未做完整导航启动、实际 C 型拐道行驶或底盘提速验证。

## 2026-09-26 — 全局路径绕横墙端点保留净空

- 改动：SMAC 2D 斜向扩展须同时通过两个相邻正交格，禁止从障碍角点穿越；增加基于静态地形 ESDF 的 `smac_2d.minimum_clearance`，当前配置为 0.25 m，搜索不会选取低于该中心净空的格子。该参数可在线调整，窄通道或贴墙目标若不足此净空会报告无解。涉及 SMAC 搜索、MINCO 参数接线、规划配置和横墙测试；先前未提交的 ESDF 权重 2.0 调整与其他工作区修改保留。
- 验证：`colcon build --packages-select mas2027_nav_executor --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过；`colcon test --packages-select mas2027_nav_executor --ctest-args -R test_smac_wall_clearance --output-on-failure` 的角点阻断与短边绕行两项测试通过，`git diff --check` 通过。未在当前实际地图上按具体起终点对比路径，未做实车行驶验证。

## 2026-09-26 — 全局路径增强通道中间偏好

- 改动：`mas2027_nav_executor/config/planner_params.yaml` 将 SMAC 2D 的 `esdf_weight` 从 `1.0` 提高到 `2.0`，增强贴墙路段的距离场软代价，使全局折线更倾向通道中间；`esdf_decay` 保持 `0.8 m`、封顶 `5.0` 仍高于权重，不改变占据格或硬净空判据。同步更新参数说明。
- 验证：PyYAML 解析并核对 ESDF 开关、权重、衰减和封顶关系；检查当前安装目录的参数文件软链接指向源码配置，`git diff --check` 通过。仅改配置与文档，未重新编译、未在实际地图上对比新旧路径，也未做实车行驶验证。

## 2026-09-25 — LIO 按雷达输入帧发布去畸变点云

- 改动：`small_point_lio` 的预处理层记录每次雷达输入的有效稠密点数，去畸变处理跨多个 IMU 回调完成整帧后，才向 `/cloud_registered` 和 `/cloud_registered_full` 发布一次，并使用该帧最后一个点的时间戳。`/Odometry` 继续按原有估计流程发布；新增分帧边界测试。当前运行中约 20 Hz 的驱动帧此前会拆成约 50–70 Hz 点云片段，导致 ROGMap 重复更新并跳过部分片段。此改动针对当前输入放大；历史上 20 Hz 输入时的投影耗时问题仍需独立监测。
- 验证：`colcon build --packages-select small_point_lio --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过；分帧测试与 `colcon test-result` 显示 2 项测试无失败，`git diff --check` 通过。用户退出原导航后，以 `use_ros2_comm:=False` 启动新版本做 128.4 秒静止现场验证：驱动采样 20.0 Hz、`/cloud_registered` 19.9 Hz，中位 14151 点/帧；ROGMap CSV 约 2569 帧输入、2553 次更新，处理比例 99.38%（旧运行最近约 56%）。仍观察到单帧丢弃告警，不能视为零丢帧；定位器持续接受配准。测试结束已关闭本次启动的全部进程。未带底盘行驶或动态障碍验证。

## 2026-09-25 — 撤回未经验证的 LIO 合并方案并澄清积压历史

- 改动：撤回上一条记录中的 `small_point_lio_node.*` 和 `small_point_lio_params.yaml` 点云合并改动；旧记录保留作为变更过程。重新核对 2026-09-16 至 17 日实车记录：即使点云输入约 20 Hz，ROGMap 也曾仅处理 12.8 Hz，全量二维投影是当时的主要耗时；改为脏列增量后丢帧下降但未消失。因此当前约 50–60 Hz 的 LIO 分片是额外负载，不能解释积压最初出现的原因。修改仅涉及上述回退和本历史说明。
- 验证：核对旧实车 CSV 摘要、当前 `.scratch/rog_map_perf_summary.csv` 和 `/tmp/rog_map_perf_detailed.csv`，并确认 LIO/bringup 文件已恢复；恢复源码后重新执行 `colcon build --packages-select small_point_lio --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release`，`git diff --check` 通过。未重启现场进程，未做实车运行或同场景 A/B 对比。

## 2026-09-25 — 合并 LIO 去畸变点云分片以缓解 ROGMap 积压

- 改动：`small_point_lio_node` 为 `/cloud_registered` 和 `/cloud_registered_full` 增加可配置的发布周期，合并 IMU 回调产生的点云片段；当前 bringup 配为 0.05 s，目标与 MID360 的 20 Hz 雷达帧频率一致。里程计及 PCD 保存仍按原回调处理，无订阅者时清空暂存点云。默认周期 0 保持其他启动配置的原发布行为。
- 验证：现场采样确认 `/mid360_driver/lidar` 约 20 Hz，而 `/cloud_registered` 每雷达帧分成约 3 个片段，ROGMap 每秒接收约 50–60 帧但只能处理约 25–40 帧。`colcon build --packages-select small_point_lio --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release` 和 `git diff --check` 通过。当前机器人仍运行旧进程，未重启验证新发布频率和 ROGMap 丢帧率，也未做实车行驶测试。

## 2026-09-25 — 补充 Foxglove 在线地形可视化步骤

- 改动：`docs/foxglove_remote_debug.md` 增加 3D 面板中开启 `/rog_map/terrain_markers` 的具体步骤、颜色含义、原始标签核对方法，以及话题白名单和 TF 排障说明。代码和消息格式未改。
- 验证：核对 Foxglove 官方 3D 面板文档支持 ROS 2 `visualization_msgs/msg/MarkerArray`，并检查仓库 bridge 启动与话题名称；`git diff --check` 通过。未实际连接 Foxglove 客户端验收。

## 2026-09-25 — RViz 显示在线坡道与隧道区域

- 改动：ROGMap 新增 `/rog_map/terrain_markers`，把在线识别的坡道 `5` 画为橙色方块、隧道 `6` 画为青色方块；与 `/rog_map/terrain_label` 共用同一次语义计算，并给标记设置短寿命，感知停更后不会长期残留。`nav_executor_view.rviz` 的 ROGMAP 分组默认显示彩色标记，并提供默认关闭的原始标签栅格；更新 ROGMap 与 Foxglove 使用说明。涉及 ROGMap 可视化发布器、ROS 适配层、RViz 配置和文档。
- 验证：`colcon build --packages-select rog_map --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过；ROGMap 投影测试通过；PyYAML 解析 RViz 配置并确认彩色标记默认开启、原始标签默认关闭，`git diff --check` 通过。未连接实车点云或手工打开 RViz 验收。

## 2026-09-24 — 区域模式优先采用 ROGMap 在线语义

- 改动：`nav_executor` 订阅 `/rog_map/terrain_label`，轨迹区域标注优先采用新鲜的在线坡道 `5` 和隧道 `6`，在线未知、普通地面、覆盖范围外、过期或 TF 不可用时回退静态 `/terrain_label_map`；在线候选造成区域模式窗口冲突时整条轨迹回退静态标注。同一轨迹收到新语义图后重新标注，同时保留控制器路径进度。`rog_map` 的空柱不再输出普通地面 `0`，坡度拟合改为单遍统计以降低在线开销。涉及区域控制、导航节点、参数、ROGMap 投影、相关测试和说明文档。
- 验证：`colcon build --packages-select rog_map mas2027_nav_executor --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过；在线优先级测试 4 项、ROGMap 投影测试 5 项全部通过，`git diff --check` 通过。未以现场点云标定阈值，未做实车底盘 mode 联调。

## 2026-09-24 — ROGMap 输出在线坡道与隧道候选图

- 改动：`rog_map` 投影层增加 `/rog_map/terrain_label` 诊断栅格，分别输出普通区域 `0`、坡道候选 `5`、隧道候选 `6` 和未知 `-1`。坡道由邻域薄地面高度拟合，隧道由上下占据层、已观测的中间空隙和邻域支持共同筛选；修改 `projection_layer.*`、`rog_map.cpp`、`rog_map_visualizer.*`、`rog_map_ros2.hpp`、CMake、模块 README，并添加合成栅格测试。此候选图暂不控制底盘 mode，静态 `/terrain_label_map` 仍负责区域控制。
- 验证：`colcon build --packages-select rog_map --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release` 编译通过；`colcon test --packages-select rog_map --ctest-args -R test_projection_terrain --output-on-failure` 与 `colcon test-result` 显示 5 个测试全通过；`git diff --check` 通过。未以现场点云标定阈值，未实车验证坡道/隧道识别或底盘 mode。

## 2026-09-24 — 统一特殊地形标签与底盘 mode

- 改动：特殊地形仅保留上坡 `5`、隧道 `6`、起伏路段 `7`，标签与各自底盘 mode 相同；`map_server` 拒绝旧的 2～4 和 8 标签，`nav_executor` 按 5/6/7 标注区域并校验 mode 与标签一致。当前导航 terrain 地图的 2,752 个旧上坡格由 `2` 迁至 `5`，其余格位和地图几何不变。按用户补充要求，地图文件移除 `direction` 通道，`map_server` 删除方向图解析、膨胀和相关接口；RViz/Foxglove 说明清除方向图入口。`mapping_web_ui` 独立仓库的编辑选项、单通道格式和对应地图也同步调整。
- 验证：核对导航与网页端两份地图均为 `{0:211299, 1:23725, 5:2752}`，且文件不含方向通道；`colcon build --packages-select map_server mas2027_nav_executor` 成功，`map_server` 实际加载单通道 772×308 地图成功；网页端 30 个相关单元测试、前端语法/轨迹 harness 和 `git diff --check` 通过。未做实车底盘 mode 联调，也未在浏览器中手工验收编辑器。

## 2026-09-24 — Foxglove 连接说明改用当前机器人地址

- 改动：`docs/foxglove_remote_debug.md` 标明旧 Wi-Fi IP 仅是历史记录，新增连接前查询 `wlo1` 当前 IPv4 和检查 `8766` 监听的步骤；SSH 隧道、直连及排障示例改为使用当前地址，并注明 bridge 默认只绑定本机。
- 验证：本机 `wlo1` 当前为 `192.168.77.141`，mapping Web UI 占用 `8765`，`8766` 当前无人监听；最近一次 nav launch 日志显示 bridge 曾在 `127.0.0.1:8766` 正常启动并广播话题，随后随 Ctrl-C 退出。检查文档差异和空白错误。未从笔记本实际建立 Foxglove 连接。

## 2026-09-24 — 区域模式速度与平地一致

- 改动：将 `mas2027_nav_executor/config/region_control.yaml` 的坡道、隧道、起伏路最大平移速度统一从 `1.0` 调为 `3.0 m/s`，最大平移加速度从 `2.0` 调为 `4.0 m/s²`，与普通路段的 MINCO/MPC 上限一致；mode 编号及区域切换距离不变。更新区域控制说明。区域模式切换本身不再额外限速。
- 验证：使用 PyYAML 解析三个参数文件，断言三种区域上限均与 MINCO `3.0/4.0` 一致，并核对 MPC X/Y 轴的 `3.0/4.0`；`git diff --check` 通过。仅改参数与文档，未重新编译或做实车速度验证。

## 2026-09-24 — 平地机器人移除运行时方向图依赖

- 改动：保持现有地形标签与 mode 编号，`nav_executor` 的 `TerrainGrid` 改由 `/cost_map` 和 `/terrain_label_map` 构成快照，搜索、轨迹与指令安全门只按静态占据栅格判定地形通过，不再订阅或要求 `/direction_map`；`map_server` 停止计算、发布方向图，旧 terrain msgpack 的 `direction` 字段保留兼容但可省略。同步更新启动映射、参数、规划拒绝原因及说明文档。在线 ROGMap 净空与区域 mode/限速继续工作。
- 验证：`colcon build --packages-select map_server mas2027_nav_executor --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过；启动文件 `python3 -m py_compile` 与 `git diff --check` 通过。未做实车行驶验证。

## 2026-09-24 — 平地坡道模式测试区解除方向阻断

- 改动：`TerrainGrid` 增加 `node.enforce_slope_direction` 开关，统一用于规划搜索、MINCO 轨迹检查、执行命令安全门和规划约束可视化；关闭时 `SLOPE=2` 仍触发区域 mode 与限速，但不再按方向图限制行进角度。当前 `node_params.yaml` 关闭此开关，以适配将平地标作坡道的测试地图；`region_control.md` 说明真实坡道需重新开启。静态占据与其他定向地形约束保持原判据。
- 验证：核对最新运行日志中 `TERRAIN_COLLISION_OR_DIRECTION`、`terrain_transition` 刹车，以及当前地图中 2,752 个坡道格方向值均为 0；`colcon build --packages-select mas2027_nav_executor --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过，`git diff --check` 通过。未做实车行驶验证。

## 2026-09-24 — 同步下位机 mode 编号到导航默认值

- 改动：按当前 `region_control.yaml` 中的普通 4、坡道 5、隧道 6、起伏路 7，同步 `nav_executor_node`、`ros2_comm` 和 bringup 的普通 mode 默认值；更新 nav_executor 总览和区域控制文档中的编号说明。保留 YAML 当前配置。
- 验证：检查配置、导航/桥接默认值与文档映射一致，并执行 `git diff --check`。本次未编译、未运行测试或实车验证。

## 2026-09-24 — 细化 nav_executor_node 功能分层注释

- 改动：进一步把 `nav_executor_node.cpp` 标注为参数读取、区域策略、MPC 配置、地图/规划装配、轨迹标注、机器人状态输入、实时命令输出、规划诊断、可视化、任务接入、控制计算、共享状态和 ROS 调度等功能层，并标明各层之间的数据职责。仅调整注释，不改行为。
- 验证：执行 `git diff --check`；未编译或运行测试（仅注释修改）。

## 2026-09-24 — 按职责整理 nav_executor_node 注释

- 改动：在 `mas2027_nav_executor/src/nav_executor_node.cpp` 为参数配置、区域控制、核心模块组装、输入订阅、输出发布、定时任务、可视化、目标接纳、控制周期、成员状态分组和主线程生命周期增加中文分区说明；不改函数逻辑和运行行为。
- 验证：执行 `git diff --check` 并检查差异仅涉及注释和本历史记录；不运行测试或编译（仅注释修改）。

## 2026-09-24 — 更正 mas_vision 修改目标为桌面工程

- 改动：按用户澄清，将 mode 透传修改落实到 `/home/mas/桌面/mas_vision` 的 `ROS2_RECV_PACKET`、ROS2 接收线程、`UserSerial::sendNav` 与串口 `SendPacket`；`uint8 mode` 位于 `nav_state` 前。撤回此前误改的 `/home/mas/vision_opensource/mas_vision` 副本中的对应改动，并在 `mas2027_nav_executor/docs/region_control.md` 标明实际工程路径。导航侧协议和 0/1/2/3 映射保持不变。
- 验证：在独立临时目录配置并完整构建桌面 `mas_vision`，`base` 目标通过；检查桌面工程及导航仓库的 `git diff --check`。未上实车，下位机接收协议仍需同步更新。

## 2026-09-24 — 模式字节固定为独立字段并贯通 mas_vision

- 改动：模式映射设为普通路段 0、坡道 1、隧道 2、起伏路 3，更新 `mas2027_nav_executor/config/region_control.yaml` 与默认值；`ros2_comm` 删除 legacy/nav_state/extended 选择，固定发送 `vx, vy, mode, nav_state` 的 10 字节 UDP 载荷，超时零速保留最近 mode。bringup 不再暴露 `mode_transport`。
- 视觉链路：`/home/mas/vision_opensource/mas_vision` 的 UDP 接收结构、ROS2 线程转发、`UserSerial::sendNav` 和串口 `SendPacket` 增加独立 `uint8 mode`，均位于 `nav_state` 前；同步更新导航包总览和区域控制文档。串口发送包从 21 字节变为 22 字节，需下位机同步升级接收协议。
- 验证：`colcon build --packages-select ros2_comm mas2027_nav_executor mas2027_nav_bringup --symlink-install --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过，增加报文偏移断言后单独重编 `ros2_comm` 通过；在独立临时目录配置并完整构建 `mas_vision`，增加报文偏移断言后重编仍通过；执行 `git diff --check`。未运行测试或实车联调；下位机接收代码不在上述两个源码目录内，未修改，必须同步更新串口协议后才能部署。

## 2026-09-24 — 新增 nav_executor 整体讲解文档

- 改动：在 `mas2027_nav_executor/README.md` 新增包级总览，按目标接纳、全局搜索、MINCO、任务状态机、区域标注、MPC、安全门和底盘桥接的实际顺序解释导航链路，并列出地图/坐标系、话题、配置、零速降级与排障入口；在 `mas2027_nav_executor/docs/region_control.md` 新增隧道、坡道、起伏路的区域控制细节，作为总览的延伸阅读。本次仅改文档。
- 验证：对照入口节点、`PathPlanner`、`TaskManager`、`PathExecutor`、配置与 bringup 启动文件核对描述，并检查 Markdown 链接及 `git diff --check`。未编译、未运行测试或实车验证（文档修改不影响可执行代码）。

## 2026-09-24 — 导航侧接入隧道、坡道、起伏路模式与速度控制

- 改动：`map_server` 接受 `TUNNEL=7`、`UNDULATING=8`，保留 `SLOPE=2`，并发布原始
  `/terrain_label_map`；`TerrainGrid` 将标签与代价、方向图作为同一快照提供给规划和执行。
  `MincoPlanner` 在三类区域的准备到释放距离内降低分段速度上限。导航节点沿最终
  `/opt_path` 采样生成 `RegionSegment`，`RegionController` 按单调路径进度管理准备、
  激活、进入、释放和重规划后的模式保持；不同 mode 控制窗重叠会拒绝轨迹。
  `PathExecutor` 按区域参数调整 MPC 的平移速度和加速度约束，准备段逐周期过渡，
  到 commit 前钳到目标上限，并发布同周期的
  `/nav_executor/chassis_cmd` 与诊断用 `/nav_executor/region_status`。
- 桥接：新增 `interfaces/ChassisCommand`、`RegionStatus`；`ros2_comm` 改消费
  `/nav_executor/chassis_cmd`，支持 `legacy`（默认 9 字节原协议）、`nav_state`（旧字节
  承载 mode）和 `extended`（10 字节独立 mode）三种可配置下行格式。默认兼容当前
  `mas_vision` 的固定 9 字节接收格式；未擅自改动仓库外的 `mas_vision`。
- 配置与说明：新增 `mas2027_nav_executor/config/region_control.yaml`，三类区域分别
  配置 mode、最大速度、最大加速度和四个距离参数；更新 bringup、README 与
  `docs/region_control_2026-09-24.md`。mode 默认 `-1`，经过未配置区域时停止执行。
- 验证：`colcon build --packages-select interfaces map_server ros2_comm mas2027_nav_executor
  --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release` 编译检查；
  `git diff --check` 检查空白格式。未运行测试、未上实车，也未验证 mas_vision/下位机模式
  编码与 UDP/串口透传；需要确认四个 mode 编号及 `nav_state` 是否即 mode 后再启用模式传输。

## 2026-09-23 — 移除项目自有测试与烟测文件

- 改动：删除 `mas2027_nav_executor/test/` 下 12 个单元测试、闭环/烟测和记录脚本，以及
  `mas2027_perception/mid360_driver/test/` 下 2 个测试文件；同步移除两个包 `CMakeLists.txt`
  中的测试目标，并删除 README 中指向已删除测试脚本的命令。
- 范围：保留 `third_party/`、`vendor/` 和 `3rdparty/` 中随外部依赖携带的测试源码；保留历史
  审计与变更文档中对过去测试的记录。
- 验证：执行 `colcon build --packages-select mas2027_nav_executor mid360_driver --symlink-install
  --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`，
  两个包均构建成功；`git diff --check` 通过，并检查项目自有测试目录、对应 CMake 目标和 README
  命令均已清除。未运行测试（测试源码已按要求移除），未上实车验证。

## 2026-09-23 — 新增方案文档：定向区域偏航（坡道关小陀螺 + 正对坡）

- 起因：用户要求"导航到特定区域（如上坡）时以指定 yaw 姿态通过，不要小陀螺"，且指出底盘自转由下位机控制。
- 改动：**纯文档，无代码/配置改动**。新增 `src/docs/yaw_region_alignment_plan_2026-09-23.md`，内容包括：
  1. 现状取证（三个断点：`path_executor.cpp:134-135` 算得出 ω、`ros2_comm.cpp:49-54/180-186` 只发 vx/vy/nav_state 不出桥、`/cmd_spin` 全仓无发布者且底盘自转由下位机内部产生；`minco_planner.cpp:1267-1300` 的坡道块触发太晚且产物同样卡在断点 ②）；
  2. 复用既有方向层语义（`map_server/utils.hpp:12-20`、`terrain_grid.cpp:105-113`，`|travel·dir| ≥ 0.85` 仅 ±31.8°）；
  3. 四层方案（方向层为区域唯一真源 + `terrain_traversal.yaml` 策略；新增 `TraversalDirector` 产出 `YawDirective`；执行器按 `CHASSIS_SPIN / NAV_ALIGN` 权限分路，`NAV_ALIGN` 时清零自转前馈、覆盖 MPC 参考 yaw、限幅限速；下行协议 v2 增 `omega`+`mode`、上行增 `chassis_status` 回执与交接互锁/降级）；
  4. 改动清单（11 项，含新增 `interfaces/msg/navigation/ChassisCmd.msg`）、三期落地与验证路径、取舍（不改固件时只能"停在区域外"）、7 条风险。
  5. 与上游 HW 参考实现（`/home/mas/下载/out_flat/`）对齐：自转是模式交接而非 yaw 指令、`chassis_cmd` 带 `omega`+`mode`、地形区 `run_up` 提前激活、区域内 ω 阻尼与朝向对齐残差、上游无绝对 yaw 设定点。
- 验证（本地，只读）：
  1. 逐条对照源码取证，行号见文档内引用；核对 `project_audit_2026-09-17.md` §2.10 与 README:99、launch:190 关于"协议只发 vx/vy/nav_state"的口径一致。
  2. 用一次性解码脚本实测两张 terrain msgpack：`lab3_terrain.msgpack`（770×347 @0.05，仅 FLAT/OBSTACLE，direction 全 0）与 `lab_map_20260921_211523_terrain.msgpack`（772×308 @0.05，同样无方向内容）⇒ 现场坡道尚未标注，方案第一步依赖 `mapping_web_ui` 标注。
  3. 上游参考由子代理逐文件核对（`control_arbitration.hpp`、`chassis_defs.hpp`、`state_machine.cpp`、`traversal_annotator.cpp`、`follow_problem.cpp`、`config__terrain_traversal.yaml`、`config__path_executor.yaml` 等），结论已写入文档 §1.4。
- 未验证：未编译、未运行、未改任何代码；协议 v2 的下位机行为、`omega` 闭环参数、`align_dist`/`yaw_tol` 等默认值均未实车标定，文档中的数值是待调默认值。

## 2026-09-22 — Foxglove 默认端口改为 8766

- 起因：本机 `mapping_web_ui/backend/mapping_server.py` 已监听 `0.0.0.0:8765`，导致随导航启动的
  `foxglove_bridge` 报 `Address already in use` 并退出。
- 改动：`mas2027_nav_bringup/launch/nav_executor_launch.py` 将 `foxglove_port` 默认值从 `8765`
  改为 `8766`；`docs/foxglove_remote_debug.md` 同步更新 bridge 启动、SSH 转发、直连、防火墙和排障命令。
- 验证：执行 Python 语法编译检查；检查 ROS 2 launch 参数展开结果中 `foxglove_port` 默认为
  `8766`；单独启动 bridge 烟雾测试，确认它在 `127.0.0.1:8766` 监听并广播 ROS 话题，测试后
  正常关闭。未重启完整导航栈，未在 Foxglove Studio 中手工检查面板数据。

## 2026-09-22 — 修复 clangd 满屏报错/无法跳转：导出并合并 compile_commands.json

- 起因：用户报"本项目 clangd 报错无法转跳"。
- 诊断（`clangd --check` 复现）：① 工作区里唯一的 `build/compile_commands.json` 只有 **22 条**（是
  mid360_driver / odom_localizer / small_point_lio 三个包的残留），项目自有 106 个 `.cpp` 基本没有编译命令；
  ② 编译库里查不到条目时 clangd 会**拿别的文件的命令来猜**（实测日志：用 `3rdparty/small_gicp/.../registration.cpp`
  的命令去解析 `minco_planner.cpp`）⇒ 缺 ROS/PCL/Qt 全部 `-I`，报 `'mas2027_nav_executor/...hpp' file not found`
  与 `use of undeclared identifier`，索引里没有正确的 TU，跳转自然失效；③ `.vscode/settings.json` 里写死的
  `--compile-commands-dir=${workspaceFolder}/build`：把 `src/` 当工作区打开时该目录**不存在**；
  ④ 同文件的 `clangd.fallbackFlags`、`ROS2.distro`、python 路径仍停留在 ROS 2 **humble** / PCL 1.12 / python3.10，
  本机是 **jazzy** / PCL 1.14 / python3.12。
- 改动：
  1. **新增 `src/tools/gen_compile_commands.sh`**：对 `build/` 下每个已配置的包重新 `cmake -S <包> -B <build>`
     并打开 `CMAKE_EXPORT_COMPILE_COMMANDS`（只重生成构建文件，**不重新编译**），再把各包 DB 合并去重成
     一份 `build/compile_commands.json`（11 份来源、146 条）。新增/删除源文件或改 `CMakeLists.txt` 后重跑即可。
  2. **新增 `src/.clangd`**：`CompileFlags.CompilationDatabase: ../build` + `Index.Background: Build`，
     与编辑器无关（换编辑器、换工作区打开方式都生效）。
  3. `src/.vscode/settings.json`：删掉写死的 `--compile-commands-dir`（交给 `.clangd`）；`fallbackFlags` 改为
     jazzy / `pcl-1.14`；`ROS2.distro` humble → jazzy，python extraPaths 改 `jazzy/lib/python3.12/site-packages`。
  4. `src/.gitignore`：忽略 `/compile_commands.json`（指向 `build/` 的本地软链，内容是绝对路径，不入库）。
  5. `src/README.md`「环境与编译」：构建命令补 `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`，并新增
     「代码跳转（clangd）」小节（生成命令、症状对照、何时需要重跑）。
- 验证（本地，CLI 等价路径）：
  1. `clangd --check` 抽查 5 个文件（`minco_planner.cpp`、`local_path_processor.hpp`、`test_local_path_processor.cpp`、
     `rog_map.cpp`、`mid360_driver_node.cpp`）：均从合并后的库拿到正确命令（头文件走"由同目录 TU 推断"），
     **不再有 `pp_file_not_found` / `undeclared identifier` 类诊断**；输出的 "N errors" 全是 `ExtractFunction` /
     `ExpandDeducedType` 这类 tweak 自检，与编译无关。
  2. **LSP 实测 `textDocument/definition` 三次跨文件跳转全部成功**：`minco_planner.cpp` →
     `clearance_gate.hpp:11`、`rog_map.cpp` → `rog_map.h:166`、`mid360_driver_node.cpp` →
     `mid360_driver_node.hpp:44`；分别在"工作区＝仓库根"与"工作区＝src/"两种打开方式下各测一遍。
  3. 覆盖度核对：项目自有 `.cpp` **106 个全部有条目**；未覆盖的 40 个全是 `3rdparty/small_gicp` 的
     demo/benchmark（本就不参与构建）。脚本可重复执行（幂等）。
- 未验证：未在 VS Code GUI 里点验（CLI 的 LSP 路径已覆盖同一份配置）；未跑全量 `colcon build`/实车。
- 提醒：**新增/删除源文件、切分支、改过 `CMakeLists.txt` 之后要重跑一次脚本**，否则新文件仍会满屏
  `file not found`；只改注释或函数体不必重跑。

## 2026-09-22 — 注释清理：删掉项目自有代码/配置里的日志类长注释（纯注释，代码零改动）

- 起因：用户要求"去除多余注释（那些很长的日志类）"。范围与力度由用户在二选一里确定：**项目自有代码 + 配置（排除
  `vendor/`、`third_party/`、`3rdparty/`）**，力度为**删掉排障叙事、每条保留 1~2 行"为什么"**。
- 改动（**只动注释文本，代码/配置 token 零改动**）：39 个文件，覆盖 `mas2027_nav_executor`（`minco_planner.cpp`、
  `local_path_processor.cpp/.hpp`、`minco_optimizer.*`、`global_path_searcher.*`、`smac_planner_2d_simple.*`、
  `clearance_gate.hpp`、`trajectory_safety_checker.*`、`path_executor`/MPC/`command_safety`、`nav_executor_node.cpp`、
  `path_planner.cpp`、测试与 `config/planner_params.yaml`）、`mas2027_perception`（`rog_map` 的
  `rog_map.cpp`/`query_adapter.cpp`/`esdf_utils.cpp`/`config.hpp`/`performance_monitor.*`、`mid360_driver`、
  `map_server/src/utils.cpp`、`small_point_lio`）、`mas2027_utils/ros2_comm`、`mas2027_nav_bringup` 的
  `config/small_point_lio_params.yaml` 与 `test/bench_closed_loop.py`。
  注释行 **1787 → 1248（−30%）**，其中中文注释 **1048 → 542（−48%）**，最长连续注释块由 44 行降到 ≤7 行。
- 删掉的：日期/场次标记（`【2026-09-17】`、`2026-09-16 14:55 那次运行` 等）、实车/台架日志原文与读数堆砌、
  多轮运行频率统计表、事故复盘与"改过又回退/已删除"的历史沿革、验证命令与对 `docs/*.md` 排障文档的逐条指引
  （这些内容本文件里已有存档）。
- **刻意保留**（压缩为 1~2 行）：取值理由与安全边界——`collision_dist` 有效硬阈值 0.26 m"不要再下调"且必须与
  `node.rog_map_clearance` 一致、`kEsdfJitterTolerance` 四道净空门必须同值（"切勿再下调"）、
  `esdf_max_cost ≥ esdf_weight`（否则势场饱和、梯度为 0 → 贴墙切内角）、`safe_dist` 软目标必须比硬判据高
  出余量、`unknown_as_occupied` 两处必须一致、"切勿再引入种子门净空硬否决开关"、"切勿改回全向 Kino 状态格点
  搜索"、MPC 加速度锚点"切勿清零"、`kMonitorClearanceTolerance` "切勿接近车体半径"；以及单位/量纲/坐标系约定、
  Doxygen 结构与 `verdict=` 条目。
- 未改动的文件：经逐文件核对本来就没有日志类注释，故保持原样——`rog_map/include/rog_map_ros/rog_map_ros2.hpp`
  （189 行中文注释全是 1~3 行 "why"）、`rog_map_node.cpp`、`mid360_driver_node.hpp/.cpp`、`merge_failover.hpp`、
  `mid360_driver.hpp`、`pcd2esdf_node.cpp`、`measure_lidar_mount.py`、`mpc_solver.cpp` 等。
- 验证（本地，离线）：
  1. **机械闸门**（`.scratch/comment_cleanup/verify.py`，一次性工具，按约定留在 `.scratch/` 不进仓库）：C++ 用
     `g++ -fpreprocessed -dD -E -P` 做编译器级去注释、Python 比对 `ast.dump`、YAML 比对 `yaml.safe_load`，
     逐文件比对 `git HEAD` 与工作区 ⇒ **39/39 代码骨架完全一致**（即改动前后除注释外逐 token 相同）；
     期间发现并回退了一次"顺手跑了 clang-format"的改动（`path_planner.cpp`，已还原后手工重做注释）。
  2. **增量编译**：`cmake --build build/<pkg>` 对 `ros2_comm`、`rog_map`、`map_server`、`mid360_driver`、
     `small_point_lio`、`mas2027_nav_executor` 全部 **rc=0**（仅剩 `fmt` 既有的 deprecation 警告）。
  3. 复核了本文件与 `rog_map/README.md` 明确引用过的关键注释仍在（上列"刻意保留"项逐条 grep 确认）。
- 未验证：未上实车、未跑 `colcon test` 全量用例、未重启节点（纯注释改动，无行为变化，故未做功能回归）。
- 顺带发现（**未改动**，非本次引入，`git HEAD` 即如此）：① `planner_params.yaml` 现值
  `surface_height_delta_max: 0.2`，而本文件 2026-09-22 那条"0.1 → 0.15 并补 9 行注释"的记录与该行现状
  （HEAD 即为 0.2、上方没有那 9 行注释）不一致；② 本文件 2026-09-16 记录过的 `fill_occ_min`"切勿取 9 及以上"
  醒目警告在 HEAD 的 `planner_params.yaml` / `rog_map` 代码里已经找不到（只剩 `fill_occ_min: 8` 与
  `denoise_occ_max: 0` 两个裸值）。两处都需要时另行核对补齐。

## 2026-09-22 — 配置：`projection.surface_height_delta_max` 0.1 → 0.15（桌子被判障碍）

- 起因：用户报桌子被 `/rog_map/layer_value` 判成障碍（100），而桌面下沿 0.7 m、车高 0.5 m 本可以钻过去。
  用户在三选一里明确选了"只改这一行"。适用于**柱内只有桌面那一层**的薄板桌（span 0.10~0.15）。
- 改动（**一行配置 + 注释**，无代码改动）：`src/mas2027_nav_executor/config/planner_params.yaml` 的
  `planner.rog_map.projection.surface_height_delta_max: 0.1 → 0.15`，并在该行上方补了 9 行注释，写明
  ① 为什么（`height_delta = (最高占据层号 − 最低占据层号) × resolution`，阈值只在 resolution 的整数倍上
  有意义：0.10 = 层号差 2、0.15 = 3、0.20 = 4；原值容不下被打厚到 3 层的桌面 ⇒ 落进 AMBIGUOUS 死区）；
  ② **代价与回退**（同一分支是 2026-09-16"真墙在 ROGMap 里消失、规划器穿墙后急停"的机制，放宽一格 =
  把掠射角薄墙切片的可通行上限从 2 层抬到 3 层；发现真墙变 66 就改回 0.1 并重启）；
  ③ 上限硬约束（必须 < `tunnel_height_delta_min` 0.25 与 `wall_height_delta_min` 0.80，否则
  `config.hpp:313-346` 抛异常、节点起不来）；④ 判读工具（RViz `/rog_map/layer_height_delta` 的
  intensity = 柱内 span；`.scratch/table_probe.py --what-if surface_height_delta_max=0.15`）。
  同步更新 `src/mas2027_perception/rog_map/README.md` 参数表里该行的现值。
- 为什么不是改 `tunnel_*`：中空档要求柱内**上下都有占据**且 `ratio ≤ 0.45`；桌面柱里只有桌面那一层时
  span（0.05~0.15）够不到 `tunnel_height_delta_min(0.25)`，改 tunnel 无效。用户桌子的实际 span 未实测。
- 验证（本地，离线）：
  1. 从**安装路径**读回 yaml（`install/.../config/planner_params.yaml`，确认仍是源文件的软链，
     `realpath` 指向 `src/.../planner_params.yaml`）：`surface_height_delta_max = 0.15`（float），
     全文该键只出现 **1 次**（无重复键）。
  2. 复刻 `config.hpp:313-346` 的四条派生校验：`0.15 < 0.25`、`0.15 < 0.80`、`0.4 ≥ 0.25`、
     `0.45 < 0.90` —— **全部通过**，节点不会因此起不来。
  3. 有效分档随之为：span ≤ 0.15 → PASSABLE；0.15~0.25 死区 → OCCUPIED；0.25~0.40 → PASSABLE(中空)；
     ≥ 0.80 → 墙。
- 未验证：**未上实车、未重启节点**（参数只在 `PathPlanner` 构造函数里读一次，必须重启才生效）；
  **桌子的真实 span 未实测**，所以 0.15 是否够覆盖仍是推断——若桌面被打厚到 4 层（span 0.20）则仍会
  判 OCCUPIED，需要再抬到 0.20（上限 0.24）；**放宽后的穿墙风险未在真实场景验证**，上车后必须斜看
  已知实墙仍是 100（`/rog_map/layer_type`）并对比 CSV 的 `reason_thin_surface` 是否暴涨；
  桌子若在 PGM/terrain msgpack 里也标注为障碍，本次改动不解决那道门。

## 2026-09-22 — 更正参数查询的节点名：rog_map 参数在 `/nav_executor_planner`，不是主节点

- 起因：用户问 `ros2 param get` 怎么用。核对发现上一轮 README 的「启动与检查」把节点写成了
  `mas2027_nav_executor_node`，**是错的**：一个进程里有**两个节点** —— 主节点是
  `Node("nav_executor")`（`nav_executor_node.cpp:56`），而 `planner.*` 与 `planner.rog_map.*` 全部声明在
  `PathPlanner` 自己创建的 `LifecycleNode("nav_executor_planner")` 上（`path_planner.cpp:20-28`），
  README 里查的那条参数属于后者。本文件 2026-09-16 的旧条目里其实记着正确用法
  （`/nav_executor_planner planner.minco_optimizer.safe_dist`），是本次重写 README 时抄错了。
- 改动（纯文档）：`src/mas2027_perception/rog_map/README.md` 的「启动与检查」补上两节点对照表，
  命令改为 `ros2 param list|get|dump /nav_executor_planner`，并写明：
  ① 参数在 `PathPlanner` **构造函数**里声明，进程一起来即可查，不必等生命周期 configure/activate；
  ② ROGMap 无 on-set-parameters 回调 ⇒ `ros2 param set` 返回成功但行为不变，改 yaml 必须重启；
  ③ "确实生效"要两条都过：启动日志 `[ROGMap Config] loaded ...`（`config.hpp:501`）+ 行为侧证据
  （分类阈值看 `/rog_map/layer_type` 翻转与 CSV 的 `reason_*` 列，`scan_z_*` 看 `projection_z_layers`，
  `update_period_ms` 看同名列，`dirty_column_enable` 看 `dirty_column_enabled`/`full_reason_*`，
  `visualization.rate` 看 `/rog_map/layer_type` 的 hz，`field.max_distance` 看 `/rog_map/field` 的
  intensity 上限，`decay.clear_time` 看 `/rog_map/decay_cells` 的 intensity 上限）。
- 验证：节点名取自源码构造处（`nav_executor_node.cpp:56`、`path_planner.cpp:20`）与旧条目里的实查记录
  （`/nav_executor_planner` / `/nav_executor`）互相印证；未跑节点实查（本轮无运行环境），未跑构建
  （纯文档）。README 结构复核：两节点表在「启动与检查」内，命令块与后文 FAQ 未被破坏。
- 未验证：未在实车用 `ros2 param list` 复核参数全集是否需要额外命名空间（launch 未加 namespace 与 remap）。

## 2026-09-22 — 探针加 `--what-if`：用实测柱内占据层数回答"改 tunnel 那一行有没有用"

- 起因：用户问 `projection.tunnel_height_delta_min/max` 与 `tunnel_occupancy_ratio_max` 是不是该改的地方。
  结论是"要看柱子形状"，而 `classifyCell()` 判断中空档需要的 `竖直占据率 = (occupied_count-1)*res/span`
  在已发布话题里读不到（`layer_height_delta` 只有 span 与最高点，`layer_confidence` 是观测数），
  于是把这一项做成可实测的。
- 改动（`.scratch/table_probe.py`，一次性探针，不进仓库）：
  1. 新增订阅 `/rog_map/occupied`。它与 `/rog_map/occupied_raw` 实际是同一份数据
     （`rog_map_ros2.hpp:589-598` 都用 `collectOccupiedForViz` 的 occ_map，未膨胀），按投影 z 窗口
     过滤后逐格统计占据层数 → 得到真实 `occupied_count`。
  2. 新增 `classify_span()`：复刻 `classifyCell()` 的四档分支，但 ratio 用实测层数，因此能预测
     "把这几个阈值改成 X 之后这一格会变成什么"；并打印模型与 `/rog_map/layer_type` 的一致率
     （低于 90% 时明确警告：可能有未被重新命中的陈旧占据体素或补洞/迟滞改过结果，此时 what-if 仅供参考）。
  3. 新增 `--what-if K=V[,K=V]`（可重复），只接受 `classifyCell` 用到的 6 个阈值；会先跑
     `config.hpp:313-346` 的派生校验，非法组合直接标"节点会抛异常起不来"；对每格给出
     "翻成 PASSABLE / 仍 OCCUPIED + 卡在哪一条"，全 0 翻时明确写"别白改"。
- 验证：`table_probe_selftest.py` 扩到 **8/8 通过**（新增 E1/E2 中空桌 span=0.65 → 只改
  `tunnel_height_delta_max` 翻 441 格、只改 `surface` 翻 0 格且原因是 `span > tunnel_max`；
  F 薄板桌 span=0.15 → 改 `tunnel_max` 翻 0 格、原因是 `span < tunnel_min`；G `surface ≥ tunnel_min`
  被校验拦住）。合成场景里模型与图层分类一致率 100%。
- 未验证：仍未上实车、未对真实 ROS 图跑过；`/rog_map/occupied` 在 `decay_active_list_en=true`（当前取
  默认值）下只含活跃表体素，长时间未被重新命中的陈旧占据不会出现 —— 这正是 what-if 一致率告警要覆盖的情形。

## 2026-09-22 — rog_map README 参数章节重写为完整参数表（现值 + 默认 + 启动校验）

- 性质：**纯文档改动**，未动任何代码与配置。用户要“rogmap 各参数”，把散在 `config.hpp`、
  `planner_params.yaml` 与各实现里的口径收敛成一份可查的表，落进模块自己的 README。
- 改动：`src/mas2027_perception/rog_map/README.md` 的「⚙️ 关键配置」整段重写，按功能分组：
  地图几何与滑动窗口 / ROS 回调与可视化 / 概率更新与 raycast / 衰减 / 二维投影（决定
  `layer_value`）/ 二维距离场 / 三维 ESDF / 性能与诊断 CSV / 已接线但无效的参数。每项给出
  **现值（本仓库 yaml）+ 内置默认（`config.hpp`）+ 说明与坑**，yaml 里没有的项明确标注“未设置（取默认）”。
- **顺带修正的旧文档错误**（都是会误导现场调参的）：
  1. 参数位置写成不存在的 `src/navigation/navi2_bringup/params/sentry1.yaml` → 实为
     `src/mas2027_nav_executor/config/planner_params.yaml` 的 `planner.rog_map` 子树
     （`path_planner.cpp:25`）；补上「install 里是软链、改完不用重编，但没有 on-set-parameters
     回调所以必须重启节点」与「独立跑 `rog_map_node` 时前缀是 `rog_map.`」。
  2. `map_size` 写成 `[10, 10, 1.5]` → 现值 `[10, 10, 2.5]`。
  3. 衰减段称“当前比赛配置 `active_list_enable: false`” → **yaml 根本没设这一项**，实际取默认
     `true`（`config.hpp:373`）。
  4. 投影范围写成 `-0.2 / 1.5` → 现值 `-1.2 / 2.75`，且 `scan_z_relative_to_robot: true`
     （两项是相对 `/Odometry` z 的偏移）。
  5. 新增判定链表（`classifyCell()` 的 span 四档）与派生校验清单（`config.hpp:313-346`），并显式标注
     “薄面判据同时是 09-16 真墙在 ROGMap 里消失的同源机制”。
  6. `/rog_map/layer_value` 补上“实际是二值 mask（100=障碍）”；`/rog_map/layer_height_delta`
     补上“z=柱内占据最高点、intensity=span”的语义（判读分类分支最直接的一条）。
  7. 常见问题表新增三行：桌子/门楣这类“上方有面、下方能过”的结构、真墙“消失”、建图频率跟不上点云；
     另新增“节点启动即死、日志为空”的参数校验清单与“无效参数”一节（`debug.layer_pub_enable` /
     `debug.field_pub_enable` / `debug.pub_rate` 全仓库 0 引用）。
- 验证：逐项对照 `config.hpp`（load 键名/默认值/校验）、`planner_params.yaml`（用脚本展开 rog_map 全部
  43 个叶子键取现值）、`projection_layer.cpp`、`prob_map.{h,cpp}`、`rog_map.cpp`、
  `query_adapter.cpp`、`trajectory_safety_checker.cpp` 核对；README 改后用标题层级 grep + 尾部读取确认
  结构完整、旧引用（`sentry1`/`navi2_bringup`）已清零。
- 未验证：未跑构建与 `ctest`（纯文档，无编译产物受影响）；README 里的行号引用（如
  `config.hpp:313-346`、`prob_map.cpp:938`）随上游改动可能漂移，属预期。

## 2026-09-22 — 桌子被判成障碍：定位判定链 + 新增现场量 span 的探针（**未改任何参数/判据**）

- 起因：用户报"桌子被识别为障碍物，但桌子下面其实可以通行"。现场尺寸由用户提供：**车体最高点约 0.5 m，
  桌面下沿约 0.7 m**（净空余量 0.2 m，物理上确实钻得过去）。
- **读码结论（本次未改行为，只把机制写成文档）**：`/rog_map/layer_value` 是二值 mask
  （`rog_map_ros2.hpp:874` `fillLayerMaskGrid`：mask==0 → 100），桌子显示 100 只有一个来源 ——
  `CellType::OCCUPIED`（`projection_layer.cpp:314`）。`classifyCell()`（`projection_layer.cpp:17-79`）
  **只用柱内占据 z 跨度 span 分类**，当前 `planner_params.yaml` 的实际分档是：
  `span ≤ surface_height_delta_max(0.10)` → PASSABLE；`span ≥ wall_height_delta_min(0.80)` 且竖直
  占据率 ≥0.90 → OCCUPIED(墙)；`0.25 ≤ span ≤ tunnel_height_delta_max(0.40)` 且比率 ≤0.45 →
  PASSABLE(中空)；**其余 → OCCUPIED(AMBIGUOUS)**。⇒ 桌面若因雷达噪声/LIO z 抖动在栅格里被打厚到
  3~4 格（0.15~0.20 m），就落进死区变成障碍格；而"抬 `surface_height_delta_max`"这个旋钮
  **正是 09-16 那次"真墙在 ROGMap 里消失"的机制**（见本文件 2026-09-16 条目，掠射角薄墙切片被判
  PASSABLE），因此不能盲调。`passable_as_free` 只改 value 不改 mask，改它解不掉本问题；
  `fill_occ_min`/`denoise`/`obstacle_hold_time` 与本现象无关。
- **新增诊断工具（脚本，不改运行时行为）**：`.scratch/table_probe.py`（一次性探针，按仓库惯例留在
  `.scratch`，不进仓库）。它同时订阅 `/rog_map/layer_type`（33/66/100/-1）、`/rog_map/layer_value`
  与 `/rog_map/layer_height_delta`（z=该柱占据最高点，intensity=span），以车身（`/Odometry`）为心或
  按 `--roi` 取一片，直接打出：类型构成、OCCUPIED 格的 span 分档直方图、"车顶以上（默认
  `--car-height 0.5`）非墙结构"的 span 中位/p95 与底面离地高度、最近若干格明细，并在最后给一句判读：
  span 全落在 AMBIGUOUS 死区 ⇒ 走 A（`surface_height_delta_max` 0.10 → p95 向上取整到体素，且必须
  < `tunnel_height_delta_min`，`config.hpp:337` 会校验）；p95 ≥ `tunnel_min` ⇒ 抬阈值救不了，需要
  C（桌下净空判据，尚未实施）；车顶以上有结构但一格 OCCUPIED 都没有 ⇒ 不是投影层判的障碍。
  参数从 `src/.../config/planner_params.yaml` 读取（`install` 里那份是该文件的软链，改 src 即生效）。
- **同时明确了一条容易漏的旁路**：静态地形图是**独立的第二道门**。terrain msgpack 是纯二维 cost、
  没有 z/高度语义（`terrain_map_query.hpp:59-63`，`navigation_map.yaml:8` 当前指向
  `lab_map_20260921_211523_terrain.msgpack`），`validateTrajectory` 与 command safety 都会否决穿过
  它的轨迹。⇒ 即使 ROGMap 把桌子放开，只要桌子在 PGM/terrain 里也是障碍，仍会复现 09-16 那种
  `Terrain rejection` + `Braking`。二者要一起看。
- 验证：`python3 -m py_compile .scratch/table_probe.py` 通过；`.scratch/table_probe_selftest.py`
  造三张合成投影图（A 桌面 OCCUPIED/span=0.15、B 桌面 PASSABLE、C 桌面 OCCUPIED/span=0.30，另加一面
  ROI 外的 span=1.5 远墙）离线跑 `report()`：**4/4 通过**（A 判读 `0.1 → 0.15`、B 判读"投影层没把
  它判障碍"、C 判读"走 C"、车高改 0.9 m 后不再把 0.7 m 桌面当候选），远墙未被算进桌面候选。
  期间修掉两个自身缺陷：Jazzy 的 `read_points` 返回结构化数组（不是元组）；`0.15/0.05` 的浮点误差
  会让向上取整多一格。
- **未验证 / 未做**：① **未改任何参数与判据**，`surface_height_delta_max` 仍是 0.10；② 探针**未上
  实车、未对真实 ROS 图跑过**（只有离线假消息自测），它依赖可视化已发布（`visualization.enable`
  且层有订阅者）；③ 桌子到底落在哪一档**仍未知**，等现场读数；④ 未确认桌子在 PGM/terrain msgpack
  里是否也是障碍；⑤ 方案 C（薄面只有在"下方实测自由净空 ≥ 车高"或"下方根本没有空腔"时才 PASSABLE）
  **未实施**。

## 2026-09-22 — 地图选择收敛为 YAML，原点直接读取 Nav2 地图元数据

- 新增 `mas2027_nav_bringup/config/navigation_map.yaml` 作为导航地图的唯一选择入口，只需配置
  `localization_pcd`、`occupancy_yaml`、`terrain_msgpack` 三个相对包 share 或绝对路径；
  `nav_executor_launch.py` 不再硬编码地图名、PCD、terrain 或 `origin_x/origin_y`，也删除了容易形成
  第二配置源的 `map_pcd` launch 参数。定位器通用 `params.yaml` 同步删除默认 PCD 路径，由 bringup
  统一注入。
- `map_server` 新增必填 `map_yaml_path`，启动时从 Nav2 地图 YAML 读取 `origin: [x, y, yaw]` 和
  `resolution`，并读取 YAML 引用的 PGM 检查宽高。它会将 PGM/YAML 的宽、高、分辨率与 terrain
  msgpack 交叉校验，错误组合立即拒绝启动；删除可手填且容易漂移的 `origin_x/origin_y` 参数。
  当前栅格查询是轴对齐实现，因此非零 YAML origin yaw 也会明确拒绝，要求控制台先成组旋转所有产物。
- 接入当前建图产物：确认导航仓库中的 `lab_map_20260921_211523.pcd` 与 mapping_web_ui 原文件
  字节一致，复制同名 PGM、YAML、terrain msgpack 到 `mas2027_nav_bringup/map/`，并将
  `navigation_map.yaml` 切换到该组。安装后 `/cost_map` 实测为 `772x308`、`0.05 m/px`，原点自动读取
  为 `(-5.2370148, -7.8741794)`。
- 烟测脚本 `smoke_goal.py`、`smoke_goal_motion.py`、`bench_closed_loop.py` 改为传入/按文件名推导
  `map_yaml_path`，不再硬编码 lab3 原点；基础烟测新增 `--goal x,y`，便于不同地图选取有效自由空间目标。
  README 已更新为“复制四份产物、只改一个 YAML、重建 bringup”的换图流程。
- 验证：
  1. `map_server`、`mas2027_nav_bringup`、`mas2027_nav_executor` Release/symlink-install 构建通过；
     `yaml-cpp` 使用非弃用的命名 target，最终构建无该警告。
  2. 安装产物中的新地图启动通过，`ros2 topic echo /cost_map --field info --once` 返回
     `width=772`、`height=308`、`resolution=0.05`、`origin=(-5.2370148,-7.8741794)`。
  3. 故意用 `rmuc.yaml` 搭配 `lab3_terrain.msgpack` 时按预期 fail-fast，错误准确报告
     `439x635` 与 `770x347` 尺寸不一致。
  4. `ctest --test-dir build/mas2027_nav_executor --output-on-failure` **8/8 通过**；三份受影响 Python
     脚本 `py_compile` 通过；`ros2 launch ... --show-args` 通过且不再暴露 `map_pcd`。
  5. 原 lab3 目标烟测仍通过（52 个 MINCO 轨迹点、48 个全局路径点）；新地图用其有效自由空间目标
     `(1.0, 0.0)` 通过（38 个 MINCO 轨迹点、15 个全局路径点）。旧固定测试目标
     `(2.56226, 0.437201)` 在新 terrain 上会穿过满代价格，安全校验正确拒绝，故不作为新地图烟测目标。
  6. 按用户要求删除本次 `.scratch/smoke_run` 临时配置与 CSV；未连接雷达/底盘，未做实车定位、
     动态障碍和运动验收；本次不涉及浏览器。

## 2026-09-22 — nav_executor launch 默认同时启动 foxglove_bridge

- 需求：启动 `nav_executor` 时一并起 Foxglove bridge，免去另开终端。
- 改动：
  - `mas2027_nav_bringup/launch/nav_executor_launch.py` 用
    `FrontendLaunchDescriptionSource` include 官方
    `foxglove_bridge/foxglove_bridge_launch.xml`；新增
    `use_foxglove`（默认 `True`）、`foxglove_address`（默认 `127.0.0.1`，配合 SSH
    隧道）、`foxglove_port`（默认 `8765`）。
  - `package.xml` 增加 `exec_depend`：`foxglove_bridge`。
  - `docs/foxglove_remote_debug.md` 改为以随栈启动为主路径，并提示勿与独立
    bridge 双开（端口冲突）。
- 验证：`python3 -m py_compile` 通过 launch；未上实车连 Foxglove。

## 2026-09-22 — 删除二维动态代价层，在线障碍统一交给 ROGMap

- 背景：生产 launch 已长期将 `map_server` 的动态障碍检测旁路，但它仍每 500 ms
  发布全零 `/dynamic_cost_map`；`nav_executor` 同时把这个空图当作目标接纳、全局搜索、
  MINCO 轨迹验收和 MPC 指令前视的必要输入。所谓“新鲜度”本来是二维动态图为权威
  障碍源时的 fail-closed 保护（防止发布器卡死后无限使用过期的“空闲”格）；在当前
  ROGMap 独占实时障碍的架构中，它只是冗余心跳门，并曾因超时边界导致周期性清零指令。
- 导航执行器彻底去耦：
  - 删除 `node.use_dynamic_cost_map`、`node.dynamic_map_timeout_s`、
    `node.topics.dynamic_cost_map_sub`、订阅器、`TerrainGrid::DynamicSnapshot`、动态栅格合并与
    `/nav_executor/debug/dynamic_obstacles`。
  - `PathPlanner`、`GlobalPathSearcher`、`MincoPlanner`、`checkCommandSafety()` 不再等待/读取
    二维动态图或其时间戳。静态 `/cost_map` + `/direction_map` 仍是目标与全局拓扑的
    硬约束；ROGMap 在在线快照、局部绕行/停车前缀、轨迹发布前验收、20 Hz 监视和
    0.35 s MPC 指令净空前视中仍 fail closed。
  - 新鲜度的安全目的保留，但改为读取 ROGMap 自己的快照时间戳：
    `MapQueryInterface::snapshotStampSeconds()` / `QueryAdapter` 暴露最后一次完成地图更新的
    ROS 时间，`node.rog_map_timeout_s: 0.5` 在目标接纳和 MPC 指令门同时检查。点云断流或
    ROGMap 工作线程停滞时会 fail closed，无需 ROGMap 另发一个兼容心跳话题。
- 静态地图服务精简：`map_server_node.cpp/.hpp`、`CMakeLists.txt`、`package.xml` 删除点云订阅、
  先验 PCD/KdTree 差分、地面分割、时间保持、动态膨胀、PCL/Eigen/TF 依赖和
  `/dynamic_cost_map` 发布；节点现在只加载 terrain msgpack 并发布 `/cost_map`、`/direction_map`。
  同时删除已无消费者的动态图烟测脚本和两份 PCD fixture。
- 消除话题语义歧义：原 `/nav_executor/global_path` 实际发的是 MINCO 局部优化轨迹，现将
  参数和话题分别改为 `node.topics.minco_path_pub` / `/nav_executor/minco_path`；真正的全局搜索
  折线仍为 `/nav_executor/global_plan`。RViz、烟测和 README 已同步。
- 烟测/文档：`smoke_goal.py`、`smoke_goal_motion.py`、`bench_closed_loop.py` 改为以静态
  `/cost_map` 判定地形就绪，不再等待动态心跳；README 明确 `map_server` 仍用于静态地形，
  实时障碍则只由 ROGMap 处理。`docs/change_device.md` 标记旧动态层段落为历史记录，
  `docs/foxglove_remote_debug.md` 更新当前栅格话题。
- 验证：
  1. `map_server`、`rog_map`、`mas2027_nav_executor`、`mas2027_nav_bringup` 四个受影响包均
     `colcon build --symlink-install` 构建通过；`rog_map` 仍有既存 PCL/FLANN CMake 与 fmt enum
     deprecated 警告，无编译错误。
  2. `ctest --test-dir build/mas2027_nav_executor --output-on-failure` 全部 **8/8 通过**，覆盖静态
     terrain 查询、ROGMap 快照超时闭锁/指令净空门、局部绕行/停车前缀和轨迹安全。
  3. `ROS_DOMAIN_ID=226 .../smoke_goal.py <terrain> <config>` 端到端通过：
     `52 trajectory poses, 48 global plan poses, marker width 0.150 m`，全程无二维动态图。
  4. 运行态 ROS graph 确认 `map_server` 只发 `/cost_map`、`/direction_map`；`nav_executor`
     订阅列表中无 `/dynamic_cost_map`，参数列表中也无二维动态层开关/超时，并正常发布
     `/nav_executor/minco_path`；`node.rog_map_timeout_s` 运行值确认为 `0.5`。
  5. `py_compile` 通过本次改动的 launch 与 3 个烟测/基准脚本。
  6. `mas2027_nav_bringup` 包级 linter 仍有两类**既存、与本次无关**的失败：
     `nav_executor_launch.py` 无版权头，`measure_lidar_mount.py` 有 49 个 pep257 中文标点/格式问题；
     `lint_cmake` 和 `xmllint` 通过。未上实车，未做雷达动态障碍运动验收。

## 2026-09-21 — 删除与 `/home/mas/mapping_web_ui` 功能重合的建图工具

- 背景：三维建图控制台 `/home/mas/mapping_web_ui` 已独立承接全部建图功能（累计
  `/cloud_registered` → PCD + PGM/YAML、PGM 笔刷/矩形/画线修图、map 原点与 +X 拖拽定义并成组
  变换 PCD/PGM/terrain、PGM→terrain msgpack 含坡地/台阶/飞坡方向标注、建图链路节点启停）。按用户
  要求删除本仓库中与之功能重合的部分，**保留里程计、驱动、机器人模型**。
- 删除（5 项）：
  1. `mas2027_utils/pcd2pgm/`（整包）：PCD 按 Z 带通切片→Nav2 占用图。控制台保存时会用同一投影
     语义（有点=占用、其余=空闲）直接从累计点云出 PGM/YAML，无需先落 PCD 再切。
  2. `mas2027_utils/pcd_trans/`（`pcd_tool.py`、`field_align.py`、`visual_point_cloud_lab.html`、
     `README.md`）：离线 PCD 刚体变换、格式转换、网页点云查看器与场外扫描打点对齐。控制台"map 坐标系"
     工作区对 PCD+PGM+terrain 做成组刚体变换并记录 `source_to_map`，自带 WebGL 三维点云查看器。
     注意 `field_align.py` 是上一版末新增、**尚未提交**的文件，删除后无法从 git 恢复
     （已在删除前向用户明确提示并获确认）。
  3. `mas2027_utils/map_edit`：空 gitlink（审计 3.18，无 `.gitmodules`，clone 后无内容），控制台的
     二维 PGM 编辑器覆盖其"修墙"用途。
  4. `mas2027_nav_bringup/scripts/save_pcd_and_make_map.sh`：存 PCD + 切片一键脚本。它依赖的
     `rm_navigation_small_point_lio_launch.py` 已不存在（审计 §1.7），本就不可用。
  5. `mas2027_perception/map_server/scripts/pgm_to_terrain_msgpack.py`：PGM/YAML→terrain 转换。控制台
     的"生成 terrain MSG"同格式，且能标注平地/障碍/斜坡/各级台阶/飞坡与方向（原脚本只有 FLAT/OBSTACLE、
     direction 恒 0）。**`map_server` 包本身保留**，它是在线地形服务节点。
- 保留：`small_point_lio`（里程计）、`mid360_driver`（驱动）、`mas2027_robot_description`（模型）；
  运行链路 `odom_localizer`、`rog_map`、`map_server` 节点、`mas2027_nav_executor`、`tf_maintainer`、
  `ros2_comm`；与建图无关的 `data_analyzer`、`rotmat_cal`、`measure_lidar_mount.{py,sh}`、
  `record_lio_bag.sh`；以及控制台不具备对应能力的 `pcd2ele`（max-Z 高程图）与 `pcd2esdf`（离线 ESDF）。
- 同步清理引用：
  - `mas2027_nav_bringup/CMakeLists.txt`：`install(PROGRAMS ...)` 去掉换图脚本。
  - `mas2027_perception/map_server/CMakeLists.txt`：去掉 terrain 转换脚本的
    `install(PROGRAMS ...)`（`scripts/` 目录已空）。
  - `README.md` **地图更新**：改为控制台建图/换图流程，写明产物在控制台自己的 `data/` 下、需拷进
    `mas2027_nav_bringup/{pcd,map}/`，以及换图后要同步的三处路径（launch 的 `map_pcd`、
    `terrain_map_path`、`odom_localizer` 的 `map.prior_pcd_file`）。
  - `odom_localizer/config/params.yaml`：先验 PCD 坐标系注释改写为"控制台直接从 `/cloud_registered`
    累计，导出即该坐标系"，不再提 `T_odom_from_internal`；并保留"别把 LIO 内部世界的 `scan.pcd` 指过来"。
  - `pcd2ele/README.md`、`pcd2esdf/README.md`：占用图来源改指控制台，并去掉指向已不存在的根 README
    **离线静态地图** 章节的引用。
  - 构建产物：删除 `build/pcd2pgm`、`install/pcd2pgm`，以及 `install/` 下指向已删源码的两条悬空软链
    （`map_server/lib/map_server/pgm_to_terrain_msgpack.py`、`bringup/lib/.../save_pcd_and_make_map.sh`）。
  - 历史文档 `docs/project_audit_2026-09-17.md` 与本文旧记录中的相关描述按约定不改写。
- 验证：
  1. `colcon build --symlink-install` 全仓 **14 个包通过**（`pcd2pgm` 已从构建列表消失），无其他包
     依赖被删项（`package.xml`/`CMakeLists.txt` 全仓 grep 无残留）。
  2. `map_server` 端到端烟测通过：`ROS_DOMAIN_ID=231 python3
     mas2027_perception/map_server/test/smoke_dynamic_cost_map.py
     mas2027_nav_bringup/map/lab3_terrain.msgpack
     mas2027_perception/map_server/test/fixture_static_floor.pcd`
     → `dynamic_cost_map smoke passed: occupied=388`。
  3. `ros2 launch mas2027_nav_bringup nav_executor_launch.py --show-args` 正常，`map_pcd` 默认仍解析到
     `share/mas2027_nav_bringup/pcd/lab3.pcd`。
  4. 全仓 grep（排除历史文档）无 5 个目标的残留引用；`install/` 下无本次相关的悬空软链。
  5. **未测试**：控制台本身的建图/编辑/换图流程（本次未改该仓库）；未上实车、未跑 `smoke_goal.py`
     与闭环基准。另外发现两条**既存**悬空软链与本次改动无关：
     `install/mas2027_robot_description/.../meshes/LakiBeam.STL`（源码树只有 `base_link.STL`/
     `mid360.STL`）与 `install/small_point_lio/.../config/unilidar_l2.yaml`。

## 2026-09-20 — 新增 `pcd_trans/field_align.py`：场外扫描打点对齐工具（比赛现场用）

- 背景：比赛需在**观众席**（场外）扫描点云建图，而车在场上开机。`small_point_lio`
  的 `align_odom_with_gravity: true` 把 odom 位置与 yaw 在启动瞬间归零，因此
  odom 原点 = 场上开机点；扫描图原点却在场外看台，两者差十几米。而 `odom_localizer`
  的 GICP bootstrap 门限只有 2.0 m（锁定后 0.75 m），初值猜错就永远锁不上。
  需要一个离线、确定性、不依赖 GICP 收敛的对齐手段。
- 新增 `mas2027_utils/pcd_trans/field_align.py`（纯 Python，numpy 必需，
  PyYAML/matplotlib/open3d 可选）：
  1. **自带 PCD 读写**，支持 `ascii` / `binary` / `binary_compressed` 三种编码
     （含 liblzf 解压，按字段分块的 SoA 布局），**不需要 Open3D**。
     这一条是硬需求：本机没装 open3d，而 `pcd_tool.py` 读 `binary_compressed`
     必须依赖它（`pcd_tool.py:108-111` 直接报"请安装 Open3D"）。
  2. **打点**：`--pick auto|matplotlib|open3d|none`，matplotlib 走俯视散点 +
     `ginput`，支持 `--roi` 放大、`--z-mode nearest|min|zero`（点击只给 XY，
     Z 由附近点云吸附）。
  3. **求解**：`yaw`（默认，Z 轴旋转+平移的解析解）与 `full`（Kabsch/SVD 六自由度）
     两种模式，输出逐点残差与 RMS。
  4. **输出**：4x4 行主序矩阵（语义 `p_field = R·p_pcd + t`，与 `pcd_tool.py
     --matrix` 的 `p' = R p + t` 一致）、`--apply-out` 直接变换点云（无 open3d 也可用）、
     `--check-plot` 出对齐检查图。
  5. 支持"以发车点为 map 原点"的用法（把发车点当第 1 个特征点、field 填 `[0,0,0]`），
     这正是让 map 原点与车体 odom 接近的最短路径。
  6. 中文标注字体探测：matplotlib 默认字体表无中文字体，图上中文会变豆腐块。
     `configure_plot_font()` 从 12 个候选里挑可用的，并用 `FT2Font.get_char_index`
     确认**真有这些字形**；一个都没有时自动改用英文标注。
- 同步更新：
  - `mas2027_utils/pcd_trans/README.md`：补 `field_align.py` 用法与"残差小 ≠ 对齐正确"的告诫。
  - `mas2027_nav_bringup/scripts/save_pcd_and_make_map.sh` 结尾"下一步（人工）"提示：
    原文 ①`ros2 launch map_edit`（该包是空 gitlink，新克隆跑不了，审计 3.18）、
    ②`pcd_trans --tx/--yaw`（已被本工具取代）、③"换图时三处同名"指向已不存在的
    `nav2_params projection.prior_map` 与 `launch map:=`——三处全部按当前代码改正为
    `map_pcd` / `terrain_map_path` / `origin_x,origin_y`。
- 验证（全部 headless 可复现，未上实车、未在真图形界面点过鼠标）：
  1. `field_align.py --selftest` 16 项全过：合成数据复原已知 yaw 变换（角度误差
     0.006°、平移 0.0055 m）与含 roll/pitch 的六自由度变换；共线点、散布过小、
     点数不足三种退化输入均正确报警；PCD 三种编码往返；对应表往返；字体字形覆盖。
  2. 端到端回归 `.scratch/field_align_verify/verify_field_align.py` 21 项全过：
     把 `lab3.pcd` 施加已知大位移（平移 (21.7, −14.3, 2.85)、yaw −118°）伪造成"观众席
     扫描"，经 模板→打点（叠加 2 cm 噪声）→解算→变换 后，**整片点云还原中位误差
     0.0136 m**、特征点最大 0.0237 m，即精度受限于打点噪声而非算法。
  3. **矩阵方向跨工具交叉验证**：本工具产出的矩阵喂给 `pcd_tool.py --matrix`，
     两条路径结果最大差 3.98e-06 m（ASCII 精度极限），证明 `p' = R p + t` 方向约定一致。
  4. `binary_compressed` 解码器用**独立产物交叉验证**：读出的 `lab3.pcd` 点数
     59522 与头部 `POINTS` 完全一致，且按 `pcd2pgm` 的 z 带通 (0.05~1.5) 复现后，
     X/Y 最小角为 (−4.44, −7.91)，与另一个 C++ 工具产出的 `lab3.yaml` origin
     (−4.6, −7.94) 相差 0.16 / 0.03 m——四条边全部吻合。
  5. 打点逻辑用 mock `ginput` 验证（XY 取点击值、Z 正确吸附、半径外退回最近点、
     三种 z-mode）；无 `DISPLAY` 时报错清晰；CLI 非法输入（缺 `--out`、非法 mode）被拒。
  6. **未测试**：matplotlib `ginput` 与 open3d `VisualizerWithEditing` 的**真实交互
     点选**（本机无 `DISPLAY`，只验证到"事件返回值之后的全部逻辑"）；`--roi` 视窗、
     以及观众席真实扫描数据上的端到端效果——后者需现场数据才能验证。
  7. `pyflakes` 干净，`py_compile` 通过；`save_pcd_and_make_map.sh` 改动后
     `bash -n` 通过、`--help` 正常。

## 2026-09-18 — 文档精简：根 `README.md` 压缩约一半篇幅（378 → 192 行）

- 背景：上一条把 `docs/README.md` 合并进根 README 后篇幅到 378 行，用户要求"更精简"。
- 改动（**纯文档，只动 `README.md`，事实与阈值一处未变**）：
  1. 组件职责表由 4 列压成 2 列；"关键话题"表由 18 行并为 10 行（同向话题合并成一行）。
  2. 净空判据一节的 `verdict=` 表改为一行内联枚举（结论不变：`GEOMETRY` 别动阈值、
     `SEED_GATE_STRICTER` 判据不一致、`PREFIX_TOO_SHORT` 是长度问题、`TERRAIN`、`NONE`），
     深度排障改为指向 `docs/refusal_triage_2026-09-16.md`。
  3. 删除已进 change-history 的历史叙述：全向 Kino A\* 的失败数据与删除经过、
     双重阈值事故的复盘细节、编译/启动的命令展开（`colcon build` 合成一行）。
  4. 保留全部硬事实：`collision_dist`/`monitor_margin`/`replan_react_time`/`rog_map_clearance`
     的 0.28/0/0/0.28、近场放宽与第四层兜底开关、`use_ros2_comm` 默认 `True`、
     `ros2_comm` 不转发 `angular.z`、动态层默认旁路、`/nav_executor/global_path` 的历史遗留命名。
- 验证：表格列数、代码块配对（7 对）、锚点与文件链接用脚本检查通过；README 引用的路径与
  上一条记录相同（未新增引用），未重新编译、未上实车。

## 2026-09-18 — 文档合并：`docs/README.md` 并入根 `README.md` 并按当前实现改写

- 背景：用户要求"修改 readme 文档"，确认后指定"合并到根目录并进行现在代码的适配"。
  `docs/README.md` 是 2026-09-12 重构期写的架构说明，此后一直没跟上代码：
  2026-09-17 的 `docs/project_audit_2026-09-17.md` §3.10 已把它的漂移登记为"文档漂移"
  （"HW 地形图不参与轨迹优化"、话题名少 `/debug`），本轮据此整改。
- 改动（**纯文档，不改任何代码、配置、脚本**）：
  1. 删除 `docs/README.md`，其仍然有效的内容（组件职责表、TF 约束、RViz 分组、
     "独立执行器没有 Global/Local Costmap"、`ros2 topic` 快速检查）并入根 `README.md`；
     根 README 开头补一句"本文件是链路、TF、话题与排障的唯一总说明"，
     文末只保留 `docs/change-history.md` 与专题排障文档的指引。
  2. 根 `README.md` 新增 `## 组件职责`（8 个组件的输入/输出/职责）、`## 关键话题`
     （含类型与方向，标注 `/nav_executor/global_path` 这个历史遗留名字发的是 MINCO 轨迹）、
     `## TF 约束`、`## 快速检查` 四节；原有的净空判据、`verdict=` 表、RViz 折线/轨迹
     对照表、地图更新与烟测内容整体保留，只做归并和口径校正。
  3. 按当前代码校正的**事实性错误**（逐条对照代码/配置取证）：
     - "HW 地形图不参与轨迹优化，代价图/方向图只是观察通道" → 改为规划输入
       （全局搜索、轨迹验收、MPC 制动都读 `node_params.yaml` 的 `terrain_cost_sub`/
       `terrain_direction_sub`/`dynamic_cost_map_sub`）；
     - `PathPlanner` 的"A* 搜索" → 全局主搜索为 SMAC 2D（`planner.use_smac: true`），
       `false` 才退回 Astar；
     - `MINCO Trajectory` 话题由 `/nav_executor/minco_trajectory` 更正为
       `/nav_executor/debug/minco_trajectory`，并补上 `/opt_path` 与
       `/nav_executor/debug/global_plan`、`/nav_executor/global_plan` 的区别；
     - `node.rog_map_clearance` 默认值 0.30 → **0.28**（`node_params.yaml:11`，
       与 `planner_params.yaml` 的 `collision_dist: 0.28` 一致；`collision_dist` 现值
       0.28、`monitor_margin: 0.0`、`replan_react_time: 0.0` 一并写明）；
     - TF 口径：`tf_maintainer` 同时发 `map→odom` 与 `odom→base_link`
       （launch 里两个开关都为 `true`），不是只发其中一段；
     - `use_ros2_comm` 的默认值已是 `True`（launch 注释：它是 `/cmd_vel` 的唯一消费者，
       不启动会"有指令但车不动"），原文"首次上车建议保持 false"会误导，改为说明默认值
       与关闭时机；
     - 新增两条现场易错点：`ros2_comm` 的 UDP 协议只发 `vx`/`vy`/`nav_state`、**不转发
       `angular.z`**；`terrain_map_server` 的动态层由 launch 的
       `bypass_dynamic_obstacle: True` 默认旁路，`/dynamic_cost_map` 恒为全 0，
       实时避障由 ROGMap 承担（烟测脚本不设该项、走代码默认 `false`，故烟测里动态层有内容）。
- 验证（本轮只改 Markdown，未编译、未上实车）：
  1. README 中引用的 17 个路径/脚本/config 全部 `test -e` 通过（含 `smoke_goal.py`、
     `smoke_dynamic_cost_map.py`、`pgm_to_terrain_msgpack.py`、`save_pcd_and_make_map.sh`、
     `clearance_gate.hpp` 等）。
  2. 逐条对照代码取证：`node_params.yaml`（话题表、`rog_map_clearance: 0.28`、
     `dynamic_map_timeout_s: 1.5`）、`planner_params.yaml`（`use_smac`、`tolerance: 0.3`、
     `collision_dist: 0.28`、`monitor_margin: 0.0`、`replan_react_time: 0.0`、
     `failure_log_*`、`stuck_escape`）、`mpc_params.yaml`（`omega` ±2 / ±4）、
     `nav_executor_launch.py`（节点清单与开关默认值）、`tf_maintainer_node.cpp`
     （两个 TF 发布开关）、`odom_localizer_node.hpp`（`/tf_maintainer/map_to_odom`）、
     `map_server_node.cpp:67`（动态层旁路）、`ros2_comm.cpp:181-186`（只发 vx/vy/nav_state）。
  3. README 里提到的 RViz 显示名与 `nav_executor_view.rviz` 的 `Name:` 字段逐一核对通过
     （`Planning Constraints (raster diagnostic)`、`Dynamic Obstacles (cyan, current)`、
     `Global Plan (SMAC search, thick)`、`MINCO Trajectory (Path)`、`ROGMAP` 等）。
  4. `grep -rn "docs/README"` 全仓库仅剩新 README 里一句"已合并进来"的历史说明；
     `docs/project_audit_2026-09-17.md` 是当日审计记录，按"不改写旧记录"保留原引用。
- 未做 / 未验证：未运行 `colcon build`/`colcon test`（无代码改动）；未做 Markdown lint
  （仓库无此 CI）；`docs/` 下其余专题文档（`*_triage_*.md`、`nav_tuning_*`、
  `project_audit_*` 等）本轮未逐篇校对，仍按各自日期归档。

## 2026-09-18 — 清理批次②③：删死代码（STRICT 链、死字段、未调用搜索器）与重复的包内 launch/配置

- 背景：承接同日的"冗余审计"（批次①见下一条）。用户要求"只要不影响原来的效果"就继续去冗余，
  工具包（`mas2027_utils/*`、`pcd2ele`/`pcd2esdf`、`map_edit` gitlink）不动。
- 改动（删除或等价重写，全部落在默认配置下**不可达**的路径上；阈值与三道轨迹级净空门一处未动）：
  1. **STRICT 种子门整条链**：`config/planner_params.yaml` 的 `strict_seed_after_failures`；
     `minco_planner.{hpp,cpp}` 的三个成员、连续失败计数、切换 WARN 与传参；
     `local_path_processor.{hpp,cpp}` 的 `buildSeed(..., enforce_seed_clearance)` 与硬否决分支；
     `test_local_path_processor.cpp` 的对照用例。**保留**停车前缀仍在用的
     `segmentClear(..., enforce_clearance)`；`pathClear` 不再透传该参数（4 个调用点本来就都用软口径）。
  2. **死字段 / 死参数**：`MincoPlanner::getTrajectoryRemainTime()`、`opt_freq_`（连带 YAML 的
     `minco_optimizer.opt_freq`）、`global_frame_`、`backup_path_pub_`（`/backup_path` 无订阅者、
     建了发布器却从不发布）、`minco_optimizer.time_allocation_iters`（读出后无人使用）、
     `mpc_types` 的 `planner_freq` / `max_iterations`（MPC 求解器从未读取）、
     `MpcSolver::has_last_u_` 与 `hasLastControl()`（只写不读，唯一读者是测试断言）。
  3. **未调用的搜索器**：`path_planner/search/omni_kino_astar.{hpp,cpp}` + `test/test_omni_kino_astar.cpp`
     及对应 CMake 目标（生产零调用点，`global_path_searcher.cpp` 的说明注释同步更新）。
  4. **两条误导日志**（只改文案/取值来源，不改逻辑）：启动日志的 `planner_mode` 改为打印实际参数
     （原来硬编码 `"EXPLORATION"`）、`global_search` 改为打印真正生效的搜索器
     （`SMAC2D` / `Astar`，原来硬编码 `"OmniKinoAstar"`）。当前 YAML 就是 `EXPLORATION`，
     因此现场日志除搜索器名字外与改动前一致。
  5. **重复入口与重复配置**：删 `mas2027_nav_executor/launch/nav_executor.launch.py`
     （与 bringup 的 `nav_executor_launch.py` 重复，且只起执行器、不拉 map_server，
     会静默跑在"没有地形图"的状态）；删 `mid360_driver/launch/mid360_driver.launch.py`
     与其 `config/params.yaml`（与 bringup 的 `small_point_lio_params.yaml` 取值相反——
     `validate_crc` true/false、`lidar_frame`/`min_distance`/`gravity` 全不同，
     且该 launch 从 share 里 glob 一个本包**从不安装**的配置，等于用默认参数起节点）。
     真源统一为 `mas2027_nav_bringup/config/small_point_lio_params.yaml`。
  6. **连带修正**（注释/文案，无行为）：两处安装规则随目录删除调整，否则 configure/install
     阶段直接报错——`mid360_driver/CMakeLists.txt` 去掉 `INSTALL_TO_SHARE config launch`、
     `mas2027_nav_executor/CMakeLists.txt` 的 `install(DIRECTORY config launch ...)` 改为只装 `config`；
     `README.md` 与 `scripts/measure_lidar_mount.py` 里指向已删文件的路径改为指向唯一真源。
- 验证：
  1. 全工作区 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`：
     **15 个包全部通过**。（中途 `mid360_driver` 因 `INSTALL_TO_SHARE` 指向已删目录失败过一次，
     修正后通过——这也是本次唯一一次构建失败。）
  2. `colcon test --packages-select mas2027_nav_executor mid360_driver`：`mas2027_nav_executor`
     **8/8 通过**（原 9 项，删掉的正是 `omni_kino_astar` 那项）、`mid360_driver` 通过。
     全工作区 `colcon test` 仍有**与本次无关的既有 lint 噪声**（`small_point_lio` 的
     cpplint/flake8/lint_cmake 等 700+ 条），改动前后一致。
  3. 端到端 `test/smoke_goal.py`（真实 map_server + nav_executor + lab3 地图/PCD，隔离域 231）：
     `goal smoke passed: 52 trajectory poses, 48 global plan poses, marker width 0.150 m`
     —— 与批次①之后、本次改动之前的输出**逐字一致**。
  4. 等价性依据：被删代码在默认配置下均不可达（`strict_seed_after_failures` 已是 0、
     `has_last_u_` 无读者、`/backup_path` 无发布/订阅、`omni_kino_astar` 无调用点、
     `opt_freq`/`time_allocation_iters`/`planner_freq`/`max_iterations` 无读者）。
- 未做 / 未验证：
  1. 未上实车；未做改前/改后的闭环台架 A/B——台架本身双峰（见 `bench_closed_loop.py` 的说明），
     单次对照不可判读，故判断依据是"删除项不可达 + 冒烟输出逐字一致"。
  2. **有意保留**：`astar.cpp`（`use_smac:=false` 的合法备选搜索器）、
     `odom_localizer/launch/odom_localizer.launch.py`（自洽可用的独立入口，非重复）、
     `minco_utils.cpp` 里 vendored 的 `publishBackupTrajectory`（上游代码，未改 vendor）。
  3. **4 份净空判据实现的合并未做**（`trajectory_safety_checker` / `local_path_processor::segmentClear`
     / `command_safety` / `minco_planner::checkCollision`）：那是会改变行为的重构，需要专项提交
     加台架/实车验证，不符合本轮"不影响原来的效果"的前提。这是结构性重复里唯一还没治的一项。
  4. 工具包（`mas2027_utils/*`、`pcd2ele`/`pcd2esdf` 无消费者、`map_edit` gitlink、
     `save_pcd_and_make_map.sh` 的失效提示）按用户要求未动。

## 2026-09-18 — 清理批次①（零行为改动）：删 vendor 未编译子树、未引用资产与构建残留

- 背景：用户问"本项目有什么冗余"。逐项取证后按"零风险批次"执行——**不涉及任何运行时行为、
  不删任何被编译或被引用的文件**；结构性问题（感知配置双真源、包内 launch、4 份净空判据）
  与代码死代码（STRICT 链、旧死字段）留待批次②③。
- 改动（全部为删除 + 一处删除后的连带修正）：
  1. `mas2027_nav_executor/vendor/mpc/qpOASES/{examples,testing,interfaces,doc}`：CMake 只
     `file(GLOB qpOASES/src/*.cpp)` 加 `include/`，这四个子树（matlab/octave/simulink/CUTEst
     绑定 + `doc/manual.pdf` 0.79 MB）从不参与构建，合计约 2.4 MB。
  2. `mas2027_nav_executor/vendor/minco/include/{cereal,fmt}`：以全部被编译源为起点做 include
     闭包（87 个文件）后，落在 cereal/fmt 下的文件数为 **0**，合计约 2.16 MB。
  3. `mas2027_robot_description/meshes/LakiBeam.STL`：在 urdf/xacro/xml/py 中 0 引用，3.87 MB。
  4. `mas2027_perception/Odometry/small_point_lio/config/unilidar_l2.yaml`：本车为 mid360，0 引用。
  5. `mas2027_perception/rog_map/config/visualization.cfg`：ROS1 `dynamic_reconfigure` 残留，0 引用；
     `config/` 随之空掉，故同步删除 `rog_map/CMakeLists.txt` 中的 `install(DIRECTORY config/ ...)`
     ——否则 CMake 会在 configure 阶段因目录不存在直接报错（这条是删除的连带修正，不是功能改动）。
     该包的参数一律由 nav_executor 的 `planner.rog_map.*` 提供。
  6. **保留** qpOASES 的 `LICENSE`/`LICENSE.txt`/`AUTHORS`/`AUTHORS.txt`/`INSTALL`/`INSTALL.txt`：
     实测两组文件内容并不相同（上游不同版本），涉及许可，不按"重复文件"处理。
- 构建残留（均在 `.gitignore` 内、不进仓库）：删除 `build/` 下 5 个源码已删包的目录
  （`minco_planner` 20 M、`minco_controller` 19 M、`fake_vel_transform` 8.1 M、
  `waypoint_editor` 7.9 M、`pb_nav2_plugins` 4.5 M）、`install/rog_map/share/rog_map/config/`、
  以及已删 `nav2_launch` 的 `.pyc`；并 `rm -rf build/interfaces install/interfaces` 后干净重编，
  清掉已删消息 `CostMaps` 的全部生成物——审计 §2.24 当时判断只有 `install/` 有残留，
  本次实测 **build 与 install 两侧都有**。
- 验证：
  1. 全工作区 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`：
     **15 个包全部通过**（48.2 s，含 interfaces 干净重建 5.6 s、nav_executor 42.3 s）。
  2. `colcon test --packages-select mas2027_nav_executor`：**9/9 通过**；
     `colcon test-result --verbose` 为 10 tests / 0 errors / 0 failures。
  3. `find build install -iname "*cost_maps*" -o -iname "*CostMaps*"` 为空；
     `ros2 interface list` 中 `interfaces/msg/MpcPositionCommand` 仍存在、`CostMaps` 消失
     （列表里其余的 `nav2_msgs/msg/Costmap*` 属另一个包，与本次无关）。
  4. 端到端冒烟 `test/smoke_goal.py`（真实 map_server + nav_executor + lab3 地图/PCD，隔离域 231）：
     `goal smoke passed: 52 trajectory poses, 48 global plan poses, marker width 0.150 m`，与清理前一致。
  5. 体积（本次实测，`du` 口径）：删除文件 279 个、按 git 记录合计 **7.61 MB**；`vendor/` 6.6 MB → 2.2 MB；
     工作树（不含 `.git`）≈25 MB → **17 MB**，含 `.git` 55 MB → 47 MB；`build/` 558 MB → 499 MB
     （先删 5 个孤儿包目录约 60 MB，随后 interfaces 干净重建有少量回补）；`install/` 5.2 MB → 5.1 MB。
- 未做 / 未验证：未运行实车；未动任何编译单元、配置数值、话题与阈值。批次②（`pcd2ele`/`pcd2esdf`
  无消费者、`mid360_driver` 与 `small_point_lio` 配置双真源、包内 `nav_executor.launch.py`、
  4 份净空判据合并）与批次③（`strict_seed_after_failures` 整条 STRICT 链、`has_last_u_`、
  `backup_path_pub_`/`BLOCK_COMMAND`、`omni_kino_astar` 等死代码）本次保留，需要各自专项提交
  与重新构建验证；`mas2027_utils/map_edit`（无法还原的 gitlink）与
  `save_pcd_and_make_map.sh:315` 的提示也留到批次②。

## 2026-09-18 — 远点导航与"反复冷启动/挪动"结案：新增实车验收结论文档，并纠正最近两条条目

- 行为与文件：**本次只加/改文档，不改任何代码、配置、脚本逻辑**。
  1. 新增 `docs/far_goal_coldstart_final_2026-09-18.md`。以 2026-09-17 21:52–22:07 的实车
     `~/.ros/log/mas2027_nav_executor_node_7150_1789653149789.log` 作为验收：**29 个目标 28 个到达**
     （唯一没打印到达行的目标在 7 s 后被下一次点击顶替），去程 `(9.79,-0.05)` 12.6 s、
     最远 `(19.54,-2.4)` 58.7 s 到达；同时 0 次 `switched to STRICT`、0 次 `dynamic_stale`、
     0 次 `Ignoring goal`，仍有 191 次 MINCO 失败 / 39 次急停 / 24 次 `reason=clearance` 制动。
     文档逐条给出相对 `7e19968` 的**净改动**清单与证据：A 远点（软种子门、四道净空门统一 0.26、
     `esdf_max_cost` 0.5→5.0、`prior_map` 保持关、`strict_seed_after_failures: 0`）；
     B 反复启停（`node.dynamic_map_timeout_s` 0.5→1.5、MPC 打断后锚到实测车速）；
     C 目标时间戳归零、日志可观测性与回归用例；并列出 P0/P1/P2 未决项。
  2. 同一文档 §3 对 `change-history.md` 顶部两条条目逐条纠错（用户指出"最近的两次历史有犯错"）：
     - 《修"远处点导航失败 / 车原地不动"：软种子门连续失败后退回净空硬否决》：其前提
       "三层兜底根本不会被执行"被 run 7150 否证（`Repaired a ROGMap-blocked global segment with a
       local grid-search detour.` 在软种子门下照常执行且到达相邻目标）；其改动
       `strict_seed_after_failures: 3` 在 21:05 / 21:07 两次实车上把到达率打成 8/4 与 4/1；
       其台架 A/B（软门 0.00 m vs 硬门 5.22~6.31 m）来自合成回放台架（PCD 与地形图不一致），
       不能当作实车判据。
     - 《回归旧工程的连续规划语义：禁用严格种子切换、恢复 HOT_START 与时间权重》：方向与 run 7150
       一致，但三处与事实不符——(a) "恢复 HOT_START"**不是行为改动**：`7e19968` 的
       `minco_planner.cpp:1574-1585` 本来就是 `return PlanningState::HOT_START;`，本次只把
       误导性的 `Downgrading to COLD_START` 文本改成 `keeping HOT_START`；
       (b) "禁用严格种子切换"只被证明"开着更糟"，没有被证明"关掉是唯一解"，属过度归因；
       (c) 条目自称"未上实车"，真正的实车验收发生在它写完后 15 分钟的 run 7150。
  3. `.gitignore` 增加 `.scratch/`（仓库内临时目录，避免 `git add -A` 误提交一次性脚本与日志）。
- 验证：
  1. `colcon build --packages-select mas2027_nav_executor --symlink-install
     --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过（产物均为最新，无重编译）；
     `colcon test --packages-select mas2027_nav_executor --event-handlers console_direct+`
     **9/9 通过**，`colcon test-result --verbose` 为 10 tests / 0 errors / 0 failures。
  2. 文档中的全部计数与引用为本次重新抽取：run 7150 的逐目标账本（29/28）、35251/32290 的
     `switched to STRICT`（2 / 26）与 `LOCAL_SEED_INVALID`（12 / 15）、9049 的 175 条
     `dynamic_stale value=0.501 threshold=0.500`、75351 的 18 条 `Ignoring goal`、
     61759 的 8 条 `threshold=0.280`（值域 0.261~0.280）、以及
     `install/` 符号链接 → 21:27 构建的二进制与源码 21:35 的 `planner_params.yaml`
     （证明 run 7150 跑的就是本次提交的工作区状态）。
- 未验证 / 未做：
  1. 未改任何代码或阈值。P0（`strict_seed_after_failures=0` 缺正向证据、STRICT 链路已成死代码）、
     P1（锚点未做可行域裁剪、`dynamic_map_timeout_s ≤ 0` 时 fail-open、"四道门统一"依赖
     `replan_react_time=0`/`monitor_margin=0`、`allow_motion==false` 分支仍清零锚点）、
     P2（`test_local_path_processor.cpp` 净空断言 0.30→0.0 与容差 1e-6→0.10、窄口未处理）
     全部只记录在结论文档，留待专项提交决定。
  2. 未做软/硬种子门的实车 A/B；run 7150 没有留存三路速度 CSV，因此"反复启停已消除"目前只有
     日志侧证据（0 次 `dynamic_stale`、24 次净空制动但均可恢复），没有实测速度曲线对比。

## 2026-09-17 — 回归旧工程的连续规划语义：禁用严格种子切换、恢复 HOT_START 与时间权重

- 需求与证据：用户反馈当前未提交版本相较 native 最新提交和
  `/home/mas/mas_nav_2027` 出现两类回归：远点目标不出轨迹，以及行进中实速跟不上指令后
  反复“冷启动”、只能一段一段挪动；并补充
  `/home/mas/nav_opensource/navi_minco_bit` 是新旧两工程 MINCO/MPC 的真正上游。检查最新实车日志
  `~/.ros/log/mas2027_nav_executor_node_35251_1789650374567.log`：去程已到达，返程开始后先出现
  `clearance=0.255 < 0.260` 制动，连续 3 次规划失败后切到 STRICT 种子门，此后在
  `(3.78,1.62)` 附近以 `clear=0.244~0.245 / req=0.260` 持续
  `LOCAL_SEED_INVALID`，整次日志共有 48 条 `MINCO path generation failed`，返程未到达。
  三方代码对照还确认：上游与旧工程在跟踪/速度误差过大时都实际返回 `HOT_START`（虽保留了
  误导性的 `Downgrading to COLD_START` 文本），上游时间权重为 100；native 最新提交同样保持
  HOT_START、时间权重 100，并因本车先验图对齐问题关闭 `prior_map`。
- 行为改动：
  1. `mas2027_nav_executor/config/planner_params.yaml`：将
     `strict_seed_after_failures` 从 3 改为 0，保持旧工程“折线只因占据/地形/查询失败而硬拒绝，
     净空由优化后轨迹门把关”的语义；将 `penalty_weight_time` 从实验值 500 恢复为上游、旧工程和
     native 最新提交共同使用的 100，避免规划速度峰值抬高后进一步放大车速跟踪误差；
     `prior_map.enable` 保持 native 最新提交的 false，避免把已记录的 map↔odom 错位先验重新叠加
     进 ROG ESDF。发布前、20 Hz、MPC 指令和静态地形安全门均未放宽。
  2. `mas2027_nav_executor/src/path_planner/trajectory/minco_planner.cpp`：参数缺省值同步为 0；
     大位置误差和大速度误差继续返回 `HOT_START`，不再丢弃上一条轨迹的时间/路点热启动种子，
     同时把日志改成与实际行为一致的 `keeping HOT_START for trajectory continuity`。
  3. 保留本轮已经加入的 MPC 中断恢复处理：`mpc_solver.hpp` / `path_executor.cpp` 在轨迹过期、
     参考失败、求解失败或安全门否决后，用实测车速而不是零值重设下一拍加速度锚点；恢复首拍从
     车辆真实速度连续接续，速度、加速度和净空上限不变。对应回归用例位于
     `test/test_mpc_horizon.cpp`。
- 验证：
  1. `colcon build --packages-select mas2027_nav_executor --symlink-install
     --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过；`colcon test --packages-select
     mas2027_nav_executor --event-handlers console_direct+` 为 **9/9 通过**。MPC 锚点用例实测：清零时
     恢复首拍上限 0.2 m/s；锚到实测 1.5 m/s 后首拍为 1.7 m/s。
  2. 标准目标冒烟通过：`52 trajectory poses / 48 global plan poses / marker width 0.150 m`；
     20 s 闭环短程（目标 `(-0.15,-0.70)`）得到规划峰值 0.90 m/s、实测峰值 0.88 m/s、位移
     0.82 m、轨迹覆盖 903/994 拍，约 1.6 s 到达；没有大速度误差导致的 COLD_START 循环。
  3. 对静态先验做了 A/B：开启时远点复现固定死在 ROG 净空 0.245 < 0.260；关闭时该合成复现
     又因回放 PCD 与地形图不一致，MINCO 看不到 terrain `(5.15,1.91)` 的墙，候选轨迹被 native
     额外的 `edge_static` 发布门拒绝。后者说明该远点合成台架不能替代实车在线点云验证，未据此
     删除静态地形安全门或下调 0.26 m 有效净空阈值。
- 未验证 / 风险：尚未上实车复测原去返航路线。重启后应确认：① 启动日志不再出现
  `loaded prior map`；② 不再出现 `Local seed gate switched to STRICT`；③ 大跟踪/速度误差只打印
  `keeping HOT_START`；④ 返程能持续发布 `/opt_path` 且 `/cmd_vel` 不再周期性跌到冷启动首拍。
  合成远点仍不能到达的原因是 PCD/地形不一致，并非本轮安全阈值已验证可放宽，因此本轮没有改
  `collision_dist=0.28` 或共用的 0.02 m ESDF 抖动余量。

## 2026-09-17 — 修"远处点导航失败 / 车原地不动"：软种子门连续失败后退回净空硬否决

- 需求与现场证据：用户反馈"还是会反复进入冷启动；无法像最新提交的那一版做远处点导航，
  会出现 traj failed"。查最近两次运行日志（`~/.ros/log/` 下
  `mas2027_nav_executor_node_6166_1789646967183.log`（20:09–20:26）与
  `mas2027_nav_executor_node_14967_1789648111771.log`（20:28–20:31））：
  1. 20:09 那次：30 个目标 18 个到达，但去程 `(1.21,-0.10) → (7.16,2.32)` 全程
     **600 条 `MINCO path generation failed; retrying`（COLLISION=583）**、`/opt_path`
     一直为空、车 19 分钟没动；其间种子门每 2 s 刷
     `Local seed is tight but not blocked (closest clear=0.248 req=0.260 at (4.65,1.84))`，
     并伴随 8 次 `Publishing emergency stop: committed traj unsafe and replan failed`。
     发布前净空门的现场值是 `Trajectory clearance 0.246 m below required 0.260 m at (5.64, 2.27)`
     ——**只差 1.4 cm，且 MINCO 连续 600 次都做不到**。
  2. 对照"最新提交那一版"的行为（2026-09-16 21:50 那次实车，`prior_map.enable` 还是 false）：
     同一片区域的目标 `(10.32,2.13)` / `(10.36,2.38)` 10 个到 9 个，日志里每次重规划都出现
     `Repaired a ROGMap-blocked global segment with a local grid-search detour.`
     ⇒ 旧行为下"净空差几毫米的种子"会被**否决**，然后走 ROGMap 绕行把种子改写成能过的路由。
- 机制（代码级）：2026-09-17 白天把种子门改成"软"门（`local_path_processor.cpp` 的
  `segmentClear(..., enforce_clearance=false)`：净空不足不再否决种子），前提是
  "MINCO 的位置罚项会把轨迹推离障碍"。这个前提在**唯一通路本身就比 required 窄**时不成立：
  此时三层兜底（ROGMap 绕行 → 障碍前完整停车前缀 → 短距离脱困前缀）**根本不会被执行**，
  规划器每 0.5 s 重复同一个不可能成功的优化 ⇒ `/opt_path` 为空 ⇒ 执行器只能发 0 ⇒
  现场就是"车不动 + traj failed + 反复启停"。这也是台架与实车都能复现的：见下面验证第 2 条。
- 改动（**只加"退路"，不动任何净空数值**）：
  1. `local_path_processor.{hpp,cpp}`：`buildSeed` 新增 `enforce_seed_clearance`（默认 false，
     保持软种子门）、`pathClear`/`segmentClear` 新增 `enforce_clearance` 透传；硬否决时既有的
     三层兜底链原样生效。另把 `seed.dense_reject` 改为"种子门否决过就记录"（含随后被绕行/前缀
     修复成功的情形），并在硬否决只因"净空差一点"时打一条带坐标的 WARN，回答"为什么走了兜底"。
  2. `minco_planner.{hpp,cpp}`：新增 `strict_seed_after_failures`（默认 3）。连续失败达到该次数
     （≈1.5 s，`replan_period_s` 0.2 + 优化耗时）就把种子门切回硬否决，任一次局部规划成功即复位；
     切换时打一条 WARN（`Local seed gate switched to STRICT clearance after N ...`）。
  3. `config/planner_params.yaml`：`planner.minco_optimizer.strict_seed_after_failures: 3`
     （附现场依据、取值理由、`0` = 关闭的回退方式）。
  ⚠️ 四道净空门（发布前校验 / 20 Hz 监视 / MPC 指令门 / 种子门与绕行）的**阈值一个都没改**，
  本项只决定"折线净空不足算不算否决种子"。
- 验证：
  1. `colcon build --packages-select mas2027_nav_executor --symlink-install`（Release）通过；
     `ctest` **9/9 通过**，其中 `test_local_path_processor` 新增用例锁住新语义：0.50 m 走廊上
     软种子门原样放行（`clearance_only=true`），同一折线在硬否决下必须落进三层兜底之一
     （`used_dynamic_detour || stop_at_local_end || used_escape_prefix || !valid`），
     且否决原因仍是"只差净空"而不是"被堵死"。
  2. 闭环台架 `test/bench_closed_loop.py`（真实 map_server + 真实 nav_executor + 真实 lab3
     地图/PCD；`map→odom` 取 20:09 实车那次的 `(0.046,0.123,0.274, yaw −1.36°)`；起点
     `(1.211,-0.097)`、目标 `(7.26,2.28)` 与实车一致）：
     | 版本 | 有轨迹的拍 | 位移 | MINCO 失败 | Braking |
     |---|---|---|---|---|
     | 改前（`strict_seed_after_failures` 缺省即软门，40 s） | **0/1986** | **0.00 m** | 73 | — |
     | 改后（60 s，两次运行） | **2804~2808/2974** | **5.22~6.31 m** | 8~20 | 9（8 clearance + 1 dynamic_reference，**无 dynamic_stale**）|
     即"车原地不动"变成"94% 的拍都有可执行轨迹、能推进 5~6 m"。
  3. 配置副本对照（`.scratch/repro/cfg_*`，同一台架同一目标）：把 `prior_map.enable` 改回
     false、`esdf_max_cost` 改回 0.5、`penalty_weight_time` 改回 100（甚至三者同时改回，
     即"改动前的配置"）**同样是 0 位移**，证明本次回归不在那三项配置，而在软种子门本身。
  4. F3（velocity error → COLD_START）与"提交前行为"（HOT_START）做了 2×2 台架对照：
     `Downgrading to COLD_START` 条数 COLD 18/44 vs HOT 19/7，位移 COLD 5.22/6.31 m vs
     HOT 6.60/4.43 m，两边都跨过对方的值域 —— 与台架已知的双峰一致，**没有证据支持回退 F3**，
     故保留现行为并把数据留在这里。
  5. 标准冒烟 `test/smoke_goal.py` 通过：`goal smoke passed: 40 trajectory poses,
     48 global plan poses, marker width 0.150 m`；实车 CSV `.scratch/rog_map_perf_summary.csv`
     冒烟前后 md5 不变（`e8a7c844f7fd3128e17e5eab948f4580`）。
- 未验证 / 未做 / 已知残留（**都要上车确认**）：
  - **未上实车**。车上要看两件事：(a) 远点目标不再出现"几分钟刷几百条
    `MINCO path generation failed`、车不动"；(b) 新增的
    `Strict seed gate rejected the global corridor (closest clear=X req=Y at (x,y) arc=..)`
    是否正好指出现场真正过不去的窄处（这条日志就是"该挪东西 / 该换目标"的证据）。
  - 台架上远点**仍未真正到达**（停在离目标 0.1~1.3 m 处蠕行/停住）：走廊在
    odom ≈ `(4.6~4.8, 1.7~2.0)` 处的真实净空只有 ~0.25 m（在线层在那里有 2 格占据，
    来自 z≈0.3~2.7 m 的结构；PGM 先验图同一处是空的），比 required 0.260 m 窄 1~3 cm。
    这属于现场通行性判断（挪开物体或换目标），**没有**为此放松任何阈值。
  - `Downgrading to COLD_START` 在台架上仍有 ~43 次/60 s，且与 HOT_START 变体同量级，
    说明它由"车被刹停 / 被门控打断"触发，而不是本项改动带来；真正压低它的是让 `/opt_path`
    不再中断。若车上仍频繁出现，下一批嫌疑是窄道里的净空门控与在线层那两格，而不是该分支。
  - 未做：把 ROGMap 实时占据接入全局搜索（让全局折线主动避开在线层窄口并居中）；
    在线层高度窗口复核（`scan_z_max_abs` 2.75 vs 旧工程 0.75，本次在台架上试过 1.0 / 0.75
    两档，那两格占据依旧存在，故未改）。一次性脚本都留在 `.scratch/`（不进仓库）：
    `repro/run_variants.sh`（配置副本对照）、`repro/ab_f3.sh`（F3 对照）、
    `corridor_width_check.py`、`layer_probe.py`。

## 2026-09-17 — 目标时间戳归零：远端 Foxglove 点击的目标不再因时钟偏差被丢弃

- 需求与现场证据：用户用 Foxglove + SSH 远程调试，在地图上点击发目标点，3D 面板能看到点，
  但 `/opt_path_vis`、`/nav_executor/global_plan` 始终没有轨迹。nav_executor 节点日志显示每次点击都是

  ```text
  Ignoring goal: cannot transform from map to odom
    (Lookup would require extrapolation into the future.
     Requested time 1789645723.193000 but the latest data is at 1789645723.112084)
  ```

  逐次量「目标到达时刻（本机时钟）vs 目标自带 `header.stamp`」得到 +351 / +346 / +313 / +320 ms，
  即**笔记本时钟比机器人快约 330 ms**。Foxglove 用客户端时钟给发布的 ROS 消息打戳，目标一到达本机
  就已经「在未来」，tf2 拒绝向未来外推，于是目标在 `acceptGoal` 的第一次坐标变换就被丢弃。
  机器人自身时钟正常（`System clock synchronized: yes` / `NTP service: active`，且无 `/clock`）。
  先尝试同步笔记本时钟，未能收敛——`systemd-timesyncd` 对小于 5 s 的偏差是约 500 ppm 的缓 slew，
  350 ms 需要十几分钟才能掰回来。
- 改动（代码，`src/mas2027_nav_executor/src/path_planner/path_planner.cpp`）：`acceptGoal` 在两次
  `tf_buffer_->transform(...)` 之前把目标 `header.stamp` 归零
  （`rclcpp::Time(0, 0, node_->get_clock()->get_clock_type())`），tf2 把 0 视为「取最新可用 TF」，
  从此与两端时钟偏差无关；第二次（转到地形图坐标系）查询前再显式归零一次，避免依赖 tf2 对
  `transform()` 输出时间戳的实现细节。**只改时间戳，坐标系、可通行性、ROGMap 快照、
  里程计新鲜度、`dynamic_map_timeout_s` 等判定门槛一处都没动。**
  下游确认不受影响：`TaskManager::submitGoal` 用的是自己的 `rclcpp::Clock().now()`，
  且 `task_manager.cpp:76` 的 `pose.header.stamp` 检查针对的是机器人位姿而不是目标。
- 验证：
  1. 改前先做无风险对照实验：用 `ros2 topic pub --once` 发一个 `stamp=0`、位于地图外 `(1000, 1000)`
     的目标，日志由 `extrapolation into the future` 变为
     `Ignoring goal: occupied in terrain or dynamic map`，证明 TF 那一步已经能通过
     （该目标在地图外必被拒，全程不产生任何运动命令）。
  2. `colcon build --packages-select mas2027_nav_executor --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
     通过（22 s），新二进制时间戳 19:54:21。
  3. **未完成**：改后尚未在实机复测。重启 nav 栈需要伪终端，而本次会话 shell 无 tty，
     `emulate_tty=True` 的节点（`nav_executor` / `odom_localizer` / `ros2_comm`）报
     `OSError: out of pty devices`，因此重启交回用户在自有 SSH 会话执行；
     待其重启后需补一次「带 +330 ms 未来时间戳的目标能被接纳」的验证。

## 2026-09-17 — 新增 Foxglove 远程可视化调试手册（含 3.5.0 子协议改名坑）

- 需求与现场证据：用户需要"在另一台机器通过 SSH 远程用 Foxglove 可视化并调试"。
  现场为 Ubuntu 24.04 + ROS 2 Jazzy，机器人 `mas-intel-1`（wlan `192.168.77.79`），
  笔记本 `192.168.77.15`，已装 `ros-jazzy-foxglove-bridge 3.5.0` 与 Foxglove Studio 3.1.1。
  排障中定位到两个必踩的坑：① 原先用 `address:=127.0.0.1` 起 bridge，只监听回环，
  远端无从连起；② 该版本 bridge 已换用新版 Rust `foxglove-sdk`，WebSocket 握手
  **只接受子协议 `foxglove.sdk.v1`**，不再接受 `foxglove.websocket.v1`，旧客户端一律被 400 拒绝，
  服务端只留下 `Dropping client ...: handshake failed`，而报错文本
  `Missing expected sec-websocket-protocol header` 位于 `/opt/ros/jazzy/lib/libfoxglove.so`。
- 改动：新增文档 `src/docs/foxglove_remote_debug.md`，内容包括链路拓扑、机器人端/笔记本端
  指令、Foxglove 面板与话题清单、`topic_whitelist` 限流、8 类已知坑、排障速查表，
  以及远程发 `/goal_pose`、录包等调试动作。**未改动任何代码、launch 或配置**；
  bridge 仍用官方 `foxglove_bridge_launch.xml`，仅通过 `address` / `port` / `topic_whitelist`
  参数区分"SSH 隧道"与"直连"两种用法。
- 验证：在本机实测。用 Python 原始 socket 对比三种子协议声明：`foxglove.websocket.v1`
  → `400 Bad Request`，`foxglove.sdk.v1` → `101 Switching Protocols`，两者同时声明 → `101`
  且服务端选中 `foxglove.sdk.v1`；在 `libfoxglove.so` 中确认该错误字符串与 `foxglove.sdk.v1`
  相邻；在 `/opt/Foxglove/resources/app.asar` 中确认客户端
  `SUPPORTED_SUBPROTOCOLS=["foxglove.websocket.v1","foxglove.sdk.v1"]`，即 Studio 3.1.1 兼容。
  bridge 分别以 `0.0.0.0:8765` 与 `127.0.0.1:8765` 起过，`ss -tln` 确认监听，
  日志出现 50 余条 `Advertising new channel`；另确认 ufw 为 `ENABLED=no`、
  `sshd_config` 未限制 `AllowTcpForwarding`。
  **未验证**：笔记本侧 Foxglove 的实连（无法代替上位机操作）、隧道带宽下的点云帧率、
  网页版 `studio.foxglove.dev` 的实际握手（仅依据 mixed-content 规则推断必须走隧道）。

## 2026-09-17 — "MPC 输出让车慢且卡顿"排查：种子门撤掉净空硬否决 + MPC 打断时锚到实测车速

- 需求与现场证据：用户反馈"经过 MPC 输出的速度，会使得频繁地进入冷启动，行驶速度慢且卡顿；
  `/home/mas/mas_nav_2027` 里设 3 m/s 上限就跑得很快，应该不是上下位机传输的问题"。
  先量了三路速度（新脚本 `test/trace_speed.py`）：2026-09-17 14:02 那次实车，
  6.8 m 的一段用了 12.8 s（平均 0.53 m/s）、**全程无任何 Braking/停车前缀/急停**，
  而 `cmd` 1.14~1.73 m/s、`odom` 0.55~0.91 m/s，且 `odom` 与位姿位移一致（0.66 vs 0.62~0.75）
  ⇒ 那一段的"慢"不在规划也不在 MPC，底盘只跟到指令的一半。
  再建闭环台架 `test/bench_closed_loop.py`（真实 map_server + 真实 nav_executor +
  真实 lab3 地图 + 一阶底盘模型）复现"走走停停"：规划峰值 2.5~2.9 m/s、MPC 输出跟着到
  2.4~2.6 m/s，但 `MINCO path generation failed` 多的那些运行移动拍只有 11~19%、
  40 s 只走 4.4~4.6 m；失败少的运行移动拍 70~84%、走 15~20 m
  ⇒ **慢的机制是规划侧拒绝发布轨迹（`/opt_path` 一断就只能发 0），MPC 不是限速环节**。
  `Large velocity error → COLD_START` 是车被刹停后规划器的自愈动作，是症状不是原因。
- 改动 1（代码，`local_path_processor.cpp` / `.hpp`）：**种子门不再用净空硬否决**。
  交给 MINCO 的是折线种子、真正执行的是优化后的轨迹，MINCO 的位置罚项会把轨迹推离障碍，
  拿折线的净空去否决种子等于要求种子先满足轨迹的指标、把 MINCO 的避障能力作废；
  旧工程 `mas_nav_2027` 的 `isLineFree`（`minco_core/components/local_path_processor.cpp:9-31`）
  只查占据，而它的轨迹级门槛更严（`collision_dist 0.30`、无抖动容差）却跑得快。
  现在 `segmentClear(..., enforce_clearance)`：占据/地形/查询无效仍是硬否决，净空不足只记现场
  （新增 `SeedRejectInfo::clearance_only`）并继续采样；停车/脱困前缀传 `enforce_clearance=true`
  （前缀末端是"车要停在哪"，本身必须满足净空）；新增可查 INFO
  `Local seed is tight but not blocked (...)`,与 `Live obstacle blocks the local route` 区分；
  `note_reject` 允许硬否决覆盖先前的软记录。**三道轨迹级净空门一处都没放松。**
- 改动 2（代码，`mpc_solver.hpp` / `path_executor.cpp`）：**MPC 加速度锚点在被"轨迹过期 /
  参考失败 / 求解失败 / 任一指令门否决"打断时，锚到实测车速而不是清零**。此前清零 ⇒
  恢复后第一拍被限成 `a_max·dt = 0.2 m/s`，车还在 1.5 m/s 就硬砍到 0.2 ⇒ 底盘急刹再爬回来，
  门控 1~2 Hz 抖动时就是"反复启停 / 频繁冷启动"。速度上下限、加速度上下限、净空判据全不变，
  只是把约束从"相对上一条指令"改成"相对车实际速度"（后者才是真正的加速度）；缺里程计仍清零。
- 改动 3（工具与文档）：`test/bench_closed_loop.py`（闭环台架，含 stdout 抓取与三路速度曲线）、
  `test/trace_speed.py`（实车三路速度记录，docstring 写明"必须先 source 工作区，否则 plan 两列恒为 0"）、
  `docs/executor_velocity_triage_2026-09-17.md`（完整证据、机制、未定论项、上车验证步骤）。
- 验证：
  1. `colcon build`（Release）通过；`ctest` **9/9 通过**；
  2. `test_mpc_horizon` 新增锚点用例，实测打印
     `anchor semantics: reset cap 0.2 m/s, measured 1.5 m/s -> first step 1.7 m/s`；
  3. `test_local_path_processor` 按新语义改写并保留反例：0.50 m 走廊（中线净空 0.25 < 0.30）
     ⇒ 种子有效 + `clearance_only=true`；走廊中段整列封死且车离封口 0.10 m
     ⇒ 四层兜底全败、`dense_path` 清空、`verdict=GEOMETRY`；
  4. 标准冒烟：`smoke_goal.py` → `goal smoke passed: 40 trajectory poses, 48 global plan poses,
     marker width 0.150 m`；`smoke_goal_motion.py` → `cmd_vel: 240 msgs, moving 239,
     max speed 0.941 m/s` + `goal motion smoke passed`。
- 未验证 / 未做：**未上实车**。台架是**双峰**的（起点 odom(0,0) 净空只有 0.158 m，
  头 1 秒能不能从 near-field 放宽里挤出来基本掷硬币），rho 100/500 各跑两次结论互相矛盾
  （78%/19% 与 73%/11%），因此**没有**据此改 `penalty_weight_time`，也没动 `safe_dist`、
  ROGMap 高度窗口（`scan_z_max_abs` 2.75 vs 旧工程 0.75）等配置；这些都写进 triage 文档 §5，
  需按"单变量 + 至少 2~3 次运行"上车复测。另记一条待查项：台架 `/cmd_vel` 间隔 p90 61~80 ms、
  max ~100 ms（标称 50 ms），而 `nav_executor_node` 的四个可视化/地图订阅回调与 20 Hz 控制
  定时器同处默认回调组，可能是抖动来源，本次未改。

## 2026-09-17 — 统一四道净空门的有效阈值（审计 §1.1）：消掉"规划放行、执行刹车"的 2 cm 缝

- 需求与现场证据：用户给出新一轮实车日志并确认"明显一顿一顿，能听到反复启停"。
  该轮 **3/3 目标全部到达**、无急停、无 `MINCO trajectory not published`，但实测平均速度只有
  0.30~0.38 m/s（目标 2：7.37 m / 19.4 s；目标 3：2.18 m / 7.3 s），而规划速度是 1.3~2.5 m/s
  （45 次 `Large velocity error (1.0~1.6 m/s)` 即为该差值）。日志里
  `Braking: ... (reason=clearance value=0.263/0.273/0.274/0.279/0.279/0.279/0.280 threshold=0.280)`
  共 7 次，**全部落在 [0.26, 0.28] 这条缝里** —— 即全部是两套阈值造成的假否决。
- 机制：四道净空门里，发布前校验（`minco_planner.cpp:1931`）与 20 Hz 监视（`:1968`）用
  `requiredClearance(v) − kMonitorClearanceTolerance(0.02)`，而 **MPC 指令门**
  （`command_safety.cpp`）与**局部种子门/绕行搜索**（`local_path_processor.cpp`）直接用完整要求。
  于是规划器按 0.26 判"能过"并发布轨迹（或生成种子），执行器却按 0.28 否决指令、
  种子门又按 0.28 否掉种子 → 现场表现就是反复启停。指令门每次否决还会
  `solver_->resetLastControl()`（丢 MPC 加速度锚点），所以是"一脚踩死 + 重新起步"。
- 改动（**一处常量、一条判据**）：
  1. `common/environment/clearance_gate.hpp` 新增唯一定义处
     `kEsdfJitterTolerance = 0.02` 与 `effectiveClearanceThreshold(full)`，
     并在注释里写明四道门、0.05→0.02 的来历（0.05 曾把有效阈值压到 0.20 导致实车撞墙）；
  2. `minco_planner.cpp` 的 `kMonitorClearanceTolerance` 改为引用该常量的别名（行为不变）；
  3. `command_safety.cpp`：完整要求改用 `effectiveClearanceThreshold(rog_map_clearance)`，
     **近场半径仍取 `rog_map_clearance`（车体安全半径），不变**；
  4. `local_path_processor.cpp`：种子门 `segmentClear`、绕行搜索 `cellTraversable` 的完整要求
     改用新私有 helper `fullClearanceRequirement()`（= effectiveClearanceThreshold(collision_dist_)），
     近场半径与 `kNearFieldSlack` 语义不变（后者也改为引用同一常量）；
  5. `classifySeedReject` 的判读模型同步改为统一后的有效阈值（否则它会继续按旧的两套阈值
     建模，把"其实能过"的点报成 SEED_GATE_STRICTER）。
- 验证：
  1. `colcon build`（Release）通过；`ctest` **9/9 通过**；
  2. `test_rog_map_command_safety.cpp` 新增 3b 用例锁住这条缝：起点净空 0.45、1.0 m/s、
     净空随 +x 以 0.46/m 下降 ⇒ 前视 0.35 s 末端（弧长 0.35 m，已出近场半径 0.30）净空 ≈0.289，
     **统一前按 0.30 必拒、统一后按 0.28 必放行**；并保留"压到 0.28 以下仍必须拦下"的反例，
     避免把统一写成放水；
  3. `test_local_path_processor.cpp` 两处期望按新语义更新：种子门的 `required` 由 0.30 改为
     `effectiveClearanceThreshold(0.30) = 0.28`（并加断言防止某道门再自己写死一个数）；
     `classifySeedReject` 的"近场之外、clearance 0.295"一格由 `GEOMETRY` 改为
     `SEED_GATE_STRICTER`（统一后 0.295 能过 0.28，因此该否决只可能来自漂移——这一格
     从"环境过不去"变成了"判据不一致"的探针），并补一格 clearance 0.27 < 0.28 仍报 GEOMETRY；
  4. 标准冒烟 `smoke_goal.py` 通过：`goal smoke passed: 40 trajectory poses, 48 global plan poses,
     marker width 0.150 m`；实车 CSV `.scratch/rog_map_perf_summary.csv` 冒烟前后 md5 不变。
- 未验证 / 未做：**未上实车**。预期效果：那 7 次 `clearance` 否决全部消失、种子门不再因
  0.26~0.28 的窄处报 `LOCAL_SEED_INVALID`（这也是 0.28 m 蠕行前缀的来源之一）。
  仍需现场确认的是：统一后实际速度是否跟上来；若"一顿一顿"仍在，下一批嫌疑是
  MPC 跟踪权重（`mpc.weights.state/command`）与建图丢帧（`[ROG WARN] Unfinished frame` 仍约 1 s 一次）。
  **未改动任何净空数值本身**（`collision_dist`/`rog_map_clearance` 仍 0.28），
  只统一了四道门对它的取用方式。

## 2026-09-17 — 提速：`minco_optimizer.penalty_weight_time` 100 → 500（**只改一项配置**）

- 需求：用户反馈"现在基本好了，轨迹也是走的中间，但是我希望更快一点"。
- 先排除嫌疑（每条单变量各跑一次冒烟，量 `/nav_executor/global_path` 上 MINCO 轨迹的速度
  剖面 —— 该话题按 `t_step = 0.05 s` 均匀采样发布，相邻点间距 / 0.05 即规划速度）：
  | 变体 | vmax | vmean | 结论 |
  |---|---|---|---|
  | 基线（`max_velocity 3.0`） | 1.937 | 1.021 | — |
  | `max_velocity` 3.0 → 4.0 | 1.942 | 1.021 | **速度上限没被顶到**，不是瓶颈 |
  | `turn_angle_deadzone` 0.1→0.35、`decay_power` 1.3→0.8、`min_turn_vel` 0.8→1.5 | 1.943 | 1.021 | **转弯限速也没起作用** |
  另核对 `calCurvatureDecay`（`minco_utils.cpp:17-29`）：`ratio` 的分母是 `saturation`，
  `saturation=1.0` 时 `ratio` 被夹在 0.9，`exp(-1.3×0.9)=0.31` ⇒ 转角限速实际下限约
  **1.48 m/s**，配置里的 `min_turn_vel: 0.8` 永远取不到（该值形同虚设，但也不是瓶颈）。
- 真正的瓶颈是 MINCO 的"能量 ↔ 时间"权衡：`rho`（本项）相对 jerk 能量项太小，优化器倾向
  拉长分段时长省能量。实测同一目标点、同一张图：
  | `penalty_weight_time` | 轨迹时长 | vmax | vmean | v_p50 |
  |---|---|---|---|---|
  | **100（原值）** | 2.55 s | 1.937 | 1.021 | 1.084 |
  | 200 | 2.25 s | 2.186 | 1.156 | 1.225 |
  | **500（本次取值）** | 1.95 s | **2.506** | **1.336** | 1.367 |
  | 1000 | 1.95 s | 2.461 | 1.339 | 1.459（已饱和） |
  即 100 → 500 使峰值速度 **+29%**、平均 **+31%**，再往上无额外收益（已顶到加速度/速度限制），
  故取 500。
- 改动：仅 `mas2027_nav_executor/config/planner_params.yaml` 的
  `planner.minco_optimizer.penalty_weight_time` 由 100.0 改为 500.0（附 30 行说明：
  排除嫌疑的实测表、饱和点、代价、回退方式）。**未改任何源码、未改其它参数。**
- 验证：
  1. 最终配置复测：`MINCO-SPEED pts=40 vmax=2.506 vmean=1.336 v_p50=1.367 v_p90=2.454`，
     与变体测量一致；折线相对融合层净空不变（min 0.427 m，`frac_lt_0.30 = 0`）；
  2. 标准冒烟 `smoke_goal.py`（ROS_DOMAIN_ID=232）通过：
     `goal smoke passed: 40 trajectory poses, 48 global plan poses, marker width 0.150 m`
     —— 轨迹点数 52 → 40 正是时长 2.55 s → 1.95 s 的体现，其余断言（格点路径、帧、末点、
     marker 样式）全部照旧通过；
  3. 本轮所有对比都在 `.scratch/` 下的配置副本里做（`cfg_speed_base` / `cfg_sp_time200` /
     `cfg_sp_time500` / `cfg_sp_time1000` / `cfg_sp_vmax4` / `cfg_sp_turnsoft`），
     实车 CSV 未被触碰。
- 顺带修正一个**测量脚本自身的坑**（`.scratch/esdf_path_compare.py`，不进仓库）：
  原先等到 `/nav_executor/global_plan`（全局折线）就退出，而 MINCO 轨迹是随后才发布的，
  导致一次变体被误判为"完全无轨迹"。已改成必须同时等到 `/nav_executor/global_path`。
- 未验证 / 未做：**未上实车**。需要现场确认：(a) 实际车速是否跟着上来（RViz 的 MINCO Trajectory
  按速度染色，可直接看颜色）；(b) MPC 能否跟上更激进的计划（若跟不上/超调，下一批旋钮是
  `mpc.weights.state / command` 与 `mpc.constraints.*`，不要先动本项）；
  (c) 本项削弱了"靠降速换净空"的倾向，若觉得"快了但不踏实"，正确做法是把
  `replan_react_time` 从 0 调到 0.05~0.1（给 required 加 v·t 的制动预算），而不是回退本项。
  未动 `max_velocity` / MPC 速度上限（当前两处都是 3.0，本次轨迹峰值 2.5 仍未顶到）。

## 2026-09-17 — 让 MINCO 看到先验墙：`rog_map.projection.prior_map.enable` false → true（**只改一项配置**）

- 需求：用户要求"让 minco 看到先验墙"，并指出"rogmap 不是有先验模式吗，你去看 mas_nav_2027"。
- 上游核对（按用户指示）：
  - 原始 HW 工程 `navi_minco_bit/src/navigation/navi2_bringup/params/sentry1.yaml:389-398`
    **在运行时是开启的**：`projection.prior_map.{enable: true, yaml_path: <first_floor_prior.yaml>,
    pgm_path: "", frame_id: map}`；注释写明"启用后只增加二维硬障碍，不清除在线障碍"。
  - 本工程 `src/mas2027_perception/rog_map/src/rog_map/prior_map.cpp` 与上游
    `navi_minco_bit/src/perception/rog_map/src/rog_map/prior_map.cpp` **逐字节相同**
    （`diff` 无输出），加载器实现正确：`negate=0` 时 `occupancy=(255-gray)/255`、
    PGM 行序按 `image_row = height-1-my_from_bottom` 翻转、支持 origin/yaw。
  - 生效链路（逐行核对）：`enable=true` → `rebuildFusedProjection()` 把先验占据格并进
    `fused_projection_mask_`（只叠加障碍；`rog_map.cpp:832-844`）→
    `field_->update(..., fused_projection_mask_, ...)` 用它烘焙二维 ESDF（`rog_map.cpp:766-770`）
    → `refreshQuery()` 把快照与 field 交给查询接口（`rog_map.cpp:869`）→
    MINCO 位置罚项 / 发布前校验 / 20 Hz 监视 / MPC 指令门（都走 `dynamicQuery()`）全部可见。
    **合并发生在 ESDF 之前**，因此是"优化器直接看得见"，而非地形门那种事后否决。
  - 失效保护：拿不到 map→odom 时 `getPriorMapTransform()` 返回 false，
    `rebuildFusedProjection()` 收到 nullptr 直接 return，退化为只用在线感知
    （`prior_map.cpp:298-301`），不会用错位姿贴图。
- **推翻 `planner_params.yaml` 里 2026-09-15 的"先验图未对齐"结论**（本次改动的关键依据）：
  当时的证据是"在 (3.2,-0.03) 附近 layer_value_static 有墙而 layer_value_dynamic 是空的"，
  但两层语义本就不同 —— static 是 PGM 先验，dynamic 只含**实时观测过**的格子，未观测处为空
  是正常的，它证明不了错位。本次直接查先验点云（新增一次性脚本
  `/home/mas/mas_nav_2027_native/.scratch/pcd_wall_check.py`，解 `binary_compressed`/LZF 后统计）：
  - `(3.20,-0.03) ±0.15 m` → **43 点**，z∈[-0.03, 3.13]，z 中位 1.89 m，z>0.05 m 的 41 点；
  - 对照 `(5.97,2.11) ±0.15 m` → 21 点，z 中位 2.14 m（审计 §1.3 已核实的真墙）。
  即 (3.2,-0.03) **本来就是真墙**。所以当次融合后净空降到 0.272~0.285 m 是**真实净空**，
  4 次 COLLISION + 10 次急停是**正确拒绝**（该路径距真墙 0.27 m < collision_dist 0.28，
  更远低于车体外接圆半径 0.306 m）。真正该修的是"路径贴墙"，不是把墙藏起来。
- 改动：仅 `mas2027_nav_executor/config/planner_params.yaml` 的
  `planner.rog_map.projection.prior_map.enable` 由 false 改为 true；`yaml_path` 仍指向
  `src/mas2027_nav_bringup/map/lab3.yaml`（`pgm_path: ""` 走 YAML 的 image 字段，
  `frame_id: map`）。**未改任何源码、未改其它参数**；同时重写了该段注释（推翻旧结论的依据、
  生效链路、失效保护、两个副作用、回退方式）。
- 验证：
  1. 冒烟 `smoke_goal.py`（ROS_DOMAIN_ID=232）在开启后仍通过：
     `goal smoke passed: 52 trajectory poses, 48 global plan poses, marker width 0.150 m`；
  2. 用探针订阅 `/rog_map/layer_value_static`（QoS 必须匹配发布端的
     `QoS(1).best_effort().keep_last(1).volatile`，否则收不到）实测**开关前后**：
     | 配置 | `/rog_map/layer_value_static` | 融合层 `/rog_map/layer_value` | 折线相对融合层净空 |
     |---|---|---|---|
     | `enable: false` | 占据格 **0** | 占据格 **0** | 无意义（距离场全为哨兵值 1.58e6）|
     | `enable: true` | 占据格 **11835** | 占据格 **11835** | min **0.427 m** / mean 0.684 m |
     11835 与离线按 `lab3.pgm` 数出的 11932 吻合；且折线净空与它相对**地形图**（同一张 PGM）
     的净空逐位一致（min 0.427 / mean 0.684）⇒ 两张图口径统一，MINCO 不再"看不见墙"。
  3. 开关前后全局折线本身不变（48 点 / 2.749 m）——符合预期：全局搜索用地形图，不走 ROGMap。
  4. 新增的一次性脚本均放在 `/home/mas/mas_nav_2027_native/.scratch/`（**不进仓库**）：
     `pcd_wall_check.py`、`rog_static_probe.py`，以及扩展了融合层统计的
     `esdf_path_compare.py`；实车 CSV `.scratch/rog_map_perf_summary.csv` 未被本轮触碰
     （对比运行写到 `.scratch/esdf_cmp/<配置名>/`）。
- 未验证 / 未做：**未上实车**。需要现场确认的三件事：(a) RViz `/rog_map/layer_value_static`
  与实测墙体是否吻合；(b) 是否出现"点目标没反应"——先验占据格会让机器人自身所在格被判不自由，
  `acceptGoal` 会拒绝（`Ignoring goal: ROGMap is not clear at the robot pose`）；
  (c) 窄处是否更频繁触发 `LOCAL_SEED_*`/`COLLISION`。回退方式：本项改回 `false` 并重启
  （configure-time，运行时改不生效）。

## 2026-09-17 — 修"全局折线太贴墙"：`smac_2d.esdf_max_cost` 0.5 → 5.0（**只改一项配置**）

- 需求：用户反馈"全局路径规划太贴墙了，明明可以有更好的路径"，并问"esdf 是有动态物体吗、
  先验图不进入 esdf 吗"。本轮先只做用户选定的**单变量**改动，其余候选（统一四道净空门、
  把 ROGMap 实时占据接入全局搜索、先验图进 MINCO 的 ESDF）留待后续。
- 机制（代码依据）：`smac_planner_2d_simple.cpp:181-186` 的
  `potential = min(weight·exp(-d/decay), max_cost)`，而 `max_cost(0.5)` 只有 `weight(1.0)` 的一半
  ⇒ **d ≤ decay·ln2 = 0.5545 m 以内 potential 恒为 0.5、梯度为 0**；走廊净宽 ≤ ~1.1 m 时
  整条走廊都落在饱和区，全局搜索只剩"最短路径"一个目标 → 贴内侧墙、切内角。
  代价量纲是"每米路径的倍率"（1 + potential）：贴墙 1.5×/m，2 m 宽走廊中心仅 1.29×/m。
  另外 `TerrainMapQuery::refresh()` 把所有非占据格统一压成 `kFreeCost=0`，SMAC 的
  `evaluateInflationCost(0)=1.0`，因此地形图里 50/66 那类中间代价在进搜索前就丢了 ——
  **ESDF 是全局搜索里唯一的软引导**，它饱和等于没有引导。
- 溯源（避免误判为笔误）：0.5 与上游 HW 原始工程
  `navi_minco_bit/src/navigation/navi2_bringup/params/sentry1.yaml:508-512` 完全一致；
  而 `smac_planner_2d_simple.hpp:161-164` 的代码默认是 `use_esdf_cost=false / decay=0.5 /
  max_cost=5.0`，即上游本意是"势场不封顶"。取 5.0 是因为 `max_cost ≥ weight` 时封顶不生效，
  与 1.0 行为等价，写 5.0 是为了显式表达"不封顶"并与代码默认一致。
- 改动：仅 `mas2027_nav_executor/config/planner_params.yaml` 的 `smac_2d.esdf_max_cost`
  由 0.5 改为 5.0（连同 40 行说明：机制、量纲、溯源、实测数据、回退方式）。
  **未改任何源码、未改其它参数**；`esdf_weight` 仍 1.0、`esdf_decay` 仍 0.8。
- 验证（新增一次性对比脚本 `/home/mas/mas_nav_2027_native/.scratch/esdf_path_compare.py`，
  **不属于仓库**）：跑真实 `map_server` + `mas2027_nav_executor_node`，取
  `/nav_executor/global_plan` 与 `/cost_map`，用与 `TerrainMapQuery::refresh()` 同一判据
  （`cost < 95` 且 `>= 0` 为自由）以 `cv2.distanceTransform` 烘焙距离场，量折线沿线净空：
  | 配置 | 点数 | 折线长 | 最小净空 | 平均 | 中位 | <0.30 m 占比 | <0.50 m 占比 |
  |---|---|---|---|---|---|---|---|
  | `max_cost 0.5`（改前） | 48 | 2.755 m | 0.391 m | 0.678 m | 0.702 m | 0.0% | **10.4%** |
  | `max_cost 5.0`（改后） | 48 | 2.749 m | **0.427 m** | 0.684 m | 0.702 m | 0.0% | **6.2%** |
  | 另测 `weight 2.0`（max 5.0） | 48 | 2.749 m | 0.427 m | 0.684 m | 0.702 m | 0.0% | 6.2% |
  | 另测 `decay 1.2`（max 5.0） | 48 | 2.749 m | 0.427 m | 0.684 m | 0.702 m | 0.0% | 6.2% |
  结论：**起决定作用的是"解除封顶"**；在本用例上再加权/放宽衰减与单改 5.0 结果完全相同，
  故本轮只改这一项。每次运行的实际取值已从节点启动日志核对
  （`SMAC 2D global search enabled: use_esdf_cost=true weight=… decay=… max_cost=…`）。
  标准冒烟 `smoke_goal.py`（ROS_DOMAIN_ID=232）通过：
  `goal smoke passed: 52 trajectory poses, 48 global plan poses, marker width 0.150 m`。
  另核对 CSV 隔离有效：本次对比运行的产物落在 `.scratch/esdf_cmp/<配置名>/rog_map_*.csv`，
  实车 `.scratch/rog_map_perf_summary.csv` 的最后一行是**用户自己那次实车运行**
  （时间戳 1789620143~1789620314）写入的，未被本轮触碰。
- 未验证 / 未做：**未上实车**，贴墙改善幅度（本用例最小净空 +3.6 cm、近墙采样点比例 10.4%→6.2%）
  在真实窄走廊里可能更明显也可能不足，需现场用同一目标点前后对比折线形状；
  未做统一四道净空门（审计 §1.1，本轮日志里仍出现
  `Braking: … reason=clearance value=0.278 threshold=0.280`，差 2 mm）、
  未把 ROGMap 实时占据接入全局搜索、未把先验图并入 MINCO 的 ESDF。

## 2026-09-17 — 修"一卡一卡"：动态层新鲜度阈值参数化（F1）+ 热启动日志/行为矛盾（F3）

- 需求与现场证据：用户报告"车移动的很不丝滑，一卡一卡"，并给出一份 25 s 的实车日志。
  日志里的两条规律性重复定位到本轮的改动：
  1. `Braking: ... (reason=dynamic_stale value=0.509~0.521 threshold=0.500)` **12 次/24.5 s**，
     且数值几乎恒定 —— 说明不是偶发抖动，而是被卡在阈值边缘按发布周期反复触发；
  2. `Large velocity error (1.0~1.7 m/s). Downgrading to COLD_START.` 与
     `Hot Start Rejected: Direction mismatch` 各几十次 —— 热启动几乎从不生效。
  用户另确认：**当时车头前方空间是空的**（见下"未解决"）。
- **F1（动态层新鲜度阈值参数化 + 给足余量）**：`map_server` 在
  `bypass_dynamic_obstacle:=True`（本仓库 `nav_executor_launch.py:119` 的默认值）下不订阅点云，
  只由 500 ms 定时器发一帧**全 0 空图**（`map_server_node.cpp:94/101/109`），而判据硬编码 0.5 s
  （`command_safety.cpp:40`），两者零余量 ⇒ 20 Hz 控制下每个周期末尾必有一拍越界，
  整条速度指令被清零且 `resetLastControl()` 丢掉 MPC 热启动 ⇒ 每秒约 2 次顿挫。
  改动：
  - 新增参数 `node.dynamic_map_timeout_s`，默认 **1.5 s**（= 3× 发布周期），
    在 `nav_executor_node.cpp` 声明并校验（必须为正）；
  - `command_safety.{hpp,cpp}` 增加形参 `dynamic_map_timeout_s`，替换硬编码 0.5；
  - `PathExecutorParams` 增加同名字段，`path_executor.cpp` 透传；
  - `PathPlanner` 构造函数增加 `dynamic_map_timeout_s` 形参，目标接纳门
    （原 `path_planner.cpp:72` 的 `> 0.5`）改用该值，并把 age 上限打进 WARN；
  - `config/node_params.yaml` 增加该项（含现场依据与"只放宽新鲜度、不触碰净空阈值"的说明）。
  ⚠️ 本项只放宽"地图新鲜度"，实时避障仍由 ROGMap（20 Hz）负责，未改动任何净空阈值。
- **F3（热启动两条分支的日志与行为矛盾）**：`minco_planner.cpp` 的 `determinePlanningState()`
  在 tracking_error / vel_error 超限时打印 "Downgrading to COLD_START" 却 `return HOT_START`
  （COLD_START 那行被注释掉）。已核对上游 `/home/mas/mas_nav_2027` 的同名文件是**同样的写法**，
  属移植遗留而非本地实验，且本地无任何验证记录。改为真正 `return COLD_START`：
  起点取实测速度、加速度置零，时间/路点种子由 PTAllocation 从新路径重新分配。
  影响面：执行器正在刹车时，优化器不再按"上一条轨迹的预测速度"（实测差 1.0~1.7 m/s）生成轨迹。
- 验证：
  1. `colcon build --packages-select mas2027_nav_executor`（Release）通过；
  2. `ctest` **9/9 通过**；并给 `test/test_rog_map_command_safety.cpp` 补了 4 条新断言锁住本行为
     （age=0.51 s 必须放行、age=2.0 s 必须被 `dynamic_stale` 拒绝、`detail.threshold` 必须等于
     传入参数），旧两处调用同步了新签名；
  3. `smoke_goal.py`（隔离域 ROS_DOMAIN_ID=232）通过：`goal smoke passed: 52 trajectory poses,
     48 global plan poses, marker width 0.150 m`；冒烟前后实车 CSV
     `.scratch/rog_map_perf_summary.csv` 行数（79）与 md5 不变，确认隔离仍有效；
  4. 已确认 `nav_executor_launch.py:96` 用 `glob` 加载执行器包内全部 `config/*.yaml`，
     因此新参数无需改 launch 即可生效。
- 未验证 / 未做：**未上实车**，1.5 s 的实际手感与是否仍偶发 `dynamic_stale` 待现场确认；
  未做 F2（旁路空图不应有否决权）、F4（否决时按加速度限幅减速而非一步到零）、F6（ROGMap 丢帧）。
- **未解决（本轮改完后仍需跟进）**：用户确认"空间是空的"，但日志里
  `No local route around the live obstacle; planning a safe stopping prefix.` 11 次、
  `escaping with a 0.28 m creep prefix.` 4 次 —— 即 ROGMap 在空场地里把车头前方约 0.35 m 处
  判成了障碍，属**误停**。候选成因（本次未取证）：车体自身点云进入 `raycasting.ray_range`
  下限 0.3 m 之外、ROGMap 丢帧（`dropped 1` 约每 0.5 s 一次）造成的幽灵占据、
  或 `pathClear` 首点即车体所在格。下一步判据：卡住时看 RViz `/rog_map/occupied` 与
  `/rog_map/field` 在车头 0.5 m 内是否有格子；必要时给"修复成功"的分支也加上
  `dense_reject=` 插桩（当前该现场只在四层全败时才打印）。

## 2026-09-17 — 新增「全局折线穿过新障碍物」判别流程文档（**只加文档，未改任何代码/配置**）

- 需求：用户反馈"运行中突然遇到障碍物，全局路径不会改变，会穿过 occupied 过去"。经逐行核对，
  用户确认现场是"RViz 里粗青色的**全局路径**穿过障碍块，车实际绕开了/停下了"，
  并要求**只诊断、暂不改代码**。
- 结论：该现象属**预期行为**。全局搜索只在 `!hasGlobalPath()` 时执行
  （`task_manager.cpp:153`），`invalidateGlobalPath()` 仅 5 处调用（同文件 68/91/108/124/145），
  因此 FOLLOWING 期间折线不随新障碍更新；实际绕障由 `local_path_processor.cpp:171-229`
  的四层兜底在 ROGMap 滑窗内完成，与折线显示无关。
- 新增文件：`src/docs/global_plan_stale_triage_2026-09-17.md`。内容为四步判别流程
  （① 局部修复日志 ② MINCO 失败原因 ③ `rog_map_perf_summary.csv` 的输入/更新频率
  ④ RViz 粗青线 vs 细彩线该看哪条）、"什么情况下才不再是预期行为"的 7 条成因
  （建图丢帧、0.3 m 盲区、投影高度带、1.2 s 监视视界、失败后仍执行旧轨迹、
  全局搜索与净空用两张不同的图、`robot_state_` 竞态），以及代码位置索引与 4 个未实施的修复选项。
- 验证：本轮全部为只读核对（`task_manager.cpp`、`local_path_processor.cpp`、`minco_planner.cpp`、
  `planner_mode_context.cpp`、`planner_params.yaml`、审计报告 §1.3/§1.4）；
  文档中引用的行号均按当前 HEAD 逐条重读确认。**未跑实车、未运行任何会写盘的脚本、未改动源码或配置**，
  因此判别流程本身的现场有效性仍待用户跑一次带日志的实车验证。

## 2026-09-17 — 审查报告补 §6「复审：现在确实存在的 vs 之前已改掉的」（**只改文档**）

- 需求：用户要求只保留"现在确实有的问题，而不是之前改过的"。因此对 `docs/project_audit_2026-09-17.md`
  的 §1~§3 做了**逐条回到当前 HEAD 代码复核**，并新增 §6 记录复核结果；**未修改任何源码或配置**。
- 改动内容（`src/docs/project_audit_2026-09-17.md`，新增 §6 约 62 行）：
  1. §6.1「现存」：28 条，每条给出**本次在 HEAD 上重新读到的行号**作为证据，并标注现场状态
     （"每次运行"/"跨多次运行"/"特定条件触发"）。其中带本次实车计数的有：四道门两套阈值（9/13 次刹车）、
     verdict 误判（11/17）、静态墙（28+10 次）、建图丢帧 30%、动态图层陈旧门（14 次目标被丢，12 次在运行中途）。
  2. §6.2「已剔除」：9 条历史上已修好的问题（发布门↔监视门阈值一致、近场查询失败误杀、
     `fill_occ_min=9` 崩溃、投影全量重算、可视化快照未限频、`smoke_goal.py` 覆盖 CSV、
     双雷达掉线整段静默、地形门无否决点日志、失败原因被时间节流），
     逐条给出**当前代码**里的修复位置，证明"以前修的现在确实还是好的"，不再作为待办列出。
  3. 明确区分：§1.1 的四道门阈值不一致**不是**历史那条"发布门 vs 监视门"问题的重复
     （那条已在 `minco_planner.cpp:1925,1962` 修好），而是同一次修复没有覆盖到
     `command_safety.cpp:63-64`（执行门）与 `local_path_processor.cpp:331-339`（种子门）。
- 本轮为复核所做的验证（只读）：`grep`/`sed` 重读 `command_safety.cpp`、`mpc_solver.cpp`、
  `mpc_types.hpp`、`recovery_behaivor.cpp`、`terrain_map_query.cpp`、`smac_planner_2d_simple.cpp`、
  `terrain_grid.cpp`、`prob_map.cpp`、`mid360_driver.cpp`、`pgm_to_terrain_msgpack.py`、
  `map_server_node.cpp`、`odom_localizer/config/params.yaml`、`install/interfaces/.../CostMaps.*`，
  并对两份 mid360 配置做了 `diff`。
- 未验证 / 未做：未跑实车、未运行任何会写盘的脚本（含 §1.6 点名的 `smoke_goal_motion.py`，故意不跑）；
  §6.1 中非"每次运行"的条目仍是**代码路径确认**，触发频率未实测。

## 2026-09-17 — 全项目只读审查：新增 `docs/project_audit_2026-09-17.md`（**只加文档，未改任何代码/配置**）

- 需求：用户要求"分析本项目找出可能问题"。本次为**只读审查 + 实测复核**，
  **没有修改任何源码、配置或脚本**；唯一新增文件是本报告本身（外加本条历史记录）。
- 新增文件：`src/docs/project_audit_2026-09-17.md`（569 行）。内容为按严重度分级的问题清单
  （7 条高、25 条中、20 条低），每条给出位置（`文件:行号`）、证据、影响、最小修复与置信度，
  并附"本次核实为正常/已改善"的一节，避免重复劳动。
- 本次为取证所做的**只读/可写产物之外的实测**（均未改动仓库内容）：
  1. `build/mas2027_nav_executor` 下 `ctest` → **9/9 通过**（0.18 s，`ROS_LOG_DIR` 指向 `.scratch/ctest_log`）；
  2. `test/smoke_goal.py` + 原始 `lab3_terrain.msgpack`（隔离域 `ROS_DOMAIN_ID=232`）→
     `goal smoke passed: 52 trajectory poses, 48 global plan poses, marker width 0.150 m`（退出码 0）；
     并**实测确认冒烟脚本的 CSV 隔离有效**：`.scratch/rog_map_perf_summary.csv`（301 行、实车数据）
     在冒烟前后行数与 mtime 不变，冒烟产物落在 `.scratch/smoke_run/`。
  3. **解析了此前未被分析的最新一次实车数据**：`~/.ros/log/mas2027_nav_executor_node_80395_*.log`
     （21:59–22:04，21 目标/18 到达）+ `.scratch/rog_map_perf_summary.csv`（300 个 1 s 窗口）。
     由此得到两条**新结论**：① 脏列增量在实车确实生效（`full_reason_dirty_over_ratio` 300/300 为 0，
     projection 中位 12.8 ms），顶条记录里"未在实车确认"的悬念可结案；② 点云回调实测 **28.78 Hz**
     而非文档假定的 20 Hz，按 29 Hz 换算预算后 **60% 的窗口超预算**（不是"贴着 50 ms 跑"）。
  4. 逐条复算了该次运行 17 条 `Live obstacle blocks` 的 `verdict=`：**11/17 被误报为 GEOMETRY**
     （分类器 `local_path_processor.cpp:43-64` 的近场规则与种子门 `:331` 不同源，也不含轨迹门的
     0.02 容差），5/17 的实际差距 ≤5 mm；并复算 13 条 `Braking: reason=clearance`，其中 9 条
     threshold=0.280 而 value 0.262~0.279（**四道净空门存在 0.26/0.28 两套有效阈值**）。
  5. 独立解包 `pcd/lab3.pcd`，确认被地形门否决的 map (5.9~6.1, 2.0~2.2) 处**确有真实墙体**
     （19 点、z 到 3.89 m），即地形门拒绝正确、问题在 ROGMap 侧未知格被当自由。
- 验证：以上 5 项均可复现；报告中的行号均以本次读到的文件内容为准，
  凡只有代码依据而无实车/实测支撑的推断，条目内已标注「较可能 / 待验证」。
- 未验证 / 未做：**未修改任何代码或配置**（因此没有"修复后回归"可谈）；
  未运行 `smoke_goal_motion.py`（报告 §1.6 指出它会 trunc 掉实车 CSV，故意不跑）；
  未做 clean build（§2.24 的 install 残留建议由有写权限的一侧执行）。
- 说明：报告 §1.6/§1.7 指出**现有两个脚本仍带着已知风险**（`smoke_goal_motion.py` 会清空实车 CSV；
  `save_pcd_and_make_map.sh` 引用已删除的 launch 且兜底路径少一层 `src/`），本次**只记录未修**，
  留待后续按报告的 §5 顺序处理。

## 2026-09-16 — 脏列增量此前**从未生效**（实测 49% > 0.30 阈值）：阈值放到 0.95 + 周期性全量兜底

- 起因：用户问"为什么突然又卡起来了"。回查 21:30 那次的 summary 分阶段数据，发现**投影耗时仍与"每帧全量"量级一致**（13.98 ms 中位、最大 64.98 ms），而同一时段基准里全量投影是 42.9 ms、增量是 0 ms ⇒ 怀疑增量分支根本没走到。
- **硬证据（新增基准，与实车同一份 `planner.rog_map` 参数，96k 点/帧，77 次更新）**：

  | 指标 | 实测 |
  | --- | --- |
  | 走脏列增量分支的行数 | **0 / 77** |
  | 走全量分支的行数 | 77 / 77 |
  | `full_reason_dirty_over_ratio=1` | **76 / 77（99%）** |
  | `dirty_column_count` 中位 | **19740 / 共 40401 列 = 49%**（阈值 `0.30×40401 = 12120`） |
  | `projection_update_full_time_ms` / `_dirty_` | 42.87 / **0.00** |

  ⇒ 2026-09-16 那条"开脏列增量把投影从 43.5 压到 15.03 ms"的**归因是错的**：增量路径一次都没执行过。根因是 `raycasting.ray_range=[0.3,10.0]` 与地图窗口（10×10 m）等大 —— 每帧射线几乎扫过窗口里所有柱子，脏列自然逼近全图，"脏列超比例就回退全量"这个保护于是退化成"永远走全量"。
- 修改：
  1. `config.hpp`：新增 `performance.dirty_full_period_s`（默认 0.0 = 关闭，保持上游行为）。
  2. `rog_map.cpp::refreshLayers()`：新增**周期性全量兜底** `periodic_full_due` —— 与脏列多少无关，每 `dirty_full_period_s` 秒无条件走一次全量；新增 `last_full_refresh_time_`（`rog_map.h`）与归因标记 `full_reason_periodic`（summary + detailed CSV 都有列）。
  3. `planner_params.yaml`：`dirty_full_ratio` **0.30 → 0.95**、新增 `dirty_full_period_s: 0.5`。取舍：阈值放到"只在脏列几乎铺满时才回退"，同时用固定周期的全量做安全网 —— 万一某列因未预期原因长期没被标脏，二维图最多旧 0.5 s。回退方式：`dirty_column_enable: false`（一行，无需重编）。
- 验证：
  - Release 编译通过；`colcon test` **10/10 通过**（注：本机 `~/.ros/log` 只读，跑测试必须 `export ROS_LOG_DIR=<可写目录>`，否则 spdlog 直接 abort，看起来像 4 个测试失败）。
  - **回归**：`test/smoke_goal.py` + 原始 `lab3_terrain.msgpack` → `goal smoke passed: 52 trajectory poses, 48 global plan poses, marker width 0.150 m`（退出码 0）。
  - **改动前后基准 A/B（同一 96k 点场景）**：改前 `dirty_column_count` 19740、全量 77/77；改后实车（下方 21:44 运行）`last_dirty_column_count` 中位 **9343**、`dirty_over_ratio=1` 的窗口占比 **9.4%**（此前 99%）⇒ 增量路径真正开始生效。
- **用户随后那次运行（21:44:06–21:50:50，392 个 1 s 窗口，405 s，10 目标 / 9 到达）**：

  | 指标（窗口中位） | 21:30（改前） | **21:44（改后）** |
  | --- | --- | --- |
  | `cloud_callback_hz` | 19.98 | 19.96 |
  | `map_update_hz` | 19.25 | **16.27** |
  | 丢帧比 | 4.8% | **19%** |
  | `last_total_update_time_ms` 中位 / 最大 | 43.26 / 122.6 | **56.17 / 166.6** |
  | `last_projection_time_ms` 中位 | 13.98 | **18.10** |
  | `last_raycast_time_ms` 中位 | 9.30 | **11.53** |
  | `last_prob_update_time_ms` 中位 | （当时未记录） | **15.74** |
  | `last_update_period_ms` / `last_update_unaccounted_ms` | — | **61.12 / 0.44** |
  | `last_viz_time_ms` | — | **4.78** |

  本次运行更快到达（10 目标 9 到达，此前 2 目标/129 s），但 `map_update_hz` 与总耗时都比 21:30 更差。**归因需谨慎**：本次 `raycast` 与 `prob_update` 同时变重（11.53 / 15.74 ms，而 21:30 只有 raycast 可读且为 9.30 ms），且脏列增量在这种"脏列约 9343 列（23%）"的工况下**本该更快**（投影 18.10 ms vs 全量 42.9 ms，约 2.4×），所以本轮变慢更可能来自点云/场景本身变重（探索量更大），而不是这次改动；但**没有做同场景 A/B，不能定论**。
- 诊断侧顺带确认的两条（都进了 CSV）：`update_period_ms` 中位 **61.12 ms**（> 50 ms 预算 ⇒ 19% 丢帧与它自洽），而 `update_unaccounted_ms` 中位仅 **0.44 ms** ⇒ 上一轮怀疑的"可视化快照是隐形大头"**被否定**（`last_viz_time_ms` 中位 4.78 ms，且它已含在周期内）。**当前瓶颈明确是 `raycast 11.5 + prob_update 15.7 + projection 18.1 ≈ 45 ms` 这三段**。
- 未验证 / 风险：
  1. `dirty_full_ratio 0.95` 属**行为改动**，尚未在实车 RViz 上确认"无残留幽灵障碍、移动障碍能消失"。周期性兜底（0.5 s）只保证"任何列最多旧 0.5 s"，不保证与全量逐格等价。
  2. 本次改动**没有做同场景 A/B**（无法用同一段实车数据跑两遍），"增量比全量快 2.4×"来自基准与跨运行对比，不是同场对照。
  3. 那 4.8%→19% 的丢帧回升尚未定性；下一步要么做同场景 A/B，要么直接压 `prob_update`（第二大项，且从未被优化过）。

## 2026-09-16 — 第三次实车（21:30）：积压降到丢帧 4.8%，但"卡"仍在 —— 刹车原因细分 + 可视化快照限频

- 起因：用户报"又跑了一遍，比之前行进卡顿"。本次运行 `mas2027_nav_executor_node_41704_1789565418006.log`（21:30:19–21:32:25，129 s，116 行，**2 目标 / 2 到达**）+ 新的 124 个 1 s 窗口（`.scratch/rog_map_perf_summary.csv`）。
- **积压本身确实继续在好转，且是三轮里最好的一次**：

  | 指标（1 s 窗口中位） | 20:53（未开脏列） | 21:04 | **21:30（本次）** |
  | --- | --- | --- | --- |
  | `cloud_callback_hz` | 19.96 | 20.00 | **19.98** |
  | `map_update_hz` | 12.83 | 18.29 | **19.25** |
  | 丢帧比 | 35% | 9.5% | **4.8%** |
  | `last_total_update_time_ms` 中位 / 最大 | 73.4 / 128.8 | 45.92 / 71.73 | **43.26 / 122.57** |
  | `last_projection_time_ms` 中位 / 最大 | 43.5 / — | 15.03 / 33.41 | **13.98 / 64.98** |
  | `last_raycast_time_ms` 中位 | 8.7 | 8.76 | **9.30** |

- **关键量化：更新周期仍然超预算，但只超一点。** 用 CSV 的窗口时长与 `map_update_hz` 反推：单轮真实周期 = **51.95 ms**（`window_duration_sec` 中位 1.0264 s 也印证窗口被拉长），而自报 `total_update_time` 中位 **43.26 ms** ⇒ **中位就已经超过 20 Hz 的 50 ms 预算**，另外约 **8.7 ms** 花在 `total_update_time` 计不到的地方（可视化快照、取帧空档、队列等待）。分阶段：raycast 9.3 + 投影 14.0 + field 1.4 = 24.7 ms，**剩下约 18.5 ms 在 `prob_update` / `decay` / `query_refresh` 这三段**——而 summary 此前根本不报这三段。
- **"卡"与积压要分开看**：本次 43 条 `Braking` / 129 s = **每 3.0 s 一次**（20:53 那次是每 5.5 s 一次，21:04 是每 3.8 s 一次），`Repaired a ROGMap-blocked global segment` 25 次（几乎恒定每 2.05 s = 全局重规划节拍命中同一处受阻段）。也就是说**积压小了对"卡"的改善有限**：真正把车速打断的是命令门（MPC 输出前的安全门）反复否决，而它否决的原因此前是**四种完全不同的成因共用同一句日志**。
- 修改 1（**只加诊断，不改判据**）：`command_safety.{hpp,cpp}` 新增 `CommandSafetyDetail{reason, value, threshold}`，把命令门 8 条出口（`grid_or_frame_missing` / `dynamic_stale` / `tf_unavailable` / `terrain_transition` / `dynamic_horizon` / `clearance` / `clearance_out_of_map` / `dynamic_reference`）分别标出并带回实测值；`PathExecutor::lastSafetyDetail()` 暴露最近一次结果，`nav_executor_node.cpp` 的刹车日志改为
  `Braking: … (reason=terrain_transition value=0.000 threshold=0.000)`。
  依据：本次 43 条里 **28 条**是"terrain layer or map transform unavailable, or next command violates terrain"，**15 条**是动态走廊；前者到底是"地形层 transition 拒绝"还是"TF 一时查不到"分不出来，只能猜。注意判据顺序与语义完全未动（早退顺序、阈值、坐标系换算逐字保持）。
- 修改 2（**行为改动，只省无用的重复计算**）：`rog_map_ros2.hpp` 的 `updateWorkerLoop` 把 `captureVizFrame()` **按可视化发布频率限频**。`vizCallback` 是 `visualization.rate`（现场 5 Hz）的定时器，而更新跑在点云频率（约 20 Hz）上——此前每轮都重建整套快照，约 3/4 的结果在下次发布前就被覆盖（RViz 订阅了 `/rog_map/layer_*` 时每轮要遍历整张 200×200 投影层）。现在只在"距上次构建已达一个发布周期"时才建，RViz 看到的刷新率不变（仍是 5 Hz 定时器在发）。
  - 实测该改动**省下的量级有限但不为零**：新增基准里让订阅端按发布端同样的 `best_effort` QoS 订阅 3 个 `layer_*` 话题后，一次快照构建中位 **0.66~1.29 ms**（3 个话题、200×200 层），按 5 Hz 发、20 Hz 更新计算，直接省掉约 3/4 ×（每次构建成本 × 20 Hz）≈ **2~3 ms/轮**。**所以它不是那 8.7 ms 的主因**——本轮原假设（"可视化快照是主要隐形开销"）被自己的测量削弱，如实记录。
- 修改 3（诊断补齐）：
  1. summary CSV 补上 `last_prob_update_time_ms` / `last_decay_time_ms` / `last_query_refresh_time_ms` / `last_update_period_ms` / `last_update_unaccounted_ms` / `last_viz_time_ms` / `last_viz_built` / `last_viz_skipped` / `last_full_reason_*` / `last_dirty_column_count` / `last_projection_z_layers`。**取舍说明**：以前这些只在 detailed CSV 里，而多数运行不会开 detailed（本次就没开，导致那约 18.5 ms 无法归因）；写进 summary 后每次运行都留下分阶段数据，代价只是每个 1 s 窗口多十几列。
  2. `planner_rog_map.performance.detailed_csv_enable` 由 `false` 改为 **`true`**（**本次为排障临时打开，跑完一次应改回 false**），配置注释里写明用途与回退。
  3. `test/smoke_goal.py` 的 CSV 隔离补一个洞：配置里没有显式 `detailed_csv_path` 时（默认值指向 `/tmp`），原实现只做"替换已存在的行"，于是 detailed CSV 既没被重定向、冒烟侧也读不到；现在找不到该键就在 `summary_rate` 旁按同样缩进补一行。实测冒烟产出 `.scratch/smoke_run/rog_map_detailed.csv`（230 行，新列齐全），实车 CSV（`.scratch/rog_map_perf_summary.csv`，124 行）在冒烟前后 md5 不变（`bc8d924e6e…`）。
- 验证：
  - Release 编译通过（`rog_map` 61 s、`mas2027_nav_executor` 50 s）；`colcon test` **10/10 通过**（0 失败）。
  - **回归**：`test/smoke_goal.py` + 原始 `lab3_terrain.msgpack` → `goal smoke passed: 52 trajectory poses, 48 global plan poses, marker width 0.150 m`（退出码 0）。
  - summary 新列实测有值：冒烟 detailed 里 `total_update_time_ms 5.22 / update_period_ms 6.72 / update_unaccounted_ms 1.23`，与"周期 = 自报 + 未计入"的定义自洽。
  - 基准（`.scratch/rog_bench.py`，本轮扩充了 `--viz` / `--viz-topics` / `--viz-rate`）：订阅端 QoS 必须与发布端一致（`QoS(1).best_effort()`），否则 `get_subscription_count()` 恒为 0、`captureVizFrame` 根本不会被调用——这一点踩过一次，已在脚本注释里写明。
- 未验证 / 仍未解决：
  1. **那约 18.5 ms（`prob_update` + `decay` + `query_refresh`）还没有实车数据**：本轮 summary 只报了总耗时，detailed 是空的。下次运行（detailed 已开）即可定案；在此之前"18.5 ms 全在 prob_update"是根据基准外推的**估计**（基准里 9.6k 点时 `prob_update` 约 14.8 ms、`raycast` 9.1 ms，与实车 raycast 9.30 ms 吻合，故点数约在 1 万量级）。
  2. **`update_period` 中位 51.95 ms 仍略高于 50 ms 预算**，即当前配置在 20 Hz 输入下**天生会丢约 4.8% 的帧**；要根治必须把单轮压到 ~45 ms 以下（本轮只压掉了 viz 的 2~3 ms）。
  3. **刹车成因细分刚上线，未跑实车**：`reason=terrain_transition` 还是 `reason=tf_unavailable`、`reason=clearance` 各占多少，要等下一次运行；这直接决定"卡"该往哪条线修。
  4. **"一卡一卡"的结构性成因（优化器/走廊只吃 ROGMap ESDF、静态地形只事后否决）本轮同样未动** —— 这是前两条记录里已定性的问题，本轮只是把它与"积压"在证据上彻底分开。
  5. 本次运行的节奏差异需要一起看：2 个目标 / 129 s（每目标 64 s），而 21:04 那次是 4 个目标 / 57 s（每目标 14 s）——路线与探索量不同（本次 `last_unknown_count` 中位 22262、free 中位约 7.8k→17.8k），"感觉更卡"里有一部分是任务构成差异，不宜全部归给积压。

## 2026-09-16 — 积压没根治：脏列增量只把丢帧从 35% 压到 9.5%，补齐"看不见的耗时"插桩

- 起因：用户报"又变成一卡一卡的前进"。复查工作区内最新一份实车 CSV（`.scratch/rog_map_perf_summary.csv`，**21:04:48–21:05:44，55 个 1 s 窗口**，即上一轮开 `dirty_column_enable` 之后的那次运行）：

  | 指标 | 上一轮（20:53，未开脏列） | 本次（21:04，已开脏列） |
  | --- | --- | --- |
  | `cloud_callback_hz` 中位 | 19.96 | **20.00** |
  | `map_update_hz` 中位 | 12.83 | **18.29** |
  | 更新/输入 比值 | 0.65（丢 35%） | **0.905（丢 9.5%）** |
  | `last_total_update_time_ms` 中位 / 最大 | 73.4 / 128.8 | **45.92 / 71.73** |
  | `last_projection_time_ms` 中位 | 43.5 | **15.03**（最大 33.41） |
  | `last_raycast_time_ms` 中位 | 8.7 | **8.76**（最大 16.96） |

  ⇒ 脏列增量**确实生效了**（投影 43.5 → 15.03 ms，丢帧 35% → 9.5%），但**没根治**：单次更新中位 45.9 ms 已经贴住 20 Hz 的 50 ms 预算，最大值 71.7 ms 仍会让工作线程丢帧（`updateWorkerLoop` 只取最新一帧）。"一卡一卡"与这个尾部分布吻合。
- **本轮最重要的一条：`last_total_update_time_ms` 并不是更新周期。** 它只覆盖 `ROGMap::updateMapInternal()` 内部；而 `updateWorkerLoop`（`rog_map_ros2.hpp`）在更新返回**之后**还要在同一线程里跑 `captureVizFrame()`——RViz 一旦订阅了 `/rog_map/layer_value` / `layer_type` / `layer_confidence` 等话题，每轮就要遍历整张 200×200 投影层（每项 40401 格，多项叠加就是十万级）。这段既不在 CSV 任何一列里，也直接推迟下一次地图更新。**因此"CSV 显示 45.9 ms"与"实际 18.3 Hz（=54.6 ms/轮）"之间的差，最可能就是它。**
- 修改（`mas2027_perception/rog_map`，插桩为主 + 一处无损裁剪）：
  1. `performance_monitor.{hpp,cpp}`：`RuntimeStats` 新增 **`update_period_ms`**（相邻两次更新返回之间的真实墙钟间隔）与 **`update_unaccounted_ms`**（= 周期 − `total_update_time`，即上面那段看不见的开销）；`observeUpdate()` 里用 `steady_clock` 直接测，两个字段进了 detailed CSV，零额外遍历。
  2. detailed CSV 补齐此前**缺失/被覆盖**的分阶段列：`raycast_parallel_time_ms`、`raycast_merge_time_ms`、`cache_count`、`hit_count`、`miss_count`、`active_cell_count`、`dirty_column_count`、`dirty_expanded_column_count`、`projection_update_full_time_ms`、`projection_update_dirty_time_ms`、`projection_mask_filter_time_ms`、`projection_value_mask_time_ms`、`projection_count_cells_time_ms`、`fused_projection_time_ms`、`layer_config_sync_time_ms`、`layer_dirty_merge_time_ms`、`projection_scanned_voxel_estimate`、`projection_z_layers`、`update_robot_state_time_ms`。其中 raycast 细分原来恒为 0：`raycastProcess()` 写好的值会被紧随其后的 `probabilisticMapFromCache()` 开头的 `runtime_stats_ = RuntimeStats{}` 整表重置（`prob_map.cpp:351`），现在在 raycast 后先存副本、更新结束再写回。
  3. 新增 **全量回退归因**四列 + 一个标记列：`full_reason_geometry` / `full_reason_explicit` / `full_reason_dirty_disabled` / `full_reason_dirty_over_ratio` / `full_layer_flag_at_consume`，以及 `mark_dirty_ok_count` / `mark_dirty_out_of_map_count` / `mark_dirty_invalid_count`（`markDirtyColumn()` 三个出口的计数）。作用是把"这一帧为什么突然变慢"从猜变成读：脏列超 `dirty_full_ratio` 会静默退回全量（43.5 ms 那一档），越界计数持续 >0 则说明射线打到滑窗外的格子在每帧把整图打成全量。
  4. `rog_map.cpp::refreshLayers()`：**把扫描 z 范围裁到局部地图真正拥有的层**，并在裁剪后为空时整列短路返回空 `ColumnStats`。配置窗口是相对雷达的 −1.2/+2.75 m，而 `map_size.z` 只有 2.5 m，窗口里约一半的层恒在局部地图之外，每个柱子都要为它们跑一遍 `rawGridType()`（内含 `insideLocalMap` + 哈希）后拿到 `OUT_OF_MAP` 丢掉。裁剪后逐格统计结果**不变**（越界格子本来就不计数），属无损改动。
- **本轮实测证伪/修正的两个假设（避免下次走回头路）**：
  - `raycasting.parallel_enable` **默认 false**（`config.hpp:231`，本仓库配置也没设 ⇒ false），所以此前"调 `performance.raycast_num_threads`"是空操作；实测把并行打开并给到 4/8/16 线程，**反而更慢**（raycast 20.9 → ~120 ms）：并行路径的合并段（`uniqueVec3i` 排序 + `insertUpdateCandidate`）在并行阶段要处理上百万个 miss 格，而串行路径是用 `operation_cnt[hash_id]` 哈希表在插入时就地去重的。**结论：不要开这个开关**，要提速得改合并策略，不是加线程。
  - 一堆"省几个整数运算"的微优化（复用 `id_g`/格心、按哈希直接入脏列）实测**在噪声内无收益**（甚至看起来更慢），已全部回退，保持 diff 最小：这一层已经是内存带宽/随机访问受限，不是算术受限。
- 验证：
  - Release 编译通过（`rog_map` 约 60 s、`mas2027_nav_executor` 43 s），`colcon test` **10/10 通过**（`mas2027_nav_executor` 9 项 + `mid360_driver` 1 项，0 失败）。
  - **回归**：`test/smoke_goal.py` + 原始 `lab3_terrain.msgpack` → `goal smoke passed: 52 trajectory poses, 48 global plan poses, marker width 0.150 m`（退出码 0）。
  - **量化基准（新增 `.scratch/rog_bench.py`）**：用与实车完全相同的 `planner.rog_map` 参数起 `rog_map_node`（独立域 231、`ROS_LOG_DIR` 指向可写目录），以 20 Hz 喂合成稠密点云并记录 detailed CSV。**实测 96k 点/帧时单次更新 170 ms（raycast 77 + prob_update 54 + projection 35），直接坐实"点数才是总量纲"**；密度扫描给出对照：9.6k 点 ≈ raycast 9.1 / prob 14.8 / proj 30.7 / 合计 59.1 ms，与实车（raycast 8.76、投影 15.03）量级吻合 ⇒ **实车 `/cloud_registered` 约在 1 万点量级**。该脚本是后续调优的标尺（注意：合成场景的脏列数会顶到 `dirty_full_ratio`，投影走全量，比实车更悲观）。
  - z 层裁剪：本机基准里 `z_layers` 恒为 50（局部地图 z 本就覆盖了整个窗口），因此**本配置下实测无可测收益**，保留它是为了边界情形（雷达 z 偏离地图中心时不再空转两百万格），下文如实记录。
  - **顺带修掉一个会毁数据的坑**：`test/smoke_goal.py` 原先直接用 `--params-file src/.../config/*.yaml` 启动，而 `rog_map.performance.summary_csv_path` 指向工作区里的实车 CSV，`PerformanceMonitor` 又用 trunc 打开 ⇒ **每跑一次冒烟就把实车那一次的数据覆盖掉**（本轮实测踩到，21:04 那次的 55 行被冲成 1 行）。现在脚本先把三份配置复制到 `.scratch/smoke_run/config/` 并把两个 CSV 路径改到该目录再启动；已实测：冒烟前后实车 CSV 的 md5 不变（`500813763e…`）。
- 未验证 / 仍未解决：
  1. **积压没根治**：实车单次更新中位 45.9 ms 仍在 50 ms 预算内贴着跑，最大值 71.7 ms 会丢帧；**尾部的构成还没有实车数据**。下一步需要**跑一次带 detailed CSV 的运行**：把 `planner.rog_map.performance.detailed_csv_enable` 改为 `true`（每次更新一行、约 20 行/s，开销可接受，跑完改回 false），即可一次性拿到 `update_period_ms` vs `update_unaccounted_ms`（可视化快照到底吃掉几毫秒）、`full_reason_*`（是否在静默退回全量）、`prob_update_time` / `decay_time` 等此前没有的列。**本轮没有实车，所以上面这条推断（captureVizFrame 是尾部主因）只是最可疑项，未被数据证实。**
  2. **"一卡一卡"的结构性成因仍未处理**：优化器与走廊生成器只吃 ROGMap 的 ESDF，静态地形只在 `validateTrajectory` 里事后否决（本文件 2026-09-16 前两条已定性）。本次运行仍留下 `Braking` 34 条（其中 `terrain layer … violates terrain` 18 条、`current dynamic obstacle intersects the MPC reference horizon` 16 条）、`Terrain rejection` 3 条、`Repaired a ROGMap-blocked global segment` **20 次且几乎恒定每 2.05 s 一次**（= 全局重规划节拍命中同一处受阻段）。这属于另一条线（统一两图口径 / 让优化器看到静态地形），本轮未动。
  3. z 层裁剪在实车（含斜坡、雷达 z 起伏）下的收益未测；本机无可测收益，若担心风险可整段回退（改动集中在 `refreshLayers()` 的 `z_min/z_max` 与 `scanner` 两处）。

## 2026-09-16 — 积压坐实（输入 20 Hz / 建图 12.8 Hz，投影占 59%）→ 开脏列增量；并抓到"真墙在 ROGMap 里变成可通行"的现场

- 起因：用户报"跑完了"。本次运行 `mas2027_nav_executor_node_80297_1789563189515.log`（20:53:10–20:55:38，148 s，114 行，**2 目标 / 2 到达**），同时 `.scratch/rog_map_perf_summary.csv`（**142 秒样本，路径已改到工作区，这次读到了**）给出了 ROGMap 的逐秒统计。
- **结论 1：积压坐实，且根因是"每帧全量重投影"。** CSV 实测（142 s 中位）：

  | 指标 | 值 | 含义 |
  | --- | --- | --- |
  | `cloud_callback_hz` | **19.96** | 点云输入频率 |
  | `map_update_hz` | **12.83**（最小 9.1） | 建图实际频率，比值 **0.65** ⇒ 约 **35% 的帧被丢弃** |
  | `last_total_update_time_ms` | **73.4**（最大 128.8） | 单次更新耗时 > 帧周期 50 ms ⇒ 物理上跟不上 |
  | `last_projection_time_ms` | **43.5**（占 59%） | 二维投影是瓶颈 |
  | `last_raycast_time_ms` / `last_field_time_ms` | 8.7 / 1.3 | raycast 与 ESDF 都不是瓶颈 |

  这正是终端那条 `[ROG WARN] Unfinished frame cnt > 1 (dropped N)` 的来源，也解释了用户观察到的"突然出现的障碍反应差"。
- **修改 1（参数，可一行回退）**：`planner_params.yaml` 的 `rog_map.performance.dirty_column_enable` 由默认 **false → true**（`dirty_full_ratio` 保持 0.30，脏列超比例自动回退全量）⇒ 投影层改走 `updateDirty` 增量分支，不再每帧重算整张 200×200 窗口。**这是行为改动**，配置注释里写了上车要做的 RViz 检查（无幽灵残留、移动障碍能消失）与回退方法。
  验证（冒烟，**静态场景、点云仅 121 点**，属最优情况）：`projection 43.5 → 2.39 ms`、`total 73.4 → 5.15 ms`。实车动态场景脏列更多，收益会小于此值，需以实车 CSV 复核。
- **结论 2（"卡"的真正来源，已抓到现场）**：本次 5 条插桩日志全部指向同一面墙——
  ```
  Terrain rejection #1: stage=edge_static map=(3.638,1.310) cell=(164,185) cost=100 prev_cell=(164,184) prev_cost=66 odom=(3.427,0.738) frame=map yaw=0.041 | rog_clear=0.753 rog_free=1 rog_value=0 rog_status=OK
  Terrain rejection #5: … map=(3.748,1.502) … | rog_clear=0.356 rog_free=1 rog_value=255 …
  ```
  即：**静态图 `cost=100`（占据）的地方，ROGMap 给出 `rog_value=0`（FREE/PASSABLE）与 0.75~0.89 m 净空**。
  三条独立证据表明这面墙是真的、而且激光看得见它：① 解包 `lab3.pcd`，map (3.3~3.6, 1.1~1.5) 有 77 点、z 到 3.73 m；② 本次 `odom_localizer` 配准 `inliers=36462 error=0.0292 overlap=0.9998`（点云与先验图几乎完全重合）；③ 由日志里"同一轨迹点的 map/odom 对"反推出的 `map->odom` 平移 (0.244,0.432) 与定位器实测 (0.245,0.448) 相差 1.6 cm，坐标系无误。
  ⇒ 墙是实的、被观测到了，却在 ROGMap 里"消失"。ROGMap 的格子统计给出线索：`free 16362 / passable 2854 / occupied 1392 / unknown 20046` —— **`passable` 是 `occupied` 的两倍**，而 `applyValueAndMask()` 里 `PASSABLE` 的 `mask` 恒为 1（自由），`passable_as_free` 只影响 value。分类规则 `classifyCell()` 中 `height_delta <= surface_height_delta_max(0.1)` 一律判 `PASSABLE`：墙面在掠射角下只被打出一条很薄的竖直切片时，就会落进这一档 ⇒ 墙变成"可通行"。
- **结论 3（为什么这会让车反复刹停）**：静态地形被**三个层次**消费，而优化器只吃 ROGMap：
  1. `validateTrajectory` 发布前否决（本次 5 次，集中在目标刚下发的 1.8 s 内）；
  2. **MPC 命令门** `command_safety.cpp:42` 的 `terrain->transition(...)`（本次 `Braking: terrain layer or map transform unavailable, or next command violates terrain` **16 条**，且该日志 2 s 限频 ⇒ 实际刹车时刻更多，覆盖 t=57~114 s 的整段行驶）；
  3. 全局搜索（SMAC 在静态图上，本来就绕开墙）。
  于是：优化器在 ROGMap 里看到的路是通的 → 生成穿过真墙的轨迹 → 被 (1) 否掉或 (2) 刹掉。**这才是"卡"的结构性原因，不是阈值问题。**
- 未改动：所有净空阈值（`collision_dist 0.28`、容差 0.02、`safe_dist 0.33`）、地形门判据、`unknown_as_occupied`、`passable_as_free`、`surface_height_delta_max`。
- 未验证：① 脏列增量的实车收益（冒烟是静态最优情况）；② 墙在 ROGMap 里消失的确切机制（掠射角薄切片 / decay 清格 / 其他）尚未定案——下一步可在 RViz 看 `/rog_map/occupied` 在那处是否有墙，或给分类结果加插桩；③ **结构性问题未处理**：让优化器/走廊也吃静态地形（或让两图一致）才能根治，属代码改动，尚未实施。

## 2026-09-16 — 地形门插桩第一次抓到真现场：MINCO 在真墙上切角，而 ROGMap 说那里是空的

- 起因：用户报"又跑了一遍"。本次运行 `mas2027_nav_executor_node_71327_1789562730373.log`（20:45:30–20:46:40，70 s，88 行，**2 目标 / 2 到达**，启动行 `performance=on` 证明配置已生效）。
- **插桩抓到的现场**（t=12.8~14.6，目标刚下发后 1.8 s 内连续 7 次，都在同一处）：
  ```
  Terrain rejection #1: stage=edge_static map=(3.461,1.285) cell=(161,184) cost=100
    prev_cell=(160,184) prev_cost=66 odom=(3.356,0.551) frame=map yaw=0.198
  ```
  配合 `Terrain global search input` 的 `start_world`（**map 系**：t=12.5 时 (0.287,0.124)）读出的时间线，可得：机器人此刻在 map≈(0.29,0.12)，而**被否决的轨迹采样点在它前方 3.4 m**（odom (3.36,0.55)）——先前一度误以为"否决点就在车身边"，此处更正。
- **那面墙是真的**：解包 `pcd/lab3.pcd`（binary_compressed，自写 LZF 解压 + 字段分块解析）后，map (3.3~3.6, 1.1~1.5) 内有 **77 个点，z 从 −0.04 m 到 3.73 m（中位 1.87 m）** → 是实打实的高墙，不是矮障碍，也不是"被 tunnel 判成可通行"那一类。静态图与点云一致，**地图没错**。
- **矛盾点**：同一段轨迹先过了净空门（ROGMap，阈值 0.26）才被地形门否决 ⇒ 在 map (3.46,1.29) 这一点上，**静态图说"占据"、ROGMap 却说"净空 ≥0.26 m"**，两张图结论相反。根因方向：MINCO 优化器与走廊生成器**只看 ROGMap**（`corridor_generator.cpp:33-42` 的 `box_half_size` 由 ROGMap 的 ESDF 推出），静态地形只在 `validateTrajectory` 里**事后否决**，从不参与优化 ⇒ 空旷处走廊盒极大，轨迹可以切进静态墙体，然后在发布前被一刀否掉——表现为目标刚下发就连续否决、车起不来步。
- 修改（**继续只加日志**，不动任何判据/阈值/行为）：
  1. `minco_planner.cpp` 的地形否决日志追加**同一时刻、同一地点的 ROGMap 读数**：`rog_clear`(ESDF 净空) / `rog_free` / `rog_value`(格子代价值) / `rog_status`。判读规则写在代码注释里：地形 cost≥95 而 `rog_clear` 很大且 `rog_free=1` ⇒ 两图结论相反，问题在"优化器只看 ROGMap"这一结构，不是阈值（此时调阈值只会误伤）；若 `rog_clear` 很小 ⇒ 两门本就一致，问题在别处。
  2. `planner_params.yaml` 的 `rog_map.performance.summary_csv_path` 由 `/tmp/rog_map_perf_summary.csv` 改到 **工作区内** `…/.scratch/rog_map_perf_summary.csv`。原因：agent 侧 shell 的 `/tmp` 是**每次调用独立的临时目录**（写入 `dsh_probe.txt` 后下一次调用即消失，已实测），而实车运行在用户自己的 shell 里写真实 `/tmp` ⇒ 20:45 那次运行虽然 `performance=on`，CSV 却读不到，白跑一趟。
- 验证：
  - Release 编译通过（26 s），构建日志 **0 warning/error**；`ctest` **9/9 通过**（0.14 s）。
  - **回归**：`smoke_goal.py` + 原始 `lab3_terrain.msgpack` → `goal smoke passed: 52 trajectory poses, 48 global plan poses`（退出码 0），无 Terrain rejection。
  - **触发式**：用此前那张"把障碍画在起点→目标直线上"的造图跑同一脚本，新字段如期出现：
    `… | rog_clear=6.000 rog_free=1 rog_value=255 rog_status=OK` —— 即"静态图 cost=100 / ROGMap 净空 6 m、判为自由"的相反结论被完整记录，正是要在实车上验证的形态。
  - **CSV 落盘且跨命令可读**：`.scratch/rog_map_perf_summary.csv` 已生成并在后续独立命令中读到（表头 + 数据行），`last_projection_time_ms`(38.7) 仍远大于 `last_raycast_time_ms`(0.25)，与上一轮判断（投影层是主要开销）一致。注意冒烟脚本的 `cloud_callback_hz` 是脚本自己刷出来的（165 Hz），**实车数值才有意义**。
- 未验证：**未跑实车** —— 新增的 `rog_clear/rog_free/rog_value` 在真墙上到底读出什么（即"ROGMap 是没观测到那面墙，还是观测到了却被 decay 清掉"）仍未知，这正是下次运行要回答的问题；实车的 `cloud_callback_hz` vs `map_update_hz` 差距也仍未知。本轮**未改任何阈值与判据**，通过率不会有变化。

## 2026-09-16 — 车体尺寸实测 612 mm → 撤回"降 collision_dist"；打开 ROGMap 性能 CSV 定位建图积压

- 用户提供两条关键现场信息：**车体旋转直径 612 mm**；最新一次运行"效果明显比上次好（safe_dist 回退 0.33 之后），但仍会卡在动态障碍上——**提前观测到的障碍没事，突然冒出来的就反应差**"；并在终端看到 `[ROG WARN] Unfinished frame cnt > 1 (dropped N), the map may not work in real-time`。
- **结论 1（安全，优先级最高）：不要再下调 `collision_dist`。** 旋转直径 612 mm ⇒ 外接圆半径 **0.306 m**，比有效硬阈值 0.26 m（= 0.28 − 0.02）大 46 mm。两者之所以在实车上仍能对上，是因为 ROGMap 的净空读数**偏保守**（障碍格 0.05 m，点落在格内即整格判占据，读数一般比真实几何净空小 2~5 cm），即"读数 0.26"≈"真实 0.29~0.31"，正好贴着车体外接圆。因此此前讨论过的 `collision_dist` 0.28 → 0.27 建议**撤回**：它会把有效阈值压到 0.25，比读数保守量能补回来的还低，等于重新允许蹭墙（9-16 撞墙那次的有效阈值就是 0.20）。窄道通过性的进一步收益应来自**降低感知延迟**与统一判据口径。该结论已写进 `planner_params.yaml` 的 `collision_dist` 注释。
- **结论 2（积压会不会影响：会，且机制与现场观察吻合）**：`rog_map_ros2.hpp:379-412` 的 `updateWorkerLoop` 是**单后台线程串行**做 raycast → 二维投影 → ESDF → 可视化快照，且**只取最新一帧、中间帧直接丢弃**（`dropped - 1` 即被覆盖的帧数）。所以积压的后果不是"数据旧"，而是**地图更新频率低于点云频率 → 突发障碍进图变慢 → 规划器在它进图前仍按旧图算路**，这正是"提前观测到就没事、突然出现就反应差"的特征（属于延迟问题，不是阈值问题）。另注意 `/dynamic_cost_map` 被 launch 里的 `bypass_dynamic_obstacle: True` 关成空图，动态障碍**完全依赖 ROGMap 的实时性**，积压会直接放大该短板。
- **修改 1（只开诊断，不改行为）**：`planner_params.yaml` 的 `planner.rog_map.performance` 由 `enable: false` 改为 `enable: true` + `summary_csv_enable: true`（`summary_csv_path: /tmp/rog_map_perf_summary.csv`、`summary_rate: 1.0`），`detailed_csv_enable`/`print_enable` 保持关闭（前者每次更新写一行、开销大；后者只打到 stdout，不进 ROS 日志文件）。CSV 每秒一行，含 `cloud_callback_hz`（输入频率）、`valid_update_hz`/`map_update_hz`（**实际更新频率，明显低于输入即坐实积压**）与 `total_update_time` / `raycast_time` / `projection_time` / `field_time` 等分阶段耗时。配置键名实测为 `planner.rog_map.performance.*`（`config.hpp:397-407`），`raycasting.parallel_enable` 属于 **raycasting** 段而非 projection 段（`config.hpp:231`）。
- **修改 2（注释记录）**：`collision_dist` 段补上 612 mm 实测值与"不得再降"的理由。
- **顺带查到的最大嫌疑（未改，待数据）**：`performance.dirty_column_enable` **默认为 false**（`config.hpp:374/557`），本仓库配置也未设置 ⇒ 投影层**每一帧都对整张二维图做全量重算**（`ProjectionLayer::updateFull` 遍历全部 cell），脏列增量路径 `updateDirty` 从未启用。冒烟实测（静态场景、点云仅 121 点）单次更新 `total 37.4 ms`，其中 **`projection_time 34.1 ms`（占 91%）**、`raycast_time 0.32 ms`、`field_time 0.88 ms` ⇒ 投影是主要开销，且与点数关系不大（窗口 201×201 全量遍历）。实车点云更密，届时以 CSV 为准。
- 验证：`yaml.safe_load` 解析 `src`/`install` 两份均 `performance.enable=true`、`summary_csv_enable=true`、`safe_dist=0.33`、`collision_dist=0.28`；`test/smoke_goal.py` + 原始图 = `goal smoke passed: 52 trajectory poses, 48 global plan poses`（退出码 0），节点日志出现 `performance=on`，**CSV 成功落盘**（`/tmp/rog_map_perf_summary.csv`，表头与一行数据已核对，字段名与预期一致）。
- 未验证：**未跑实车** —— 实车 `cloud_callback_hz` vs `map_update_hz` 的差距、各阶段真实耗时、以及 `dirty_column_enable` 打开后能否消掉积压，全部待测。`dirty_column_enable` 属于行为改动（走增量更新路径，脏列超 `dirty_full_ratio`(0.30) 时回退全量），若要启用应先确认 RViz 里地图更新正常、无残留幽灵障碍、`last_projection_time_ms` 确实下降。

## 2026-09-16 — 回退 safe_dist 0.30 → 0.33：降软目标把"起步顿挫"放大

- 现象：用户反馈"重新跑了一遍，改完之后起步更卡了"。
- 现场（`mas2027_nav_executor_node_51164_1789561714784.log`，20:28:34–20:29:55，80 s，140 行）：**1 目标 / 1 到达**，失败 `total=50 [COLLISION=49 LOCAL_SEED_INVALID=1]`。目标 t=11.1 下发后，**t=12.5~14.4 连续 10 次 COLLISION**（净空 `0.258` / `0.255` 对要求 `0.260`，**差 2~5 毫米**）→ t=14.4 急停；全程 **7 次急停 / 12 次 Braking**，到 t=52.3 才到达；t=52.3 之后无目标，日志在 t=82 被 SIGINT 结束。
- 三次运行拉平口径（都换算成"每分钟"频率，避免被运行长短误导）：

  | 运行 | safe_dist | 时长 | 急停 | Braking | COLLISION 占比 |
  | --- | --- | --- | --- | --- | --- |
  | 19:50 `..._7425` | 0.33 | 743 s | 1.2 /min | 2.5 /min | 88/250 = 35% |
  | 20:13 `..._33562` | 0.30 | 103 s | 2.3 /min | 5.8 /min | 22/50 = 44% |
  | 20:28 `..._51164` | 0.30 | 80 s | 5.2 /min | 9.0 /min | 49/50 = 98% |

- 判断：顿挫频率随 `0.33 → 0.30` 成倍上升，机制与 `minco_optimizer.cpp:320` 附近的注释一致——软目标是"优化器只能渐近逼近"的目标，**必须比硬判据高出余量**（原文即记录过"软目标恰好等于硬判据时，解会稳定地差几毫米被 validateTrajectory 否掉"）。降到 0.30 后余量只剩 `0.26 → 0.30` 这 4 cm：净空 0.27 处的罚项推力只有 0.33 时的一半，轨迹不再被推回通道中心，于是更频繁地差几毫米撞上检查器的 0.26。而当初降 0.30 想换的"窄段空转"收益**从未被证明**——两次 0.30 运行的 `OPTIMIZER_FAILED` 分别只有 1/50 与 0/50，本次更是 0。
- 修改：`src/mas2027_nav_executor/config/planner_params.yaml` 的 `minco_optimizer.safe_dist` **0.30 → 0.33**（回到之前多轮验证过的值），并把上面三次运行的频率表与归因强度写进配置注释。余量恢复为 `0.33 − 0.26 = 0.07 m`。
- 未改动：`collision_dist`(0.28)、`node.rog_map_clearance`(0.28)、`kMonitorClearanceTolerance`(0.02)、`kNearFieldSlack`(0.02)、地形门判据、以及上一轮加的地形门插桩代码。**"种子门 0.28 vs 轨迹门 0.26"这条不一致本轮同样未处理。**
- 两条附带发现：
  1. 本次是地形门插桩上线后的第一次实车运行，`Terrain rejection` **0 条**，与 summary 中没有 `TERRAIN_COLLISION_OR_DIRECTION` 一致——说明插桩在位且未触发，本次的行为差异不来自插桩（插桩只加日志）。
  2. **到点之后出现新的循环**：t=52.3 到达、其后无目标，但 t=67.7~81.0 出现 **14 次 `Near-field clearance query failed: OUT_OF_MAP; near-field relaxation disabled for this check`**，与 `Trajectory collision detected.` 交替（约 1 Hz）；前两次运行（7425 / 33562）该字符串均为 **0 次**。本次只做回退，未定位、未处理，下次排障需一并看。
- 验证：`yaml.safe_load` 解析 `src` 与 `install`（软链）两份均为 `safe_dist=0.33 / collision_dist=0.28`；不变量脚本校验 `safe_dist > collision_dist`、`collision_dist == node.rog_map_clearance` 通过，并打印余量 0.07 m；`test/smoke_goal.py` + 原始 `lab3_terrain.msgpack` = `goal smoke passed: 52 trajectory poses, 48 global plan poses`（退出码 0）。纯配置改动，无需重编（`install/share` 下 config 为软链）。
- 未验证：未跑实车——回退后"起步顿挫"是否缓解未实测；**归因强度是"强嫌疑而非已证实"**（三条样本路线不同、时长差 9 倍）。回退的第二条理由更硬：它是尚未验证收益的实验，不该同时占住"安全余量"和"排障变量"两个位置。

## 2026-09-16 — 地形门插桩：TERRAIN_COLLISION_OR_DIRECTION 首次留下否决现场（只加日志，不改判据）

- 起因：用户复查最新一次运行后反馈"又出现了这个现象"。该次运行（`mas2027_nav_executor_node_33562_1789560798786.log`，20:13:19–20:15:02，104 s，170 行）**4 目标 / 2 到达**，失败构成 `total=50 [TERRAIN_COLLISION_OR_DIRECTION=26 COLLISION=22 LOCAL_SEED_INVALID=1 OPTIMIZER_FAILED=1]`；最长一次**连续 21 s 卡在 odom (6.77~6.80, 2.29~2.32)**（目标 (0.18,−0.08)），期间以约 2 Hz 反复 `MINCO path generation failed; retrying`，最后靠用户改点目标才脱开。其中 `COLLISION` 早就会打印否决点，而占 **52%** 的 `TERRAIN_COLLISION_OR_DIRECTION` **从不打印位置**，无法判断它是在真实墙体上否决（说明现场挤压是真的）还是在别处（判据/坐标系有问题）。
- 修改（`minco_planner.{hpp,cpp}`，**不动任何判据、阈值或行为**）：`validateTrajectory` 的地形段加插桩，四个分支各留下现场——`start_static` / `start_dynamic` / `edge_static` / `edge_dynamic`，内容为：地形图(map)系坐标、格号、该格 cost、前一点格号与 cost、对应的 odom 轨迹点、地形图 frame 与 yaw。打印策略与既有 failure_log 同源：前 `kTerrainRejectLogFirstN`(10) 次逐条，之后每 `kTerrainRejectLogEveryN`(50) 次采样一条（**刻意不按时间节流**，理由见 2026-09-16 14:55 那次的教训）。同时把原先 `!terrain->transition(...) || !dynamic_clear` 的短路写法改为先求值再判断（求值顺序与语义不变），以便区分静态图否决与动态层否决。
- **本轮读码纠正的一个坐标系陷阱（务必记住，避免下次再踩）**：同一份日志里两组坐标分属两个坐标系——
  - `Terrain global search input` 行的 `start_world/goal_world` 是 **map 系**（该行由 `makePlanOnQuery(start_map.pose, ...)` 打印，`minco_planner` 只是把 output_frame 写进 header，不做换算，见 `global_path_searcher.cpp:311-315` 的注释）；
  - `Queued goal ... in odom`、`Trajectory clearance ... at (x,y)` 是 **odom 系**。
  两者相差现场实测约 0.5 m。本次运行 odom_localizer 收敛值 `map->odom`：`T=(0.604,0.107,0.254)`、`yaw≈−0.193 rad`（overlap=1.0、error=0.029、内点 37k），而**换算方向是 `p_map = R(yaw)·p_odom + t`**（可用"点击目标 → 同一时刻 `goal_world`"自检，残差 2~4 cm；反向换算残差 >1 m）。
- 据此**推翻**了本轮一度得出的"先验图过期"结论：按正确换算，两次运行的 41 个净空否决点里 **0 个**落在先验图障碍格上（先验图中位净空 0.54 m），64 个机器人位置样本也全部位于先验图自由格（中位 0.64 m）。现场卡住处按先验图看是 **1.1~1.9 m 宽**的走廊；是**实时图上新摆的障碍**把净空压到 0.176~0.259 m，再撞上 0.26（轨迹门）/0.28（种子门）两条门。
- 验证：
  - Release 编译通过（65 s），构建日志**无 warning/error**；`ctest` **9/9 通过**（0.15 s）。
  - **回归**：`test/smoke_goal.py` + 原始 `lab3_terrain.msgpack` → `goal smoke passed: 52 trajectory poses, 48 global plan poses, marker width 0.150 m`（退出码 0），日志中 **0 条** Terrain rejection —— 正常路径行为未变。
  - **触发式验证（证明新日志真的会打）**：用脚本把 lab3 图上 `(1.28,0.22)`（起点→目标直线上）画成半径 0.2 m 的硬障碍，另存 `.scratch/terrain_reject/lab3_blob_on_line.msgpack`，同一冒烟脚本跑出 11 条
    `Terrain rejection #1: stage=edge_static map=(1.165,0.112) cell=(115,161) cost=100 prev_cell=(114,160) prev_cost=66 odom=(1.165,0.112) frame=map yaw=0.000`
    并伴随 11 次 `MINCO trajectory not published: TERRAIN_COLLISION_OR_DIRECTION` —— 日志与它要解释的失败原因一一对应。（该造图下冒烟断言 `goal produced no path` 属**预期**：障碍正压在路径上，MINCO 无合法轨迹可发。）
  - 顺带印证：把障碍画在机器人**起点**上时，先被种子门拦住（`dense_reject=terrain ... verdict=TERRAIN` → `LOCAL_SEED_INVALID`），`validateTrajectory` 根本轮不到跑 —— `local_path_processor` 的 terrain 门是更靠前的一道。
- 未验证：**未跑实车**，因此实车那 26 次（上一轮长运行 132 次）TERRAIN 究竟卡在哪里仍未知，要等下一次运行取数据；插桩只加日志，不改变通过率。上一轮的 `safe_dist 0.30` 本次已生效（启动时间 20:13:19 晚于改动 20:09:50），但本次 `OPTIMIZER_FAILED` 仅 1/50，**不足以判断该改动有效与否**——本轮失败由两道硬门主导，与优化器收敛无关。
- 下一步（待定，未实施）：① 用插桩数据给 TERRAIN 定性；② 统一净空两条门（种子门改用 `collision_dist − 容差`，保持 0.26 底线）；③ 若仍不够，再考虑 `collision_dist`/`node.rog_map_clearance` 0.28 → 0.27（底线降到 0.25，接近 9-16 撞墙那次的量级，需谨慎）。

## 2026-09-16 — 窄道卡死结案读码：safe_dist 0.33 → 0.30（零安全影响），并证实"种子门比轨迹门严 2 cm"

- 起因：用户反馈"最新一次运行还是**有概率在变窄处卡住**，但明显可以过"，要求给出下一步该调的参数。本轮**只读日志与源码 + 改一个软目标参数**，未动任何硬判据。
- 现场（`~/.ros/log/mas2027_nav_executor_node_7425_1789559440199.log`，19:50:40–20:03:04，12.4 min，634 行）：**27 个目标 / 22 到达**；5 个未到达的目标全部挤在同一片区域（x≈1.4~3.6、y≈0~1.1）：`(2.46,0.97)`、`(1.44,0.14)`、`(3.14,1.11)`、`(1.60,0.20)`，最后一次 `(2.56,1.01)` **卡 73 s 到收尾未到达**。失败构成：`MINCO failure summary: total=250 [COLLISION=88 LOCAL_SEED_INVALID=30 TERRAIN_COLLISION_OR_DIRECTION=132]`，另有 103 次 `MINCO path generation failed; retrying`。
- **结论 1（卡住的直接原因，本轮已用插桩数据证实）**：种子门与轨迹门阈值不一致。卡死那 73 s 的 4 条否决现场是 `clear=0.275 / 0.274 / 0.271 / 0.262` 对 `req=0.280`（差 **5~18 mm**），四处**全部大于轨迹门的 0.26**；但种子门（`local_path_processor.cpp:329-336`）用的是**裸 `collision_dist` 0.28、不减容差**，于是 `seed.valid=false` → `LOCAL_SEED_INVALID`，`minco_planner.cpp:1073` 直接返回，**轨迹校验根本没机会跑**。这证实了 `docs/refusal_triage_2026-09-16.md` §1.3 的"强嫌疑"。
- **结论 2（判读陷阱，已定位、本轮未修）**：14 条 `Live obstacle blocks the local route...` 里 13 条打的是 `verdict=GEOMETRY`（读作"几何上真过不去"），但按 `local_path_processor.cpp:43-48`，起点净空 **≥** `collision_dist` 时分类器仍用 `start_clearance − 0.02` 当近场规则（实测起点净空 0.342~0.368 → 近场规则 0.322~0.348，**比完整的 0.28 还严**），而同一文件 `:331` 的种子门放宽条件是 `start_clearance < collision_dist_` —— 两条规则不同源，于是"差 5 mm 够不到更严的种子门"被误报成"物理阻断"。
- **结论 3（不是净空问题）**：最大一类失败 `TERRAIN_COLLISION_OR_DIRECTION=132/250` 是 MINCO 轨迹压在**静态先验图**上。已解包核对 `src/mas2027_nav_bringup/map/lab3_terrain.msgpack`（770×347 @0.05 m）：`direction` 数组 267190 格**全为 0**，故 `terrain_grid.cpp::permitted()` 恒走 `magnitude != 255` 的软分支，**方向层不参与判断**，这 132 次就是"撞静态图（`lab3.pgm`）的占据格"。卡死那一分钟里只出现 1 次（`dense_reject=terrain`）。若某处反复报 `verdict=TERRAIN`，要动的是地图对齐/更新，不是净空参数。
- 本次修改（按用户选择，只做单变量）：`src/mas2027_nav_executor/config/planner_params.yaml` 的 `minco_optimizer.safe_dist` **0.33 → 0.30**，并把上述依据写进配置注释。
- 为什么是**零安全影响**：`safe_dist` 只进 `minco_config.magnitudeBounds(0)`（`minco_planner.cpp:494`、`:880`），只被 `minco_optimizer.cpp:267/320` 当作位置罚项的软目标；`TrajectorySafetyChecker::configure` 收到的是 **`collision_dist`**（`minco_planner.cpp:587`），`checkCollision`/运行时监视用 `requiredClearance(v) = collision_dist + max(v·replan_react_time, monitor_margin)`，地形/ROGMap/MPC 各有独立判据 —— **没有任何一条硬判据读 `safe_dist`**。另 `collision_dist` 在 YAML 里显式存在，故 `minco_planner.cpp:317` 以 `safe_dist` 作 declare 默认值那条路径不会生效。改动依据：卡死处实测净空 0.262~0.275，即该窄道几何上限约 0.28 m，而软目标 0.33 要求"通道宽 ≥0.66 m"、现场只有约 0.56 m → 罚项恒满格、L-BFGS 空转。0.30 对应 ≥0.60 m，仍高于硬判据 0.28。
- 未改动：`collision_dist`(0.28)、`node.rog_map_clearance`(0.28)、`kMonitorClearanceTolerance`(0.02)、`kNearFieldSlack`(0.02)、`fill_occ_min`(8)、`denoise_occ_max`(0)、`corridor.robot_radius`(0.30)、`replan_react_time`(0.0)、`monitor_margin`(0.0) 及全部 rog_map 参数。**种子门与分类器两处不一致本轮未修**。
- 验证：`yaml.safe_load` 解析 `src` 与 `install` 两份（后者是指向源码的软链）结果一致；不变量脚本校验 `safe_dist(0.30) > collision_dist(0.28)`、`collision_dist == node.rog_map_clearance`、`1 ≤ fill_occ_min ≤ 8` 且 `denoise_occ_max < fill_occ_min` 全部通过；隔离域 `ROS_DOMAIN_ID=230` 直启节点 10 s，日志出现 `[ROGMap Config] loaded ...` 与 `[MincoPlanner] ...`，**无 throw/terminate/invalid**（只有预期内的 `No point cloud input`）；再以 `ROS_DOMAIN_ID=231` 起节点用 `ros2 param get` **实查运行值**：`/nav_executor_planner planner.minco_optimizer.safe_dist = 0.3`、`collision_dist = 0.28`，`/nav_executor node.rog_map_clearance = 0.28` —— 确认硬阈值确实未动。`install/share/mas2027_nav_executor/config/*.yaml` 是指向源码的符号链接，改源码即生效、**无需重新编译**（配置项不参与编译，故本轮未跑 colcon/CTest）。
- 未验证：未跑实车 —— "窄段空转与 103 次 `retrying` 是否下降""窄道通过率是否改善"均未实测；**卡死的直接原因（种子门 0.28 vs 轨迹门 0.26）本轮刻意未处理**，因此卡住现象预期仍会复现。另注意本次运行末段车被脱困前缀**越挪越贴**（`start_clear` 0.368 → 0.127 → 0.047 → 0.012，最后在 (2.19,0.38) 净空 −0.048 处触发 `bounded escape`），它是"卡一下"变"卡死"的放大器，本轮同样未处理。

## 2026-09-16 — 双雷达一台掉线后进入单雷达模式，导航不断流（含"掉线后再也回不来"的根因修复）

- 需求：双雷达系统里任意一台掉线后自动进入单雷达模式，不影响正常导航使用（用户原话）。
- 现状（改动前，两个独立根因，均已用改动前的构建复现）：
  1. **融合只在两路都有帧时才发**：`collect_merged_frames()` 要求两队都非空才配对，一台雷达静默后另一台的队列涨到上限被丢，`/mid360_driver/lidar` **整段静默**。改动前构建的日志是 `lidar merge is waiting for the back lidar: front_queue=8 back_queue=0` 刷屏；同一份端到端脚本（下）在改动前构建上测得 `/lidar` 从 1.58 s 起**停发 9.9 s 直到脚本结束**，导航等于死掉。
  2. **掉线雷达永远回不来**：`is_timestamp_plausible()` 的时间戳水位线只在接受时前进，所以任何超过 `max_packet_time_jump`(0.5 s) 的静默之后，该雷达的每个包都被判 `implausible timestamp` 并永久拒绝；`NO_SYNC` 模式下雷达重启（内部时钟归零）还会让首包算出的 delta 永久失效。改动前构建日志里 `dropped packet: lidar/imu: implausible timestamp (total drops: 1210)` 持续到进程结束，且恢复帧被拿去和 2.6 s 前的旧帧配对（`dt=2641 ms window=100 ms`）。
- 修改：
  1. 新增 `mas2027_perception/mid360_driver/include/mid360_driver/merge_failover.hpp` + `src/merge_failover.cpp`：把原先埋在节点 cpp 匿名命名空间里的 `RigidTransform/transform_points` 提成可单测的 `MergeTransform/make_merge_transform/transform_merge_points`，新增 `rotate_imu_to_front()`（后雷达 IMU → 前雷达 IMU 系：`ω_front=R·ω_back`、`a_front=R·a_back+ω×(ω×d)`，`d=-t` 为杆臂；角加速度项 `α×d` 不补偿）与 `LidarHealthGate`（每台雷达"新鲜/在线/恢复保持"判据，点云与 IMU 各一个实例）。
  2. `src/mid360_driver_node.cpp`：融合 tick 改为 `merge_pointcloud_tick_locked()`——两台都在线走原配对逻辑；有一台掉线则 `collect_single_lidar_frames()` 把还在线那一路的帧按时间顺序单独发出，掉线那一路的积压直接丢；IMU tick 改为 `merge_imu_tick_locked()`——参考雷达 IMU 掉线时切到另一台（回调里就已换算到前雷达系），两个来源每 tick 都清空，避免同一时刻出现两份 IMU。模式切换打一条限频日志（降级原因、静默时长、累计次数；IMU 换源单独一条 WARN）。融合队列上限由常量改为按窗口计算 `ceil(merge_stale_timeout_s/发布周期)+4`（现场配置 = 14 帧），保证降级生效前不丢帧。
  3. `src/mid360_driver.cpp` + `include/.../mid360_driver.hpp`：新增 `packet_resync_silence`（默认 1.0 s，0 关闭）与 `resolve_packet_timestamp()`，把两个水位线 map 从 `address→double` 改成 `address→TimestampAnchor{timestamp, wall_time}`；静默超过阈值即重锚定（同时重算 `NO_SYNC` 的 delta），其余情况仍按 `max_packet_time_jump` 拒绝跳变。
  4. 参数（两份配置都加，现场值）：`merge_stale_timeout_s: 0.5`、`merge_recover_hold_s: 0.5`、`merge_imu_stale_timeout_s: 0.1`、`packet_resync_silence: 1.0`。语义：**首次**收到某台雷达数据立即上线（不拖慢启动），**掉线后回来**才需连续新鲜满保持期才重新参与配对。
- 验证：
  - Release 编译通过（`-Wall -Wextra -Wpedantic -Wconversion` 无告警）；`mas2027_nav_executor`/`small_point_lio`/`mas2027_nav_bringup` 一并重编无影响。
  - 新增单测 `test/test_merge_failover.cpp`（挂 CTest，`merge_failover`）：外参矩阵与点变换、IMU 旋转（含手算的离心项 `(0,−0.0214117,−0.1993378)`）、在线/掉线/恢复保持/抖动/`stale_timeout=0` 各分支。`colcon test --packages-select mid360_driver` = **1/1 通过**；`mas2027_nav_executor` 回归 **9/9 通过**（本次未改执行器，仅确认无连带影响）。注：本机沙箱里 `~/.ros/log` 只读，跑 ROS 相关测试需 `ROS_LOG_DIR=<可写目录>`，否则 spdlog 直接 abort（看起来像测试失败）。
  - 新增端到端 `test/integration_degrade_check.py`：本机 `127.0.0.2/127.0.0.3` 冒充前后雷达按 Livox 私有协议发 UDP（点云 20 Hz、IMU 100 Hz、`time_type=0`），跑**真实节点 + 真实话题**，11.5 s 时间轴覆盖：后雷达掉线→恢复、前雷达掉线（IMU 换源）→恢复、后雷达重启（设备时钟归零）。结果 **PASS**：点云最大空档 **400 ms**、每个 0.5 s 窗口都有点云、header.stamp 单调、后雷达点在单雷达模式下已在前雷达系 `(0, 1.1948, −0.3300)`、IMU 换源后为 `(0, 0.9943, −0.1068)`、恢复后双雷达融合帧回来了。
  - **反证**：用 `git worktree` 在 HEAD 建基线、单独编出改动前的驱动再跑同一脚本，**FAIL 9 项**（`/lidar` 停发 9.9 s、19 个空窗口、IMU 换源缺失、恢复/重启后均无法重新融合），确认脚本测的就是本次修的行为。
  - 隔离域（`ROS_DOMAIN_ID=232`）用两份真实配置各启动一次，确认加载新参数且无 throw：`... single-lidar degrade after 500 ms (recover hold 500 ms), IMU failover after 100 ms, merge queue 14 frames`。
- 未验证/已知限制：**未接真雷达、未跑实车**——实车掉线判定阈值（0.5 s）是否会在真实丢包下误触发、恢复后重影/外参是否仍然正确均未实测；IMU 换源会带来两台 IMU 之间的零偏台阶（LIO 需重新收敛），切换瞬间的定位抖动未量化；`packet_resync_silence` 对"处理卡顿后补读旧包"的场景会把首个旧包的时间戳记成当前墙钟（仅一帧，未实测影响）。单雷达模式下车体一侧的视野覆盖变差（少一台雷达），导航仍能跑但不等于双雷达的通过性。

## 2026-09-16 — 撞墙后回调净空：collision_dist 0.25→0.28、容差 0.05→0.02

- 现象：把 `collision_dist` 降到 0.25 后规划成功率大幅改善（优化失败 78→9、`verdict=GEOMETRY` 18→2、到达 10/16），但**实车发生撞墙**。
- 根因：真正的安全阈值不是 `collision_dist` 本身，而是 **`collision_dist − kMonitorClearanceTolerance`**。上一轮为消除毫米级否决把容差保持 0.05，于是有效硬阈值被压到 **0.25 − 0.05 = 0.20 m**，**小于车体半宽**，等于允许车体与障碍重叠约 10 cm。该风险在上一条记录的"未验证"里已标注，但未能拦住。
- 修改（四处联动）：
  1. `planner_params.yaml`：`minco_optimizer.collision_dist` 0.25 → **0.28**；
  2. `planner_params.yaml`：`minco_optimizer.safe_dist` 0.30 → **0.33**（软目标须高于硬判据）；
  3. `node_params.yaml`：`node.rog_map_clearance` 0.25 → **0.28**（执行器侧，须与 collision_dist 一致）；
  4. `minco_planner.cpp`：`kMonitorClearanceTolerance` 0.05 → **0.02**（影响监视器与发布门两处，故仍保持一致）。注释说明该余量应保持在"几毫米~2 cm"量级，不得取到接近车体半径。
- 结果：**有效硬阈值 0.20 → 0.26 m**（+6 cm）。实测约 0.59 m 的窄道给出净空约 0.295 > 0.26，**仍可通过**；同时不再允许车体与障碍重叠。
- 未改动：`fill_occ_min`(8)、`replan_react_time`(0.0)、`corridor.robot_radius`(0.30) 及其余参数。
- 验证：`yaml.safe_load` 解析通过；脚本校验 `collision_dist == rog_map_clearance`、`safe_dist > collision_dist`，并打印有效硬阈值 0.26；`mas2027_nav_executor` Release 构建通过（25.8 s）；执行器 **9 项 CTest 全部通过**。
- 未验证：未跑实车——撞墙是否消除、窄道是否仍可通过均未实测；**车体窄边实际半宽仍未测量**（0.28 与 0.02 都是按现象推的）；若仍擦碰，应继续上调 `collision_dist`（0.29→0.30，后者等于按"直径 60 cm"取满，窄道将重新不可过）或把窄道摆宽。

## 2026-09-16 — 发布前校验补上净空容差（与运行时监视一致），消除毫米级否决

- 现象：净空阈值改到 0.25 后大幅改善（收到 16 / 到达 10；`MINCO path generation failed` 78→**9**；`verdict=GEOMETRY` 18→**2**；`OUT_OF_MAP` 30→**0**，证实前一轮修复生效），但剩下的否决几乎全是**毫米级**：`clearance 0.245 / 0.248 / 0.247 / 0.246 / 0.250 m below required 0.250 m` —— 差 **0~5 毫米**。同一模式此前在 0.40 / 0.30 两档阈值上也重复出现过。
- 判断：问题不在阈值取值，而在判据是**严格大于且零容差**（`esdf_dist > check_dist`），而 ESDF 每帧更新、读数本身存在抖动。同一份代码里**运行时监视**早已用 `requiredClearance(v) - kMonitorClearanceTolerance`（`minco_planner.cpp:1845`），**发布前校验**却用不减容差的严格值——而该函数上方注释本就写着"阈值必须与运行时监视一致"，属实现与自身设计矛盾。
- 修改：`minco_planner.cpp` 的 `checkCollision(const traj_opt::Trajectory &)` 中 `options.check_dist` 改为 `std::max(0.0, requiredClearance(v) - kMonitorClearanceTolerance)`（第 1878 行），与监视器对齐；注释完整记录本次依据，以及 2026-09-15 那次"改了又回退"的因果澄清（当时随后的"完全无法规划"瓶颈在更上游的全局搜索失败，与本处无因果；该上游阻塞已由 `isFree`/`allow_unknown`/`OUT_OF_MAP` 三处修复清除）。
- 未改动：`kMonitorClearanceTolerance`(0.05)、`collision_dist`(0.25)、`safe_dist`(0.30)、`rog_map_clearance`(0.25)、全部 rog_map 参数。
- 验证：`mas2027_nav_executor` Release 构建通过（27.3 s）；执行器 **9 项 CTest 全部通过**；grep 确认第 1878 行已生效、第 1845 行监视器分支未受影响。
- 未验证：未跑实车——毫米级否决是否消失、到达率能否由 10/16 提升未实测；**容差 0.05 相当于把有效硬阈值降到 0.20（collision_dist 0.25 − 0.05）**，与车体实际半宽的关系未复核，若出现贴障擦碰应优先调小该容差而不是继续降 `collision_dist`；另外 `Braking`（本次 59 次）仍是并列的主要症状，未处理。

## 2026-09-16 — 按用户指示调小净空：collision_dist 0.30 → 0.25（三处联动）

- 背景：窄道仍卡死。最新一次运行（`mas2027_nav_executor_node_100594_1789546630427.log`，约 1 分钟）收到 4 个目标 / **到达 0**，78 次 `MINCO path generation failed`、18 次 `verdict=GEOMETRY`；净空读数 `0.295 vs required 0.296`、`0.291 vs required 0.297` —— **始终差约 5 mm**。而该通道**遥控旋转车体可以实际通过**，说明车体对准后的窄边半宽小于按"直径 60 cm"推得的 0.30；上游 `navi_minco_bit` 用的也是 0.25。
- 修改（三处**必须联动**，否则规划器放行而执行器仍按 0.30 刹车，即此前多次出现的"口径打架"）：
  1. `planner_params.yaml`：`minco_optimizer.collision_dist` 0.30 → **0.25**（硬判据基准，近场半径亦由它派生）；
  2. `planner_params.yaml`：`minco_optimizer.safe_dist` 0.35 → **0.30**（软目标须高于硬判据；0.35 在约 0.59 m 的通道里给不出 0.35，会重新变成"够不到的目标"）；
  3. `node_params.yaml`：`node.rog_map_clearance` 0.30 → **0.25**（执行器 MPC 下一段指令的净距判据，其注释本就要求与 `collision_dist` 一致）。
- ⚠️ 安全取舍（已写入配置注释）：规划模型仍是**圆**，0.25 相当于按"车体窄边"而非"外廓"规划；车体未对准时圆模型会**低估实际截面**，存在蹭墙风险。若实车出现擦碰，应回调至 0.27~0.28 并考虑摆宽通道。
- 未改动：`corridor.robot_radius`(0.30，备份 SFC 盒子用，语义独立)、`replan_react_time`(0.0)、`fill_occ_min`(8)、`safe_dist` 之外的全部参数。
- 验证：`yaml.safe_load` 解析通过；脚本校验 `collision_dist == rog_map_clearance`（联动一致）且 `safe_dist > collision_dist`；隔离 `ROS_DOMAIN_ID=230` 直启 `mas2027_nav_executor_node`（补 `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu`）9 秒，日志出现 5 行 `[ROGMap Config]`/`[MincoPlanner]`，**无 throw/terminate/invalid**（已吸取 `fill_occ_min: 9` 越界崩溃的教训，改配置后先做启动冒烟）。
- 未验证：未跑实车——约 0.59 m 的通道是否因此可通过、`verdict=GEOMETRY` 与卡死是否消失均未实测；**未实测车体窄边半宽**（本次系按用户指示直接调小，非基于实测值）；蹭墙风险未评估。

## 2026-09-16 — 纠正：fill_occ_min 取 9 会抛异常导致节点启动即死（改为 8）

- 上一版把 `rog_map.projection.fill_occ_min` 由 4 改为 9（意图让"补洞"条件永不成立），**这是一个错误**：`rog_map_core/config.hpp:299` 明确校验
  `if (layer_fill_occ_min < 1 || layer_fill_occ_min > 8 || layer_denoise_occ_max < 0 || layer_denoise_occ_max > 7 || layer_denoise_occ_max >= layer_fill_occ_min) throw std::invalid_argument(...)`，
  9 越界 → 构造 ROGMap 时抛异常 → **节点启动即死**：表现为日志文件 0 行（`mas2027_nav_executor_node_97478_1789546499839.log`）、进程不存在、**ROGMap 全部可视化消失**。
- 纠正：改为 **8**——仅当 8 个邻格全为占据时才填，那属于障碍物内部空洞，**不改变外轮廓、不会把通道补窄**，同时满足 `denoise_occ_max(0) < fill_occ_min(8)` 的校验。代码注释中已加醒目警告：**切勿取 9 及以上**。
- 保留原意图：仍要消除"每侧加粗 1 格 = 5 cm"造成的窄道假性过不去（车体直径 60 cm，半径 0.30 与 `collision_dist` 相等，通道只要严格大于 0.60 m 即可通过，而实测读数 0.288~0.318）。
- 未改动：`mask_filter_en`(true)、`denoise_occ_max`(0)、`collision_dist`(0.30)、`safe_dist`(0.35) 及其余参数。
- 验证：`yaml.safe_load` 解析通过；脚本按 `config.hpp` 的规则判定 `1<=fill_occ_min<=8 && 0<=denoise_occ_max<=7 && denoise<fill` 成立；隔离 `ROS_DOMAIN_ID=230` 直启 `mas2027_nav_executor_node`（补 `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu`）9 秒，日志出现 `[ROGMap Config] loaded ...`、`[ROG-Map] Init, resetMapSize` 与 `[MincoPlanner]` 行，**未再崩溃**。
- 未验证：未跑实车——窄道净空是否升到 0.30 以上、卡死是否消失仍未实测；`fill_occ_min: 8` 相对 4 会保留更多内凹处的小孔洞，对去噪以外行为的影响未评估。
- 教训（写给后续）：本仓库的配置项**有区间校验并在越界时抛异常**，而异常发生在节点构造期，表现为"日志为空 + 可视化全无"，容易被误判为显示问题。改配置后应先做一次隔离域启动冒烟，再上车。

## 2026-09-16 — 关闭投影层"补洞"（fill_occ_min 4→9），修复窄道假性过不去

- 现象：窄处**卡住不动**（不是慢）。日志 `mas2027_nav_executor_node_87489_1789545904187.log`（273 行，约 2.2 分钟）：目标 10 / **到达 2**、`MINCO path generation failed` **135 次**，并有 35 条结构化诊断 `Live obstacle blocks the local route and leaves no safe stopping prefix ... verdict=GEOMETRY`——即规划器判定**几何上不可行**（不是抖动或超时），故保持不动。此前一次运行的净空读数为该处 **0.288~0.318 m**。
- 关键事实：**车体直径 60 cm，半径 0.30 m，与 `collision_dist: 0.30` 相等**。因此只要通道宽度**严格大于 0.60 m** 车就能过（判据是 `esdf_dist > check_dist`，等于 0.30 亦不合格）；反之，地图只要把通道**每侧画粗 1 格（5 cm）**，净空读数就会掉到 0.30 以下，规划器即判定"过不去"。
- 根因：投影层的形态学闭运算。`projection_layer.cpp::fillMask()` 中 `cell.hole_filled = occupied_neighbors >= config.fill_occ_min;`（8 邻域，最多 8），即周围有 ≥4 个占据邻格的自由格会被填成障碍——窄道两侧恰好多为占据格，于是通道被"补"窄，**每侧最多 1 格 = 5 cm**，与实测的 5~10 cm 缺口吻合。
- 修改：`planner_params.yaml` 的 `rog_map.projection.fill_occ_min` 由 4 改为 **9**（等效关闭补洞——邻域上限为 8，条件永不成立）。**只关"补洞"，保留 `mask_filter_en: true` 与 `denoise_occ_max: 0` 的"去噪"**，因此孤立障碍的清除行为不变。
- 未改动：`collision_dist`(0.30，等于车体半径，是安全下限故不动)、`safe_dist`(0.35)、`replan_react_time`(0.0)、`unknown_as_occupied`(false)、`mask_filter_en`(true) 及全部其它参数。
- 验证：`yaml.safe_load` 解析通过并打印确认（`fill_occ_min = 9`、`mask_filter_en = True`、`denoise_occ_max = 0`、`unknown_as_occupied = False`）。
- 未验证：未跑实车——该处净空是否因此升到 0.30 以上、`verdict=GEOMETRY` 与卡死是否消失均未实测。**若读数仍 ≤0.30**，说明通道确实只有 0.60~0.65 m，此时不应再降 `collision_dist`（它等于车体半径，再降等于允许车体与障碍重叠），而应回到物理层面（摆放间距 / 放宽 `collision_dist` 需先实测车体外廓半宽并接受蹭墙风险）。

## 2026-09-16 — safe_dist 0.40 → 0.35：缓解窄道"特别慢"

- 现象（现场摆入若干障碍物后）：窄路段明显卡顿，虽能到达目标但**特别慢**。日志 `mas2027_nav_executor_node_63424_1789544651235.log`（730 行，约 13 分钟）：30 目标 / 27 入队 / **20 到达**；`Repaired a ROGMap-blocked global segment with a local grid-search detour` **145 次**、`MINCO path generation failed` **101 次**、`Trajectory clearance 0.288~0.318 m below required 0.300 m` **38 次**、执行器地形层与动态/净空刹车各 **38 次**。
- 判断（本次只处理第二条，单变量）：优化器位置罚项的软目标 `safe_dist: 0.40` 在几何上要求「通道宽度 ≥ 0.80 m 才够得到」；现场摆入障碍后窄于此的通道里罚项恒不满足 → L-BFGS 收敛困难 → 反复重算 → 走走停停。硬判据只需 `collision_dist`(0.30)，故降到 **0.35**（对应通道宽度 ≥ 0.70 m）。这与 2026-09-15 把 `safe_dist` 由 0.50 回退到 0.40 是同一类问题的继续：0.50 过高（93 次不收敛），0.40 在窄段仍偏高。
- 未处理的另两条（本轮刻意不动，避免多变量混杂）：① 全局搜索跑在静态地形图（SMACND）上，**看不见现场新摆的障碍物**，故 ROGMap 段反复被挡 → 145 次绕行修复（与 `bypass_dynamic_obstacle: True` 关闭动态层有关）；② 硬阈值 0.30 与地图读数 0.288~0.318 挤在刀锋上；③ 附带发现一处**日志语义可疑**：出现 `0.318 / 0.317 / 0.314 m below required 0.300 m`，净空大于要求却报 "below"，说明该行打印的两个数值不是同一对，需读当前源码确认后再据此调参。
- 修改：`planner_params.yaml` 中 `minco_optimizer.safe_dist` 0.40 → **0.35**，注释写入上述依据与方向（0.50 太高、0.40 偏高）。配置改动，无需重新编译。
- 未改动：`collision_dist`(0.30)、`replan_react_time`(0.0)、`node.rog_map_clearance`、`bypass_dynamic_obstacle`、全部 rog_map 参数。
- 验证：`yaml.safe_load` 解析通过并打印确认（`safe_dist = 0.35`、`collision_dist = 0.3`、`replan_react_time = 0.0`）。
- 未验证：未跑实车——`MINCO path generation failed`（101）与窄段通行时间是否下降未实测；0.35 仍可能在更窄的通道里够不到，需按现场最窄通道宽度决定是否继续下调（下限不应低于 `collision_dist`）。

## 2026-09-16 — 修复近场净空查询失败导致整条轨迹被误杀（OUT_OF_MAP）

- 现象（`mas2027_nav_executor_node_63424_1789544651235.log`，730 行，约 13 分钟）：30 个目标 / 27 入队 / **20 到达**，远处目标（含 (10.64, 1.92)、(9.43, 2.05)，均已超出 10×10 m 滑窗）**全部到达**；但日志中成对出现 **30 × `Near-field clearance query failed: OUT_OF_MAP`** 紧接 **30 × `Trajectory collision detected.`**，即整条轨迹被否。同一次运行的其它节点（`odom_localizer`/`map_server`/`tf_maintainer`）**零告警**，故排除定位、TF 与地图服务。
- 根因：`trajectory_safety_checker.cpp` 在近场判据中，取轨迹起点净空失败时直接 `return false`（`Near-field clearance query failed: %s` 后即返回），于是**"查不到起点净空"被当成"轨迹不安全"**。而 `clearance_gate.hpp` 的既定语义明确写着：`current_clearance_ok` 为 false（拿不到当前净空）时**不放宽**，全程按完整 `required` 判——检查器的实现与其自身设计相矛盾。`OUT_OF_MAP` 由 `QueryAdapter::query()` 在查询点落到二维栅格之外时返回（现场推断为远端目标下发车后车跑得快、滑窗尚未跟上）。
- 修改：`trajectory_safety_checker.cpp` 的 `checkTrajectory(...)` 中，起点查询失败时不再 `return false`，改为保留 `start_clearance_ok = false` 继续执行——`makeClearanceRequirement()` 在该分支下 `near_required = required`，即**取消近场豁免这一放宽项**；告警文案改为 `...; near-field relaxation disabled for this check`。逐个采样点的完整净空判据不受影响，**安全底线不放松**（这只是不再应用"近场按不比当前更差"的宽松规则）。
- 未改动：`clearance_gate.hpp`、`makeClearanceRequirement()`、`collision_dist`/`safe_dist`/`replan_react_time`、`QueryAdapter::query()` 的 `OUT_OF_MAP` 判据本身（栅格外返回该状态是合理的，问题在于调用方如何处理它）。
- 验证：`mas2027_nav_executor` Release 构建通过（13.4 s）；执行器 **9 项 CTest 全部通过**（测试数由此前的 7 增至 9，系其他改动新增，非本次所加）。
- 未验证：未跑实车——`OUT_OF_MAP` 误杀是否消失、到达率是否由 20/30 提升未实测；**若失败样本点本身也在栅格之外**，`checkPoint()` 仍会因 `worldToMap`/`query` 失败而返回 false，本次改动对该情形无效（需另行处理"轨迹采样点出图"）；"车跑得快导致滑窗跟不上"这一 `OUT_OF_MAP` 成因仍属推断，未用日志时间戳与地图滑动记录证实。

## 2026-09-16 — 可观测性：失败原因按计数采样、种子否决点插桩

- 起因：上一轮定性（见 `docs/refusal_triage_2026-09-16.md`）发现两个"看不见"的问题——
  ① 158 次发布失败里只有 54 次留下原因；② `Live obstacle blocks the local route...`
  分不清"毫米级净空否决"与"物理阻断"。本轮**只改日志与诊断，不改任何阈值、判据或行为**。

- **① 失败原因从"按时间节流"改为"按计数采样"**（`minco_planner.{hpp,cpp}`、
  `config/planner_params.yaml`）：`finish()` 里第 `failure_log_first_n`(10) 次之后原本走
  `RCLCPP_WARN_THROTTLE(..., 2000)`。这在本轮数据上直接失效：COLLISION 以约 0.5 次/s 持续
  发生，比 2 s 节流窗口更密，109 次被压成 31 条（`reason #115` 出现在 total=158 那次），
  窗口内的现场整片丢失。现改为每 `failure_log_every_n`（新参数，默认 25）次采样一条：
  失败再密也保证等间隔留下样本，样本量可预期。`failure_log_every_n: 1` 表示每次都打，
  `0` 表示退回旧的 2 s 节流行为。

- **② 种子否决点插桩**（`local_path_processor.{hpp,cpp}`）：新增 `SeedRejectInfo`
  （首个否决点的 `point / clearance / required / arc_from_start / near_field_relaxed /
  terrain_blocked / length_limited`），`segmentClear` 在返回 false 前记录**第一次**否决
  （`nullptr` 时零开销），经 `pathClear` 透传到 `searchDynamicDetour` /
  `buildStoppingPrefixCore`，最终由 `Live obstacle blocks the local route...` 一条日志报出
  四层各自的现场：`dense_reject=.. detour_reject=.. prefix_reject=.. escape_reject=..`
  （每项 `(x,y) clear=.. req=.. arc=[nearfield] [len-limited]`）。
  `LocalPathSeed` 增加 `dense_reject` 字段，供调用方与测试读取。

- **③ 自动判读 `verdict=`**（`classifySeedReject()`，`local_path_processor.{hpp,cpp}`）：
  现场靠人读上面那串数字要同时手算**两套判据**——种子门（`segmentClear`：只有起点净空严格
  小于 `collision_dist` 才放宽）与近场规则（`clearance_gate.hpp` / 脱困层：近场外就是完整
  `collision_dist`）——很容易算错，所以直接编进代码，日志里给结论：
  `GEOMETRY`（该点连近场规则都过不了 → 拒绝正确，别动阈值）、
  `SEED_GATE_STRICTER`（该点靠近场规则能过、靠种子门过不了且起点没贴死 → **判据不一致**）、
  `PREFIX_TOO_SHORT`（该点合格，否决来自安全段太短，`len-limited`）、
  `TERRAIN`、`NONE`。

- 实现期间的一处修正：初稿还有一个 `ROBOT_TOO_CLOSE` 分支（"起点贴死、规则本身没问题"），
  写测试时发现它在真实调用路径上**不可达**——种子门一旦放宽就会把该点记成"放宽后的要求"，
  所以"起点贴死但能过放宽规则"这个组合根本不会被记录下来。已删除该分支并在实现处写明原因，
  避免留一个永远走不到、也没法测的 verdict。

- 验证：
  - `colcon build --packages-select mas2027_nav_executor --cmake-args -DCMAKE_BUILD_TYPE=Release`
    通过，无新增 warning；`ctest --test-dir build/mas2027_nav_executor` **9 项全部通过**。
    （沙箱把 `~/.ros/log` 判为只读，需先 `export ROS_LOG_DIR=/tmp/... ROS_HOME=/tmp/...`。）
  - `test_local_path_processor` 新增一条用例（`AnalyticQuery` 解析距离场 + 0.50 m 走廊、
    中线净空 0.25 m < `collision_dist` 0.30 m）：关掉第四层脱困后必须是四层全败，且
    `seed.dense_reject` 必须报出 `required == 0.300`（**完整要求，不是被近场放宽过的 0.23**）、
    `near_field_relaxed == false`、`clearance ≈ 0.25`、否决点落在近场弧长 0.30 m **之外**。
    这条断言同时锁住了"近场放宽只作用于起点附近"这一语义。
  - 定点探针（`.scratch/verify_reject/probe.cpp`，仅编译不入口仓库）实测确认新消息按设计输出：
    `... dense_reject=(0.55,1.52) clear=0.202 req=0.300 arc=0.32 ... prefix_reject=(0.54,1.52)
    clear=0.200 req=0.300 arc=0.31 len-limited ... verdict=GEOMETRY`——净空 0.20 明显低于 0.30
    且无近场放宽，即**该例拒绝本就是正确行为**；同时可见前缀层是 `len-limited`（安全段太短）
    而非净空不足，两层的否决性质在日志里已经分开。
  - `classifySeedReject()` 5 个分支各有直接断言，其中最关键的一条是**同一对数字、只改 `arc`**：
    `clear=0.295 / start_clear=0.31 / arc=0.10` → `SEED_GATE_STRICTER`（近场规则放宽到 0.29
    能过、种子门按 0.30 判过不了），而 `arc=0.50` → `GEOMETRY`（近场外规则退化为完整 0.30）。
    这条同时锁住"近场放宽只作用于起点附近"这一语义。
- 未验证：
  - **未跑实车**。两条改动的实车效果（尤其 `failure_log_every_n: 25` 的日志量是否合适）
    均未上车确认；该参数可在现场直接调。
  - 探针还顺带暴露一条**未解释**的语义：0.50 m 走廊（净空 0.25 < `collision_dist` 0.30）
    在**开着**第四层时会被救成一个 0.24 m 的 creep 前缀并成为**有效**种子。也就是说实车日志里
    `Live obstacle blocks` 只在连脱困前缀都建不出来时出现，那说明当时前方连 `collision_dist`
    那么短的安全段都没有，比 0.50 m 走廊更贴死。这与上一轮"17 次拒绝对应车在 0.318 m 净空处"
    的读数**存在张力**，需实车数据厘清（可能说明那 17 次的否决点不在起点附近，而在路径更远处）。

## 2026-09-16 — 定性：`Live obstacle blocks` 与 `LOCAL_SEED_INVALID` 的出处与真实含义

- 起因：对 `~/.ros/log/mas2027_nav_executor_node_8292_1789541753456.log`（14:55:53 起，294 s）
  里 14 次 `Live obstacle blocks the local route and leaves no safe stopping prefix` 与
  17 次 `LOCAL_SEED_INVALID` 做定性。**本轮只读源码与日志，未改动任何阈值或机制**，
  完整结论见 `docs/refusal_triage_2026-09-16.md`。

- **代码变更（仅一处，措辞，判据与行为零改动）**
  `src/path_planner/trajectory/trajectory_safety_checker.cpp:146-159`：近场放宽的 INFO 日志
  旧措辞为 `Near-field exemption: start clearance %.3f m below required %.3f m`。该分支的
  成立条件（`gate.near_required < gate.required`）在**起点净空高于 required** 时同样成立
  （净空落在 `[required, required + slack)` 即成立），于是会打印出
  `0.318 m below required 0.300 m` 这种 X > Y 的自相矛盾数字。`src/README.md:65-66` 虽已
  提醒"不要按字面读"，但该措辞已再次导致把"近场规则正常工作"误判为"日志拼接错误/阈值不一致"。
  现改为直接打印三个量：`start clearance`、`near requirement lowered to`、
  `full requirement`。**判据、阈值、分支条件均未改动。**

- **主要发现（尚未证实，需插桩）**：两条代码路径的近场放宽门槛不一致。
  `trajectory_safety_checker` 经 `clearance_gate.hpp:50-51` 得到
  `near_required = min(required, max(0, current_clearance - slack))`，等价于
  `current_clearance < required + slack(0.02)` 即放宽；而种子门
  `local_path_processor.cpp:229-232` 的门槛是 `start_clearance < collision_dist_`（严格小于）。
  在净空 `[0.300, 0.320)` 这 2 cm 带内，种子门要求 0.300 而轨迹校验门只要 0.298——
  **种子门更严，因此不会放过不安全轨迹，但会把一条会被轨迹校验接受的路线在种子阶段否掉**。
  本轮现场净空读数（0.318/0.319/0.318/0.314/0.317/0.299/0.292）与反复卡住的
  x ≈ 2.7~3.3 一带与该带吻合，但日志从不打印"被种子门否掉时该点净空"，
  故**仅为强嫌疑，未证实**。

- 同时澄清两条易误读：
  - `Live obstacle blocks ...`（`local_path_processor.cpp:145-150`）是**四层兜底全败**的最终
    else，不是"第三层失败"；其字面"路被挡住"实为"几何路线存在、每段都过不了净空判据"。
  - 17 次 `LOCAL_SEED_INVALID` 中 14 次与 `Live obstacle blocks` 是**同一事件的两条日志**
    （9 次间隔 ≤ 0.45 s，其余 8 次被 2 s 节流吞掉），不应分别计数。
  - `Braking: current dynamic obstacle intersects the MPC reference horizon`
    （`nav_executor_node.cpp:447-450`）真身是 `command_safety.cpp` 的 `DYNAMIC_BLOCKED`
    动态层检查，与净空判据无关，**无需再查**。

- 观察到的可观测性口子（本轮未处理）：158 次发布失败里只有 54 次留下原因。
  `finish()` 的"每种原因前 `failure_log_first_n`(10) 次逐条打印"在长运行中失效——
  第 11 次起退回 `RCLCPP_WARN_THROTTLE(..., 2000)`，COLLISION 以约 0.5 次/s 持续发生，
  把 109 次压成 31 条。summary 行本身正确。

- 验证：
  - `colcon build --packages-select mas2027_nav_executor --cmake-args -DCMAKE_BUILD_TYPE=Release`
    通过，无新增 warning。
  - `ctest --test-dir build/mas2027_nav_executor` **9 项全部通过**。
    （注意：沙箱把 `~/.ros/log` 判为只读，直接跑 ctest 会因 spdlog 写日志失败而
    `Subprocess aborted`；需先 `export ROS_LOG_DIR=/tmp/... ROS_HOME=/tmp/...` 再跑。
    这是环境限制，与被测代码无关。）
  - 日志侧核对：13 目标 / 12 次 `Navigation goal reached`；17 次拒绝的**下一次成功事件间隔
    中位 0.938 s、最大 2.84 s**，即本轮是"每轮约 1 s 短暂停"，**不是"车不动"**；
    73 次 `Repaired`、12 次停车前缀、6 次 creep 前缀、0 次 `Repair rejected`。
- 未验证：
  - **未插桩、未跑实车**。§1.3 的门槛不一致只是读码结论 + 现场数据吻合，未经单次运行的
    定点量化证实；`docs/refusal_triage_2026-09-16.md` §5 给出了最小插桩方案。
  - 措辞改动后**未重跑实车**，新措辞的**实际打印效果未经任何测试覆盖**：
    `ctest` 9 项全过，但本轮 `test_trajectory_safety_checker` 因夹具场景的起点净空高于
    `required + slack` 而**未走进该分支**（改动前的 `LastTest.log` 里曾有该行 INFO），
    故只验证了编译与"判断逻辑未变"，未验证格式化输出本身。

## 2026-09-16 — 排障：规划失败原因可见性、稀疏化丢路径、窄处脱困兜底、install 残留

- 起因：查看最新一次实车运行日志（`~/.ros/log/mas2027_nav_executor_node_8853_1789537465602.log`，
  2026-09-16 13:44:25–13:54:35，609 s）发现问题，并按用户选择实施 4 项修复。当日数据：
  35 个目标 / 16 个到达、`MINCO path generation failed` 350 次、`Repaired a ROGMap-blocked...`
  176 次；其中 **76 次「绕行修复成功后 10 ms 内（中位 0.282 ms）立即失败」，且这 76 次里只有
  16 次留下了失败原因**。对照 13:40 那次运行（104 s、10 个目标、0 到达、122 次失败、
  0 次 `Repaired`）：机器人全程停在 `(-0.007, 0.59)` 未移动，32 次
  `Live obstacle blocks the local route and leaves no safe stopping prefix`。

- **① 失败原因可观测性**（`minco_planner.{hpp,cpp}`、`local_path_processor.{hpp,cpp}`、
  `config/planner_params.yaml`）：`ReplanLocal` 的 `finish()` 原来对失败原因整条 2 s 节流，
  10 分钟运行 350 次失败只留 139 条原因（约 1/2.5），无法判断瓶颈在种子还是在轨迹校验。
  现改为按原因计数：每种原因前 `minco_optimizer.failure_log_first_n`（默认 10）次逐条打印
  `MINCO trajectory not published: <原因> (reason #n, total n)`，之后才退回 2 s 节流；每
  `failure_summary_every`（默认 50）次失败再打一条 `MINCO failure summary: total=... [原因=次数 ...]`。
  两个参数设 0 即恢复旧行为。同时把「局部种子无效」从笼统的 `COLLISION` 拆成
  `LOCAL_SEED_INVALID` 与 `LOCAL_SEED_REJECTED_AFTER_REPAIR`（后者由 `LocalPathSeed` 新增的
  `repair_rejected` 标志判定），并在 `buildSeed` 里为「修复成功但种子仍无效」单独打一条带规模的
  `Repair rejected: seed invalid after repair (dense=.. sparse=.. detour=.. stop_prefix=..)`。

- **② `getSparseWaypoints()` 把合格路径整体丢弃**（`vendor/minco/src/minco_utils.cpp`）：
  碰撞恢复时插入的拐点线段（`path[current_safe_idx] -> path[corner_idx]`）**插入时不校验净空**，
  末尾统一复核一旦发现该段碰撞就 `return {}`，而调用方 `buildSeed` 手里的 `dense_path`
  是已经逐段校验合格的（绕行 A* 结尾也做过 `pathClear`）。
  **实测反例**：随机障碍场 + 0.05 m 栅格 A* 生成的 199 条逐段合格绕行路径中，旧实现丢弃
  **26 条（13%）**；紧凑几何检索 2250 例中丢弃 6 例。在 `buildSeed` 里这等价于
  `seed.valid=false` → `finish(false,"COLLISION")`，与「修复后 0.3 ms 立即失败」的现象一致。
  现改为：插入前必须校验，拐点段不合格就退化逐点推进；末尾复核若仍有不合格段，把该段展开成
  原始稠密点（相邻点之间的段也碰撞才返回失败），不再整体丢弃。
  修复后同一组检索：199 例与 2250 例的返回空次数均为 **0**，且输出段无不合格。

- **③ 窄处第四层兜底**（`local_path_processor.{hpp,cpp}`、`minco_planner.{hpp,cpp}`、
  `config/planner_params.yaml`）：13:40 那次「车不动」是三层兜底同时失效——完整停车前缀要求
  安全段 `0.30(最小前缀) + 0.15(收尾余量) = 0.45 m`，而车前方没有这么长；净空一旦低于
  `collision_dist`，两层路径判据又都无解。新增短距离脱困前缀（creep）：三层都失败时，
  退化成一个「不比当前实测净空更差（减 0.02 m ESDF 抖动余量）」、末速度为零的短前缀。
  **长度上限固定为一个车体半径 `collision_dist`**，整段都落在既有近场放宽半径以内，
  因此没有改动任何安全阈值，也不需要放宽轨迹校验。参数
  `planner.minco_optimizer.stuck_escape.{enable,min_length,buffer}`（默认 `true/0.08/0.05`），
  `enable: false` 恢复三层行为。新增日志
  `Live obstacle leaves no full stopping prefix; escaping with a X.XX m creep prefix.`。

- **④ `install/` 残留**（不改仓库源码，只清理构建产物）：`install/` 下留着 5 个 `src/` 里已删除的
  包（`waypoint_editor`、`minco_planner`、`minco_controller`、`pb_nav2_plugins`、
  `fake_vel_transform`，共 10.3 MB）与 345 个断链符号链接，另有 11 个指向已删源文件的断链。
  `install/_local_setup_util_sh.py` 在 source 时**动态扫描** `install/*/share/colcon-core/packages`，
  所以这些目录会让 `AMENT_PREFIX_PATH` 仍然列出它们；RViz 启动时 pluginlib 因此去加载
  `install/waypoint_editor/share/waypoint_editor/plugin_description.xml`（断链）并报
  5 条 `has no Root Element` ERROR。已删除 5 个残留目录与全部 356 个断链，
  清理后 `find install -xtype l` 为 0、`ros2 pkg prefix waypoint_editor` 返回 Package not found、
  干净环境下 14 个前缀全部可用（`common_libs` 本就不在 `AMENT_PREFIX_PATH` 里：它的
  `package.dsv` 只有构建期 hook、没有 `local_setup.dsv`，与本次清理无关）。

- 验证：
  - `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`（全工作空间 15 个包）通过，无新增 warning。
  - `ctest --test-dir build/mas2027_nav_executor` 9 项全部通过；`local_path_processor` 新增 3 组断言：
    ① 稠密绕行逐段合格时 `getSparseWaypoints` 必须非空且每段合格（旧实现返回空的最小反例，
    障碍矩形 `x=[1.00,1.40] y=[-0.80,0.30]` + 80 点绕行折线，由 2250 例检索得到）；
    ② 窄道场景（0.50 m 宽走廊、中线净空 0.275 m）关闭脱困时为无效种子、开启后必须给出
    长度 ≤ `collision_dist` 且逐段满足近场判据的短前缀；③ 绕行成功时 `repair_rejected` 必须为假。
  - 端到端离线冒烟：`test/smoke_goal.py` 普通版 `goal smoke passed: 52 trajectory poses,
    48 global plan poses, marker width 0.150 m`，`--preempt` 版 `50 / 56`（与本次改动前的
    基线 52/48、54/56 同量级），说明新增参数不破坏正常路径，脱困分支在正常场景不介入。
  - 反例检索与复现程序放在 `.scratch/verify_detour/`（`search.cpp`、`compact_search.cpp`、
    `probe.cpp`、`repro.cpp`，仅链接现有库，不属于仓库源码）。
- 未验证：
  - **未跑实车**。三项行为改动都只经过单元测试与离线冒烟：失败日志格式、`getSparseWaypoints`
    的降级展开、脱困前缀在真实 ROGMap/真实点云下的表现均未上车确认。
  - 本次仍**没有定论**「13:44 那次 76 次瞬时失败」到底死在 `buildSeed` 还是候选轨迹硬校验：
    实测 MINCO 单次优化耗时中位 0.251 ms（518 个样本取自 `.scratch/*.log`），与观测到的
    0.3 ms 间隔同量级，两条路径都放得下。① 的日志改造就是为下一次运行直接给出答案；
    在拿到该数据前，不应把 ② 当作现场主因。
  - 脱困前缀的安全边界只在离线场景验证：长度上限 = `collision_dist`、末速度为零、
    每点「不比当前净空更差」。它在连续多个周期里最多允许净空每次下滑 0.02 m（与既有近场规则
    同源），该「棘轮」效应未标定；若实车出现贴墙蠕动，先把 `stuck_escape.enable` 置回 `false`。
  - 清理 `install/` 残留属构建产物操作：下次 `colcon build` 若发现源码里仍缺这些包不会重建它们，
    但若有旧终端仍 source 过清理前的环境，需要新开终端。

## 2026-09-16 — ROGMap 动态障碍局部绕行与安全停车前缀

- 问题：全局 SMAC 运行在地形图与 `/dynamic_cost_map` 上，而当前 bringup 关闭了
  `map_server` 动态检测；实时动态障碍只存在于 ROGMap。全局折线因此可能穿过
  `/rog_map/occupied`。旧的局部处理仅用 `isFree()` 检查稀疏化捷径，不按
  `collision_dist` 检查净空，也不会搜索障碍左右的绕行拓扑；`getSparseWaypoints()`
  最后还会无条件补入终点。MINCO 从穿障种子开始只能依靠 ESDF 软罚项，硬校验失败后
  不发布轨迹，TaskManager 又会保留同一条全局路径反复重试，表现为有全局折线但底盘不动。
- `local_path_processor.{hpp,cpp}`：局部种子现在从机器人实测位置开始，按 ROGMap
  `isFree + ESDF > collision_dist` 与静态地形约束统一检查整段路径；保留已有的起点近场
  “不比当前净空更差”规则。路径被实时占据截断时，在 ROGMap 滚动窗口内运行 8 邻域 A*，
  禁止对角穿角，并把可行绕行路径重新接到原局部终点。若当前窗口被障碍完全封死，则沿
  原路径提取障碍前安全部分、额外回退 0.15 m，形成至少 0.30 m 的停车前缀。
- `minco_planner.cpp`：安全停车前缀即使不是任务终点，也以零末速度/零末加速度进入时间分配
  和 MINCO；候选轨迹硬校验失败不再把 `is_traj_safe_` 置假，避免失败候选污染仍可执行的
  已提交轨迹，已提交轨迹安全性继续只由 20 Hz 安全监视器决定。
- `minco_utils.cpp`：稀疏路径只有在最后一段通过同一安全回调时才加入终点，并在返回前复核
  每一条稀疏线段；无法修复的碰撞种子直接返回失败，不再无条件把终点接回障碍另一侧。
- 新增 `test_local_path_processor.cpp` 与 CTest 目标：覆盖“直线路径被墙截断但有宽缺口时必须
  绕行且所有稀疏段净空合格”和“整墙封死时必须生成障碍前零末速度停车前缀”两种回归场景。
  README 同步说明 ROGMap 滑窗内的动态绕行与安全停车行为；本次新增源码/测试注释均使用中文。
- 验证：`cmake --build build/mas2027_nav_executor -j2` 构建通过；
  `ctest --test-dir build/mas2027_nav_executor --output-on-failure` 共 9 项全部通过（原 8 项 +
  `local_path_processor`）。未验证：未在真实点云、真实移动障碍与底盘上运行；停车前缀固定
  回退量 0.15 m 和最小长度 0.30 m 仍需结合实车制动距离标定；全局 RViz 折线本身仍来自
  地形层，动态绕行发生在下游局部种子中，因此显示的全局折线不会随 ROGMap 障碍改道。

## 2026-09-15 — 新增导航调参与修复全过程文档

- 新增 `docs/nav_tuning_2026-09-15.md`：把当日从「车不动 / 一卡一卡」到「行驶流畅」、以及从「只能近处规划」到「可规划远处目标」的全部因果链与改动按**因果**重组（本文件按时间顺序、逐条含验证边界，两者互补）。
- 文档结构：① 卡顿/不动（近距盲区幻影障碍、速度相关净空的极限环、线速度死区、未观测语义、投影窗口与虚拟地面矛盾、动态障碍层、两次失败尝试、仍未处理的一处阈值不一致）；② 远处规划（`isFree` 把 255 当不可通行、Kino 无解降级、`allow_unknown` 不同源、降级改用地形图查询、先验图融合启用后关闭）；③ 与上游 `navi_minco_bit` / 旧工程 `mas_nav_2027` 的对齐结果（含**唯一剩下的实质差异：方向约束层**，以及两处有意保留的本工程取值）；④ 仍未解决/未验证清单；⑤ 按文件的改动清单。
- 仅新增文档，未改动任何代码、配置或参数。
- 验证：文档中每条结论的出处（现场日志文件名与关键行、源码文件与函数、实测数值）均在正文标注，数值取自当日实车日志与配置核对。**未验证**：文中"与上游对齐"的判断基于配置与源码的静态比对，未在 docker 内实跑对照；文档描述的是截至 2026-09-15 的状态，不随后续改动自动同步。

## 2026-09-15 — RViz 增加真正的全局折线显示（原来的 "Global Path" 发的其实是 MINCO 轨迹）

- 问题：RViz 里那个叫 `Global Path` 的显示挂在 `/nav_executor/global_path` 上，但该话题由
  `publish_trajectory_visualization()`（`nav_executor_node.cpp`）用 `trajectory.cmds` 填出来，
  发的是 **MINCO 轨迹**，不是全局搜索结果。真正的 SMAC 折线（`MincoPlanner::latest_global_path_`）
  从来没发布过 —— 也就是说「搜索给出的拓扑引导」在现场一直看不见，这是排障时最需要的一条信息。
- 修改（6 个文件）：
  1. `minco_planner.{hpp,cpp}`：新增 `copyLatestGlobalPath(std::vector<PoseStamped> &) const`，
     在 `path_mutex_` 下一次锁里连数据一起取出（避免调用方分两次加锁看到不同快照）。
  2. `path_planner.{hpp,cpp}`：`PathPlanner::copyLatestGlobalPath()` 透传到 `MincoPlanner`。
  3. `nav_executor_node.cpp`：新增 `/nav_executor/global_plan`（`nav_msgs/Path`）与
     `/nav_executor/debug/global_plan`（`visualization_msgs/Marker`，LINE_STRIP + 终点球）
     两个发布器，以及一个默认 5 Hz 的 `global_plan_timer_`。折线画在 z=0.03 抬高一点，
     避免与 z=0 的代价图/规划约束栅格闪面；线宽/颜色/频率来自 `node.visualization.global_plan_*`。
     没有可用折线时发一条空 Path 并 `DELETE` 两个 Marker，避免 RViz 上留着一条已经失效的旧线。
  4. `config/node_params.yaml`：新增两个话题名与四个样式参数（默认线宽 0.15 m、青色、5 Hz）。
  5. `nav_executor_view.rviz`：新增 `Global Plan (SMAC search, thick)` 与 `Global Plan (Path)`
     两项（放在显示列表最后 = 最后渲染 = 画在最上层）；把原来误名的 `Global Path` 改名成
     `MINCO Trajectory (Path)` 并调暗调细（0.6 alpha / 0.04 m），避免和真正的全局折线抢视线。
  6. `test/smoke_goal.py`：新增全局折线的端到端断言。
- 顺带修掉一个真实缺陷：Marker 发布器原本用 `rclcpp::QoS(1).transient_local()`。
  `transient_local` 的 durability 缓存只保留最后 depth 条，而折线与终点球是**两条独立消息**，
  depth=1 时**后打开 RViz 只能拿到终点球、看不到折线**。已改为 `QoS(10)`，并且 `publish_global_plan()`
  改为每个定时器周期都重发（而不是只在内容变化时发），这样 RViz 在本节点之后启动也能拿到完整折线。
- 验证：
  - `mas2027_nav_executor` Release 构建通过；执行器 8 项 CTest 全部通过。
  - `test/smoke_goal.py` 普通与 `--preempt` 两个变体均通过，新增断言覆盖：话题确实有消息、
    frame 为 `odom`、坐标全部有限、末点与精确目标重合（1e-6）、Marker 与 Path 点数一致、
    线宽 ≥ 0.10 m 且不透明、起点贴着机器人。
  - **判别性断言**：折线相邻点间距必须落在格点尺度（SMAC 直接在地图 0.05 m 格上扩展，
    故为 0.05 或 0.0707 m）。这条专门防「以后有人把这条线又接回 MINCO 轨迹」——
    轨迹按 dt 采样，间距小一个量级，一接错就立刻失败。实测普通版 52 轨迹点 / 48 折线点，
    preempt 版 54 / 56，两者点数不同也印证是两份数据。
  - 本次实测输出：`goal smoke passed: 52 trajectory poses, 48 global plan poses, marker width 0.150 m`。
  - RViz 配置用 `yaml.safe_load` 解析通过，四个相关显示项与 `Fixed Frame: map` 均正确。
- 未验证：
  - **未在真实 RViz 图形界面里看过**（沙箱内无法起 GUI）——线宽 0.15 m / 青色是否够醒目、
    0.03 m 抬高是否完全消除闪面，需要上车目视确认。
  - 未跑实车；`node.visualization.global_plan_publish_hz` 等参数只做了默认值路径的验证，
    没有做运行时 `ros2 param set` 实测。
  - 换目标瞬间会先发一条空 Path 去清 RViz，实车点击密集时是否会出现肉眼可见的一闪未评估。

## 2026-09-15 — 全局主搜索换成从 mas_nav_2027 移植的 SMAC 2D（含 ESDF 势场软代价）

- 决策：用户选定「只换搜索算法」——把 `mas_nav_2027` 的 `smac_search/` 移植进来替换现有 NavFn 式 `Astar` 主搜索，地图仍是本工程的地形 msgpack，其余链路不动；并选定「SMAC 失败直接失败并打诊断日志，不回退 A*」。
- 背景（上一条已详述）：上一条把主搜索从 Kino 换成了纯栅格 A\*，但那是 `Astar`（NavFn 波前），只是「与 SMAC2D 同口径」，并非同一个算法；旧工程实际跑的是自写简化版 SMAC 2D，且带 `smac_2d.use_esdf_cost` 的距离场软代价，本工程一直没有。
- 移植范围（**不是整包搬运**）：旧工程 `src/smac_search/` 共 1627 行，其中 `node_2d.cpp`（151 行）与 `collision_checker.cpp`（167 行）全仓库只有自己 include 自己，是死代码，未移植；实际搬入 `constants.hpp` / `types.hpp` / `smac_planner_2d_simple.{hpp,cpp}`，算法本体逐行保留（8 邻域扩展、octile 启发式、`tolerance` 到点判定、膨胀代价因子、ESDF 势场软代价、SoA 搜索缓冲）。
- 修改（7 个文件，新增 4 个）：
  1. 新增 `include/mas2027_nav_executor/path_planner/search/smac/{constants.hpp,types.hpp,smac_planner_2d_simple.hpp}` 与 `src/path_planner/search/smac/smac_planner_2d_simple.cpp`。相对旧工程的三处必要改动：去掉 `nav2_costmap_2d::Costmap2DROS` 成员与两个 `configure` 重载，改为 `configure(rclcpp::Logger)`（地图几何全部取自 `MapQueryInterface`，`createPath()` 本来每次都会刷新 origin/resolution/size）；代价常量改从本目录 `constants.hpp` 取（取值与 Nav2 相同 255/254/253/252）；`types.hpp` 删掉只服务 Nav2 平滑器的 `SmootherParams`（它带来 `rclcpp_lifecycle` 依赖）。命名空间为 `mas2027_nav_executor::smac`。
  2. `global_path_searcher.{hpp,cpp}`：`configure()` 增加 `smac`/`use_smac` 形参；`makePlanOnQuery()` 改为旧工程同构的 `if (use_smac_ && smac_) {...} else { ...Astar... }`，SMAC 分支失败即 `return false` 并打 `SMAC 2D failed to find path` + 端点诊断。`planner=%s` 的日志字段由硬编码 `"Astar"` 改为 `(use_smac_ && smac_) ? "SMAC2D" : "Astar"`（旧工程就是这么写的；不改会让现场排查误判实际用的是哪条搜索）。
  3. `minco_planner.{hpp,cpp}`：新增 `smac_planner_` 成员与 `use_smac_` / `smac_use_esdf_cost_` / `smac_esdf_weight_` / `smac_esdf_decay_` / `smac_esdf_max_cost_` 参数；构造点与 `setParameters(allow_unknown_, 1000000, tolerance_)` 取值对齐旧工程 `minco_planner.cpp:543-548`；`rebuildModeDependentQueries()` 与 `cleanup()` 同步。`onSetParameters`：`use_smac` 归入 configure-time（结构开关），`smac_2d.*` 四项支持在线调并即时下发到 `smac_planner_`。
  4. `config/planner_params.yaml`：新增 `use_smac: true` 与 `smac_2d.{use_esdf_cost,esdf_weight,esdf_decay,esdf_max_cost}`，取值与旧工程 `nav2_params.yaml` 的 `smac_2d` 段一致（true / 1.0 / 0.8 / 0.5）。
  5. `CMakeLists.txt`：planner 库加入新源文件；新增测试目标 `test_smac_planner_2d_simple`。
  6. 新增 `test/test_smac_planner_2d_simple.cpp`（4 项断言组）。
  7. `README.md`：在线链路与配置说明由「全向 Kino A\*」改为「SMAC 2D」，并补上 Kino 已移出关键路径、方向层当前全 0 的说明（上一条改了代码但没同步 README）。
- **顺带修掉一个移植过来的真实缺陷**：旧工程 `setMap()` 里有一句 `planning_id_ = 0u;`。旧工程只在 configure 时调一次 `setMap`，所以从未暴露；本工程沿用「每次搜索前把地图交给 SMAC」的接线方式，每次搜索都会调，于是归零后 `createPath()` 的 `++planning_id_` 又重新得到 1，与上一次搜索留在 `visited_/closed_/parent_` 里的 1 撞号。后果不是「搜索变慢」而是**静默返回陈旧路径**：`createPath` 末尾的失败判定是 `if (!goal_reached && closed_[goal_index] != planning_id_)`，goal 格带着上一轮的 id 使判定被跳过，于是直接顺着上一轮留下的 `parent_` 回退出上一条路径（实测 `iterations=1`、路径与上一轮逐点相同）。已改为 `planning_id_` 单调递增、`setESDFQuery` 变更时只失效 `esdf_cost_cache_id_`，并在代码注释里记下成因。
- 验证：
  - **回归测试确实是有效的（先证实再修）**：把 `planning_id_ = 0u;` 临时加回去重建，`test_smac_planner_2d_simple` 在第 224 行断言失败（`assert(!search(...))`），即「缺口封死后仍返回上一轮那条穿墙的陈旧路径」；恢复修复后通过。最初写的「两次搜索路径应逐点相同」版本抓不到该缺陷（陈旧路径恰好等于上一轮路径，比对通过），已改成「地图变化后必须基于新地图搜索或失败」。
  - `mas2027_nav_executor` Release 构建通过。
  - 执行器 8 项 CTest 全部通过（原 7 项 + 新增 `smac_planner_2d_simple`），`ROS_LOG_DIR` 指向工作区内。
  - **端到端离线冒烟**（`test/smoke_goal.py`，真实 `lab3_terrain.msgpack` + 真实 config，`ROS_DOMAIN_ID=232/230/229`）：日志出现 `SMAC 2D global search enabled: use_esdf_cost=true weight=1.000 decay=0.800 max_cost=0.500 tolerance=0.300` 与 `Terrain global search input: planner=SMAC2D`，`goal smoke passed: 52 trajectory poses`；`--preempt` 变体（连续两次全局搜索、复用同一个 SMAC 实例）`passed: 54 trajectory poses`；`use_smac: false` 的临时参数副本确认回退分支可用（日志 `SMAC 2D disabled (use_smac=false)`、`planner=Astar`、`passed: 52`）。
- 未验证：
  - **未跑实车**——SMAC+ESDF 偏置对实际路径形态、通行时间、窄道通过率的影响未实测；出厂值 1.0/0.8/0.5 是软偏置，单测里为了可观测用的是放大权重（100/0.8/无上限），两者不等价。
  - 旧工程的膨胀梯度（Nav2 `InflationLayer` 0.55/cost_scaling 2.5）**未移植**：本工程地形图经 `TerrainMapQuery` 折成二值 0/254，`evaluateInflationCost()` 对自由格恒返回 1.0，因此 `cost_penalty` 目前不起作用，真正改变路径的只有 ESDF 项。路径贴墙程度是否可接受需现场看。
  - 本工程每次全局搜索都会 `setMap()`（旧工程只在 configure 时调一次），虽然已按幂等实现，但高频调用下的开销未测。
  - `smac_2d.*` 的在线改参路径（`onSetParameters`）只做了代码与日志检查，未做 ros2 param set 实测。

## 2026-09-15 — 全局搜索改为「地形图纯栅格 A* 主搜索」（对齐 mas_nav_2027 的 SMAC2D），Kino 移出关键路径

- 决策：用户选定"把地形图纯栅格 A\* 提为主搜索，对齐 mas_nav_2027 的 SMAC"（上一条记录末尾留的"待用户决定"项即此项，现予落地；上一条的帧换算修复保留不动）。
- 背景（上一条已详述）：本工程 `planner_mode_context.cpp` 删掉 `PRIORMAP` 分支后强制 `EXPLORATION`，全局搜索换成速度格点 Kino（`searchOmniKinoPath`），纯栅格 A\* 只剩降级这一条路；实测 Kino 在真实 lab3 图上 3 m 起大量方向无解、5 m 后几乎全灭，每次失败烧光 50 000 次扩展预算、耗时 0.76~1.03 s。
- 修改（3 个文件）：
  1. `global_path_searcher.cpp`：`planExploration()` 的地形分支里，**主搜索**改为 `TerrainMapQuery`（= `terrain_->planningConstraints()`：静态地形 + 当前动态层）上的 `makePlanOnQuery()`，即纯栅格 A\*，与旧工程 SMAC2D 同口径；删掉 Kino 调用与其降级分支（降级逻辑已无意义——A\* 比 Kino 更宽松且完整），并删掉只服务于 Kino 的 `dynamic_free` 包装（含 ROGMap 滑窗判据；旧工程在 PRIORMAP 模式下同样不把 ROGMap 用于全局搜索）。`cancel_checker` 上提到函数开头，地形主搜索与无地形回退分支共用（此前降级调用传的是空 checker）。动态层就绪检查保留（`TerrainMapQuery` 只在 `dynamic_snapshot_` 存在时才并入动态障碍，故仍 fail closed）。**路径输出的 map→odom 换算保留**（上一条修的帧错配）。
  2. `global_path_searcher.hpp` / `minco_planner.cpp`：`plan()` 与 `planExploration()` 去掉已无消费者的 `start_velocity` / `max_speed` / `max_acceleration` 形参，调用点同步（`getCurrentSpeed()`、`minco_config.max_vel/max_acc` 不再传）；注释说明运动学输入已不需要。
- 未改动：`Astar`、`TerrainMapQuery`、`makePlanOnQuery`、Kino 模块本体与 `test_omni_kino_astar`（保留备用，现已无生产调用点）、ROGMap 与全部 `planner_params.yaml` 参数、无地形时的 ROGMap 回退分支。
- 验证：
  - **探针实测（`.scratch/kino_probe/probe_astar.cpp`，复用 `makePlanOnQuery` 的同一套 A\* 循环与端点投影，lab3.pgm + direction 全 0）**：对 Kino 失败的那批远处目标，A\* 全部给出路径且耗时 **1.0~13 ms**——d=5.0/45° → 120 点 4.1 ms、d=6.0/0° → 151 点 4.1 ms、d=7.0/0° → 151 点 4.2 ms、d=8.0/0° → 161 点 2.4 ms、d=9.0/0° → 181 点 1.9 ms（同批目标 Kino 分别耗时 0.76~1.03 s 后失败）。仍为 0 ms 即失败的样本是目标落在墙里或地图外（如 180° 方向超过 4 m 即出界），与搜索能力无关。
  - `mas2027_nav_executor` Release 构建通过（12.5 s）；执行器 7 项 CTest 全部通过（`ROS_LOG_DIR` 指向工作区内）。
- 未验证：**未跑实车**——远处目标能否稳定规划并真的开出去未实测；`Terrain` 主搜索在终端的新签名是 `[MincoPlanner] Terrain global search input: ...`（成功）与 `Terrain Astar failed to find path` + 端点诊断（失败），下次现场以它为准计数；A\* 路径不含速度/加速度可行性，是否会让 MINCO 的收敛率变化未评估；掉头/窄缝等 Kino 原本能给出运动学种子的场景是否退化未知（若退化，需把 Kino 作为"近处可选精修"重新接入）。

## 2026-09-15 — 降级路径补做 map→odom 换算（修「降级成功但车不动」）+ Kino 远处目标能力实测

- 现象（现场 20:50 运行 `mas2027_nav_executor_node_139128_1789476613275.log`，408 行）：点 4 个远处目标（odom 系 5.77/4.57/4.6x m）后，**每个目标都在约 1.1 s 后出现一条** `No acceleration-feasible route satisfies terrain and current dynamic obstacles; falling back to terrain-map A* direct plan`；降级 A\* 本身**成功**（全程没有 `Astar failed`、没有端点诊断），但紧接着是 **238 次 `MINCO path generation failed; retrying`**、车一动不动，`Braking: ...` 反复出现。
- 根因一（本次修复）：`makePlanOnQuery()` 的 `output_frame` 形参**只用于写 `header.frame_id`，不做任何坐标变换**：返回点位来自 `query->mapToWorld()`，即**地形图查询坐标系（map）**。上一条改动把降级查询从 ROGMap 换成 `TerrainMapQuery`、起终点改用 `start_map/goal_map` 时，仍把 `mode_context.outputFrame()`（= odom）当输出帧传了进去，于是种子路径**整体被贴上 odom 标签却是 map 坐标**。现场两者并不重合：`odom_localizer` 日志 `map->odom T=(0.316, 0.403) Q≈yaw 0.10 rad`，车停在 odom 原点时 map 系位置是 `(0.309, 0.391)`（即上面日志里 `start_world` 那一对）——种子路径起点离车 **0.5 m**、终点离用户点的目标 **0.5 m**，还多转 5.8°。MINCO 只能沿这条错位参考线优化，起点附近净空 0.24~0.29 m 反复低于 0.30 m 被判 `COLLISION`。
- 修改（1 个文件，`global_path_searcher.cpp`）：降级 `makePlanOnQuery` 成功后，逐点做 `tf2::doTransform(in, out, map_to_rog)` 把路径由地形图系换算到 `outputFrame()`——用的是**同一个 `map_to_rog`**、与紧随其后的 Kino 成功分支完全一致；注释记录成因与现场数字。未改 `makePlanOnQuery` 本体、未改任何参数。
- 根因二（本次只实测、未改代码）：**`searchOmniKinoPath` 在真实 lab3 图上够不到远处目标**。新写的独立探针（`.scratch/kino_probe/`，直接编译 `terrain_grid.cpp`+`omni_kino_astar.cpp`，用 lab3.pgm 按 map_server 同款翻转/阈值建图，direction 全 0 与 `pgm_to_terrain_msgpack.py` 一致）实测：**空图**上 1~8 m 全部有解、最慢 85 ms；**真实 lab3 图**上以车当前位置为起点，3 m 起大量方向无解、5 m 后几乎全灭，且每次失败都烧掉整个 50 000 次扩展预算、耗时 **0.76~1.03 s**（与现场日志里"目标→WARN 相隔 1.08 s"吻合）。即瓶颈是**带速度维的状态格点在有障碍空间里爆炸**（`speed_bins = max_speed/0.1 = 30`、8 个朝向 → 单节点最多 248 个后继），不是解不存在。
- 对照：旧工程 `mas_nav_2027` 的全局搜索是 **SMAC 2D 纯栅格**（`PlannerMode::PRIORMAP`，跑在整张先验图的 Nav2 costmap 上，`smac_planner_2d_simple`），不含速度/加速度可行性约束、也没有扩展预算，故"远处目标"从不会因此失败；本工程 `planner_mode_context.cpp` 已把 `PRIORMAP` 分支删掉、强制 `EXPLORATION`，全局搜索换成了速度格点 Kino，降级 A\* 反而成了唯一能覆盖远处的通路。**是否把纯栅格 A\* 提为主搜索（对齐旧工程）待用户决定，本次未动。**
- 验证：`mas2027_nav_executor` Release 构建通过（26.9 s）；执行器 7 项 CTest 全部通过（`ROS_LOG_DIR` 指向工作区内，因沙箱下 `~/.ros/log` 只读）；探针实测数据如上。
- 未验证：**未跑实车**——帧换算是否消除"降级成功但车不动"未实测；Kino 远处无解因此是否仍要每次白烧 1 s 未处理；日志里 `Near-field exemption: start clearance 0.253 m below required 0.300 m` 说明车当时本就停在离障碍物约 0.25 m 处，该因素是独立叠加项，未评估。

## 2026-09-15 — 降级全局搜索改用地形图查询（方案 A，消除占据类 TERRAIN 违规）

- 背景：Kino 无解时的降级 A\* 原先在 **ROGMAP** 查询上做纯栅格搜索，不看 HW 地形/方向层，于是 MINCO 沿该种子路径优化出的轨迹被 `validateTrajectory` 判 `TERRAIN_COLLISION_OR_DIRECTION`（现场实测一次运行 10 次），轨迹发不出去。
- 修改（2 个文件）：
  1. `global_path_searcher.cpp`：降级分支改为构造 `mas2027_nav_executor::TerrainMapQuery(terrain_)` 作为查询传给 `makePlanOnQuery`，**起终点改用已算好的 map 系 `start_map.pose` / `goal_map.pose`**（地形查询的坐标系是 map，不能再用 ROGMap 系的 `start_rog`/`goal_rog`）；日志前缀由 `ROGMap(fallback)` 改为 `Terrain(fallback)`。同时新增 `#include "mas2027_nav_executor/common/environment/terrain_map_query.hpp"`。
  2. `CMakeLists.txt`：把 `src/common/environment/terrain_map_query.cpp` 加入 `${PROJECT_NAME}_node` 目标——该实现此前只出现在 `test_terrain_map_query` 中，不加会报 `undefined reference to TerrainMapQuery::TerrainMapQuery / vtable`（首次构建即踩到）。
- 已知限制（写在代码注释里）：`TerrainMapQuery` 的取值是**二值**的（`kLethalCost` 254 / `kFreeCost` 0），**不含方向信息**；而方向约束是逐边判定的（Kino 用 `transition(from,to)`，朴素 A\* 没有该概念）。因此本次只预期消除"穿过不可通行格"一类违规，**"逆向穿过方向受限格"仍可能被否**。
- 未改动：Kino 搜索、`makePlanOnQuery` 本体、`Astar`、ROGMap 与 planner 的全部参数。
- 验证：`mas2027_nav_executor` Release 构建通过（7.66 s）；执行器 7 项 CTest 全部通过。
- 未验证：未跑实车——`TERRAIN_COLLISION_OR_DIRECTION` 下降多少未实测；**`TerrainMapQuery::refresh()` 为私有、本次未调用**，该查询是否在降级时刻已由构造/惰性路径填好快照未确认（若为空则 A\* 失败、降级返回 false，行为退回本次改动前）；方向类违规的残留量未知（需现场计数后决定走"边感知方向"还是放开方向约束）。

## 2026-09-15 — 回退 safe_dist 至 0.40：0.50 使 MINCO 不收敛

- 现象（现场运行时日志 `mas2027_nav_executor_node_122922_*.log`，141 行）：单击一个目标点后，全局路径**已正常生成**（仅 1 次降级调用，日志含 `allow_unknown=true` 且无 `Astar failed`，说明此前的 `isFree`/`allow_unknown` 修复生效），但随后出现 **93 次 `MINCO path generation failed; retrying`**、15 次 `MINCO trajectory not published: COLLISION`、17 次净空不足（如 `0.282 / 0.255 m below required 0.300 m at (1.17,-0.07) / (1.32,-0.06)`）、8 次 `TERRAIN_COLLISION_OR_DIRECTION`。即瓶颈从"全局搜索"转移到了 **MINCO 优化本身不收敛**。
- 判断：降级 A\* 给出的种子路径要穿过未观测区（该次日志 `goal_cost=255(unknown)`），此类区域几何通常很紧；而此前为把"软目标 vs 硬判据"交叉点推到 1.33 m/s 将 `safe_dist` 由 0.40 抬到 **0.50**，在窄处/未知区成为**够不到的软目标**，位置罚项恒不满足 → L-BFGS 不收敛 → `path generation failed`。硬判据只需 `collision_dist`(0.30)，故回退。
- 修改：`planner_params.yaml` 中 `minco_optimizer.safe_dist` 由 0.50 改回 **0.40**，注释记录抬高的动机、实测代价与回退理由。配置改动，无需重新编译。
- 未改动：`collision_dist`(0.30)、`replan_react_time`(0.0)、`node.rog_map_clearance`、全部 rog_map 参数、发布前校验（仍为已回退的 `requiredClearance(v)`）。
- 验证：`yaml.safe_load` 解析通过并打印确认（`safe_dist = 0.4`、`collision_dist = 0.3`、`replan_react_time = 0.0`）。**未跑实车**：93 次优化失败是否由此消除未实测。
- 未验证/遗留：8 次 `TERRAIN_COLLISION_OR_DIRECTION` 与 15~17 次净空不足仍存在——前者是 HW 地形/方向图否决降级 A\* 给出的纯栅格路径（该路线已知不保证方向可行性），后者是"监视器减容差、发布前校验不减"的不一致（该不一致仍成立但已回退，需在干净现场数据下再评估）。`safe_dist` 是否应介于 0.40~0.45 之间未调参。

## 2026-09-15 — 回退「发布前校验补容差」（按用户要求）

- 回退内容：`minco_planner.cpp` 的 `checkCollision(const traj_opt::Trajectory &)` 中 `options.check_dist` 恢复为 `requiredClearance(v)`（即上一条记录改回原样），并在注释中保留该次尝试的结论与"监视器与本处阈值不一致"这一仍成立的事实。
- 回退理由：改动后 `validateTrajectory: collision detected` 确实不再出现（改动生效），但随后现场报告"完全无法规划"。核对日志（`mas2027_nav_executor_node_114441_*.log`，161 行）显示瓶颈在更上游：**31 次 `Global path search failed` + 33 次 Kino 降级尝试 → 无全局路径 → MINCO 无种子 → 46 次 `MINCO path generation failed`**，与本处校验阈值无因果关系。同时发现 7 次目标点中有 **3 次被 `Ignoring goal: odometry is stale` 直接拒收**（`/Odometry` 过期，属独立问题）。
- 未改动：`kMonitorClearanceTolerance` 常量、`collision_dist`、`replan_react_time`（0.0）、`node.rog_map_clearance`、全部 rog_map 参数。
- 验证：`mas2027_nav_executor` Release 构建通过（26.9 s）；执行器 7 项 CTest 全部通过；回退后用 grep 确认第 1683 行已恢复为 `options.check_dist = requiredClearance(v);`（第 1652 行的监视器分支仍为 `monitor_dist`，即减容差的版本，未受影响）。
- 未验证：未跑实车确认"无法规划"是否随之消失——按日志判断该现象由全局搜索失败与 odom 卡顿造成，**回退本身预期不会解决它**；`Global path search failed` 与 `odometry is stale` 两条线都尚未定位。

## 2026-09-15 — 发布前校验补上净空容差，修复"过某个特定点必卡"

- 现象：同一条路径经过某个特定点必然卡住。日志（`mas2027_nav_executor_node_97120_*.log`，315 行）中失败样本沿一条直线排开，净空为 **0.300 / 0.299 / 0.298 / 0.297 / 0.296 m，而要求恰为 0.300 m** —— 即差 **0~4 毫米**；终端同时出现 `validateTrajectory: collision detected.` + `Trajectory validation failed! Rejecting`。
- 根因：同一份代码里发布前校验与运行时监视的阈值**不一致**。运行时监视（`minco_planner.cpp:1643`）用 `requiredClearance(v) - kMonitorClearanceTolerance`；发布前校验（`checkCollision(const Trajectory&)`，原第 1676 行）用**不减容差**的严格 `requiredClearance(v)`。而该函数正上方的注释写的是"发布前校验：阈值必须与运行时监视一致"——**代码违背了自己的注释**。于是净空停在 0.296~0.300 的轨迹：监视器放行（阈值 0.25），发布前校验毫米级否决，新轨迹发不出去；旧轨迹走完，车就停在原地。
- 修改：`checkCollision(const traj_opt::Trajectory & traj)` 中 `options.check_dist` 改为 `std::max(0.0, requiredClearance(v) - kMonitorClearanceTolerance)`，与监视器对齐；注释补充该不一致的成因与现场证据。
- 未改动：`kMonitorClearanceTolerance` 取值、`collision_dist`、`replan_react_time`（仍为 0.0）、`node.rog_map_clearance`、`safe_dist`、全部 rog_map 参数。
- 验证：`mas2027_nav_executor` Release 构建通过（30.1 s）；执行器 7 项 CTest 全部通过。
- 未验证：未跑实车确认该点不再卡；**该点净空本就只有约 0.296~0.300 m**（释放容差后为 0.25 阈值，理论可通过），但该处是否仍有真实窄点导致速度受限未评估；容差 0.05 是否过大（相当于把有效净空降到 0.25，与本车内切半径 0.30 的关系）未复核。

## 2026-09-15 — 关闭先验图融合：实测未对齐，会在返程制造幻影墙

- 现象：开启先验图融合（lab3）后，返程（第二个目标）出现一次明显卡顿。时序证据（`mas2027_nav_executor_node_92610_*.log`，49 行）：去程 +0→+19 s 到达、无任何告警；返程 +24 s 出发后，在 **(3.16~3.31, −0.02~−0.04)** 一带出现 4 次 `Trajectory clearance 0.272~0.284 m below required 0.275~0.300 m` 与 **10 次** `Braking: current dynamic obstacle intersects the MPC reference horizon`，返程 26 秒中有约 20 秒耗在该点。
- 根因：`fusePriorMapProjection()` 把先验图的占据格当**硬障碍**叠加进二维投影/mask。RViz 确认该处 `/rog_map/layer_value_static` 显示有墙、而 `/rog_map/layer_value_dynamic`（实测）为空——即**先验图与现场未对齐**，净空由约 0.4 m 被压到 0.272~0.285 m，低于 `collision_dist` 0.30，于是规划器否决、执行器急停。
- 另注（排查误导点）：`command_safety.cpp:56-64` 中「动态图不自由」与「ROGMAP 净空不足」**共用同一个 `DYNAMIC_BLOCKED` 状态**，调用方统一打印 `Braking: current dynamic obstacle ...`。本次靠坐标与净空告警交叉比对才区分开——该层已由 `bypass_dynamic_obstacle: True` 关闭，并非复活。
- 修改：`planner_params.yaml` 的 `rog_map.projection.prior_map.enable` 由 `true` 改回 **`false`**，并按第 1 步"只验对齐、对不上不继续"的约定暂停后续放大窗口与切换全局搜索。`yaml_path`/`pgm_path`/`frame_id` 原样保留，便于对齐修好后一键恢复。配置改动，无需重新编译。
- 未改动：`map_size`、全部 `minco_optimizer` 参数、`node.rog_map_clearance`、`collision_dist` 均未动（避免又是一次口径打架）。
- 验证：`yaml.safe_load` 解析通过并打印确认 `prior_map.enable = False`。
- 未验证：未跑实车确认返程卡顿消失；**先验图错位的成因未查明**（可能方向：`prior_map.cpp` 的 `loadPriorMap` 对 PGM 的 y 翻转/origin 处理，或 `map`↔`odom` 标定，或 lab3.pgm 与 lab3.pcd 本身不同源）；对齐修好前，先验图融合与"全局搜索改用 A*"的路线保持暂停。

## 2026-09-15 — 启用 ROGMap 先验图融合（lab3），为全局搜索改用 A* 做准备

- 背景与目标：远处目标规划一直受"全局搜索跑在 HW 地形图上、且受 ROGMap 10×10 m 滑窗牵制"的限制；docker 内的上游（`/home/mas/nav_opensource/navi_minco_bit`）是把整张先验图交给 Nav2 StaticLayer、由 SMAC 在**整张图**上规划，`rog_map.projection.prior_map` 只在滑窗内给 MINCO 当 ESDF。本工程已移除 Nav2，故按路线 A 对齐：**先验图融合进 ROGMap，再由 A\* 规划**。本次只做第 1 步。
- 上游配置对照（`src/navigation/navi2_bringup/params/sentry1.yaml:393-398`）：`prior_map: {enable, yaml_path, pgm_path, frame_id}` 四个键与 `rog_map_core/config.hpp:287-290` 读取的 `projection.prior_map.*` **完全一致**。
- 改动：`planner_params.yaml` 的 `planner.rog_map.projection` 下新增 `prior_map` 段——`enable: true`、`yaml_path` 指向 `mas2027_nav_bringup/map/lab3.yaml`（origin 为 `[-4.6, -7.94, 0]`，resolution 0.05）、`pgm_path: ""`（留空即用 YAML 的 `image` 字段并相对 YAML 目录解析）、`frame_id: map`。
- **本步刻意不动 `map_size`**：保持 10×10 m 窗口，先验证对齐。理由：先验图与现场若对不上，后面放大窗口与切换搜索都是白做；`map`↔`odom` 由 `odom_localizer` + `tf_maintainer` 提供。
- 未改动：`map_size`、`ray_range`、两个 `unknown_as_occupied`、`scan_z_min_abs`、`virtual_ground_height`、`minco_optimizer` 全部参数；无代码改动。
- 验证：`yaml.safe_load` 解析通过并打印确认该段取值；隔离 `ROS_DOMAIN_ID=230` 直启 `mas2027_nav_executor_node`（补 `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu`）9 秒，日志出现 **`[ROGMap] loaded prior map 770x347 at 0.050 m/cell in frame 'map'`**（770×347 即 lab3.pgm 的实际尺寸），`[MincoPlanner]` 正常初始化、无异常退出。
- 未验证：**未验证先验图与现场是否对齐**（需真机 RViz 看 `/rog_map/layer_value_static` 与实测墙体是否吻合）——这是本步的核心判据，也是后续放大窗口与切换全局搜索的前提；未跑实车。

## 2026-09-15 — allow_unknown 与 exploration.unknown_as_occupied 同源，修复降级 A* 仍失败

- 现象（终端/日志 `mas2027_nav_executor_node_76587_*.log`）：Kino 无解后降级已被触发（出现 `falling back to ROGMap A* direct plan`），但**降级也失败**——`ROGMap(fallback) Astar failed to find path`，端点诊断显示 `start(used) cell=(100,99) cost=0(free)`、**`goal cell=(181,147) cost=255(unknown)`**。
- 根因：两个"未知格是否可通行"标志来源不同。外层判定用 `smacTraversableCost()` / `goal_traversable`，取自 mode context 的 `exploration.unknown_as_occupied`（本轮已改为 `false`，故放行了降级）；而 `Astar` 内部只在 `allow_unknown` 为真时才接受 `cost == 255`（`astar.cpp:196,259`），该值由 `GlobalPathSearcher::configure()`（`global_path_searcher.cpp:174`）一次性写入，调用点在 `minco_planner.cpp:479` 传的是 **MincoPlanner 自己的 `allow_unknown_` 成员**，与 `exploration_unknown_as_occupied_` 无关联。于是"外面说未知可通行、里面仍把未知当障碍"，目标落在未观测区域时必然搜不到路径。
- 修改：`minco_planner.cpp` 的 `global_path_searcher_->configure(...)` 第三个实参由 `allow_unknown_` 改为 `!exploration_unknown_as_occupied_`，使两者**同源**，不再可能各说各话。未改动 `Astar`、`GlobalPathSearcher` 与 `MincoPlanner::allow_unknown_` 成员本身（后者保留但不再影响该处）。
- 未改动：Kino 搜索、降级逻辑、地形/方向判定、全部参数。
- 验证：`mas2027_nav_executor` Release 构建通过（1 min 8 s）；执行器 7 项 CTest 全部通过。
- 未验证：未跑实车——远处目标能否由此成功规划未实测；`MincoPlanner::allow_unknown_` 若另有其他用途（本次未追查其赋值来源），改动后两者的语义差异未评估；降级路径给出的纯栅格路径能否被下游 MINCO/执行器正常接受仍未验证。

## 2026-09-15 — 远处目标全局规划失败：Kino 无解时降级到 ROGMap A* 直连规划

- 现象：在先验图上设置较远目标点后全局规划持续失败（`mas2027_nav_executor_node_63143_*.log`：目标 `(4.43, 2.29)`，38 次 `No acceleration-feasible route satisfies terrain and current dynamic obstacles` + 38 次 `Global path search failed; retrying`），而同一位置在 docker 内的上游/旧工程可以规划并执行到先验图最远处。现场在 RViz 确认 `HW Cost Map` 显示可通过、`HW Direction Map` 无异常，故排除地形层与方向层。
- 根因：本工程自研的全向 Kino A*（`searchOmniKinoPath`，`global_path_searcher.cpp:304`）以「实测里程计速度种子 + 速度/加速度可行性」扩展状态格点，**可行解集依赖车当前状态**，目标较远或朝向不巧时可能无解；而失败分支只是 `return false`，**不会走到紧随其后、本可用于兜底的 ROGMap A\* 直连规划**（同文件 353-371 行，仅对"目标在 ROGMap 内且可通行"生效）。上游用的是 Nav2 SMAC（对栅格搜索、不含速度可行性硬约束），因此不存在这种"看状态抽风"的行为。
- 修改：`planExploration()` 中 Kino 失败分支不再直接返回，改为打印 `falling back to ROGMap A* direct plan` 后，按与下方 `goal_traversable` 分支相同的判据（目标在 ROGMap 内、且 `value(kUnknownCost)` 时按 `explorationUnknownAsOccupied()` 取反、否则 `< kInscribedCost`）调用现成的 `makePlanOnQuery(start_rog, goal_rog, query, ...)`；成功则 `latest_global_path` 已由其填充，直接 `return true` 跳过基于 `map_path` 的填充逻辑，失败才 `return false`。降级调用传入空的 `cancel_checker`（该处 try 作用域内取不到外层 checker；`makePlanOnQuery` 内部先判空，A* 另有自身循环预算）。
- 未改动：Kino 搜索本身、地形/方向层判据、`isFree`、全部参数。
- 验证：`mas2027_nav_executor` Release 构建通过（10.1 s）；执行器 7 项 CTest 全部通过。
- 未验证：未跑实车——远处目标是否因此可规划未实测；降级路径给出的是**纯栅格路径**（不含速度/加速度可行性），是否会被下游 MINCO/执行器接受、以及 `latest_global_path` 与调用方 `minco_planner.cpp:1318` 的期望是否完全一致，均未验证；日志中新增的 `falling back to ROGMap A* direct plan` 可用于确认降级是否被触发。

## 2026-09-15 — 修复 isFree 把「无信息」当「不可通行」导致远处目标全局规划失败

- 现象：在先验图上设置远处目标点后全局规划持续失败。日志（`mas2027_nav_executor_node_58693_*.log`，243 行）中目标为 `(4.22, 1.59)`（落在 10×10 m 滑窗内），随后 **118 次** `No acceleration-feasible route satisfies terrain and current dynamic obstacles` + 118 次 `Global path search failed; retrying`，车不动。
- 根因：`global_path_searcher.cpp:295-301` 的 ROGMap 判据在窗口内要求 `query->isFree(mx,my)`；而 `QueryAdapter::isFree()` 原实现是 `values[idx] < 253U`。按 `projection_layer.cpp::applyValueAndMask()`，`unknown_as_occupied: false` 时**未知列的取值是 255**（no information），`255 < 253` 为假，于是**未观测区域整片被判为不可通行**。远处目标必经的正是车还没看过的区域，搜索状态空间被堵死。255（无信息）与 254（`kLethalCost`/OCCUPIED）语义不同，必须区别对待。
- 修改：`query_adapter.cpp::isFree()` 改为 `cost < 253U || cost == 255U`。该改在两种配置下都自洽：`unknown_as_occupied: true` 时未知列取 254，仍判为不可通行；`false` 时取 255，判为可通行，与二维 ESDF 的 `mask=1` 以及其他消费方（`trajectory_safety_checker` 只拒绝 253/254）保持一致。同时受益的还有 `path_planner.cpp:123` 的目标准入与 `local_path_processor.cpp:26` 的种子路径裁剪——此前它们同样把未观测格当"不自由"。
- 未改动：ROGMap 与 planner 的全部参数；本次仅一处判据，无配置改动。
- 验证：`rog_map`(5.8 s) 与 `mas2027_nav_executor`(6.6 s) Release 构建通过；执行器 7 项 CTest 全部通过。
- 未验证：未跑实车——远处目标能否规划成功未实测；**未评估新放行的"未知区域可通行"带来的风险**（规划可能穿过尚未观测的区域，与 `unknown_as_occupied: false` 的既有取向一致，靠地形先验图兜底）。

## 2026-09-15 — 对齐上游：取消速度相关净空要求（消除「一卡一卡」的极限环）

- 背景：实车仍「一卡一卡」。用户指出上游（docker 内的 `/home/mas/nav_opensource/navi_minco_bit`）用 `raycasting.ray_range: [0.03, 10.0]` 也完全正常，促使改为以上游为参照逐项核对。
- **决定性发现**：在上游源码中检索 `replan_react_time`、`monitor_margin`、`requiredClearance` **全部不存在**（`grep -rn ... src/` 无结果），其碰撞校验（`minco_planner.cpp:1567 validateTrajectory` / `1638,1662 checkCollision`）使用**固定阈值** `collision_dist: 0.25`。而本工程在发布前校验与运行时监视中都引入了 `required = collision_dist + max(v * replan_react_time, monitor_margin)`，使净空要求**随速度增长**，等价于速度上限 `v < (可用净空 - collision_dist) / replan_react_time`；规划速度略超该上限时，轨迹会在最窄处差几毫米被否掉，形成「加速 → 否决 → 急停 → 再加速」的极限环。实车日志证据：19:37 次运行 6 次否决全部为毫米级（0.414 vs 0.425、0.395 vs 0.403、0.314 vs 0.340、0.342 vs 0.345、0.414 vs 0.414、0.336 vs 0.338），且 6 个失败点与 6 次起点净空落在同一区间（0.30~0.42），与位置无关。
- 改动一：`planner_params.yaml` 中 `minco_optimizer.replan_react_time` 由 0.15 改为 **0.0**。此时 `requiredClearance()` 退化为固定 `collision_dist`（0.30），与上游的固定阈值行为一致，极限环不再成立。
- 改动二（修正前一次改动引入的矛盾）：`rog_map.virtual_ground_height` 由 −1.0 改为 **−1.5**。`prob_map.cpp` 的 `getGridType()` 将 `z <= virtual_ground_height` 的体素一律判为 OCCUPIED，而 `projection.scan_z_min_abs` 为 −1.2，窗口探到地面以下 0.2 m，每个柱子凭空多出 4 个体素的虚假占据，污染分类统计。旧工程取 −1.5 正是为了让 −1.2 的窗口落在其上。
- 未改动：`collision_dist`（保持 0.30，本车内切半径标定值；上游为 0.25，属另一台车）、`monitor_margin`（0.0）、`safe_dist`（0.50）、`speed_aware_clearance`（false）、`ray_range`（[0.3, 10.0]）、两个 `unknown_as_occupied`（false）均未动；本次为配置改动，无需重新编译。
- 验证：`yaml.safe_load` 解析通过并打印确认（`replan_react_time = 0.0`、`collision_dist = 0.3`、`virtual_ground_height = -1.5` 且 `-1.2 > -1.5` 成立）。
- 未验证：未跑实车——卡顿是否消除、速度是否进一步提升均未实测；**代价是高速下不再预留制动距离**（安全余量退化为固定 0.30 m，靠 20 Hz 重规划兜底），若实车出现高速制动不及，应从 0.05 起逐步回调 `replan_react_time`。

## 2026-09-15 — 关闭 map_server 动态障碍层（对齐旧工程架构）

- 背景：实车速度已恢复，但「明显可以通过的地方仍不丝滑」。逐项对比发现**结构性差异**：旧工程 `/home/mas/mas_nav_2027/mas2027_perception/` 下只有 `Localization`、`mid360_driver`、`Odometry`、`rog_map` —— **没有 `map_server`，没有动态障碍层**；本工程多出一整套「点云对比全局静态云 → 检测动态障碍 → 按 `full_cost_radius_m 0.2` / `cutoff_radius_m 0.4` 膨胀 → 发布 `/dynamic_cost_map`」，且执行器据它刹车（`command_safety.cpp` 中 `dynamic->freeAt()` → `ExecutorStatus::DYNAMIC_BLOCKED`，日志表现为 `Braking: current dynamic obstacle intersects the MPC reference horizon`）。现场 RViz 观察：该层明显比实物厚，把路径显示得比实际窄。
- 改动：`mas2027_nav_bringup/launch/nav_executor_launch.py` 中 `terrain_map_server` 的 `bypass_dynamic_obstacle` 由 `False` 改为 **`True`**。按 `map_server_node.cpp:67,88`，该开关为真时不加载全局静态云与 KdTree、**不订阅点云**，`/dynamic_cost_map` 保持全空；`/cost_map` 与 `/direction_map` 照常发布，规划器使用的地形图不受影响。
- 安全性取舍：动态物体改由 ROGMap 的时间衰减（`decay.keep_time 0.8 s` / `clear_time 1.2 s`）承担，即旧工程既有做法；代价是失去「点云与全局静态云比对」这一层显式动态检测，快速移动物体不再有独立的急停判据。需要动态避障时把该参数改回 `False` 即可。
- 未改动：ROGMap 的全部参数、`minco_optimizer` 全部参数（`safe_dist 0.50`、`replan_react_time 0.15`、`speed_aware_clearance false` 等）均未动；本次为 launch 文件改动，无需重新编译。
- 验证：`python3 -m py_compile` 解析通过；确认文件中该参数当前为 `True`（第 119 行）。**未验证**：未跑实车——`Braking: current dynamic obstacle ...` 是否消失、卡顿是否改善均未实测；若卡顿依旧，说明主因是「规划速度超过净空允许值」形成的极限环（净空 0.396 vs 要求 0.406 一类毫米级否决），下一步应改为按沿途净空给规划速度加上界，而不是继续动地图层。

## 2026-09-15 — 抬高优化器软目标 safe_dist 至 0.50，把硬判据交叉点推到 1.33 m/s

- 目的：在不引入"降速收益"的前提下消除窄道「卡一下」。卡顿根源是优化器软目标固定 0.40、而硬判据 `collision_dist + v * replan_react_time = 0.30 + 0.15v` 在 `v > 0.67 m/s` 时反超软目标；优化器只能渐近逼近软目标，于是窄道处稳定地差几毫米被 `validateTrajectory` 否掉（实测净空 0.396 vs 要求 0.406、0.425 vs 0.427）。
- 改动：`planner_params.yaml` 中 `minco_optimizer.safe_dist` 由 0.40 改为 **0.50**。交叉点随之推到 `(0.50-0.30)/0.15 ≈ 1.33 m/s`，覆盖当前实测约 0.8 m/s 的巡航速度。
- 影响面核查（改动前逐条确认，避免重蹈 `speed_aware_clearance` 的覆辙）：
  - `minco_planner.cpp:488` 为 `safety_checker_->configure(collision_dist, ...)`，检查器阈值取自 `collision_dist` 而非 `safe_dist`，**所有硬判据不受影响**；
  - `minco_planner.cpp:289` 的 `double collision_dist = minco_config.safe_dist` 只是参数缺省值，YAML 已显式给出 `collision_dist: 0.30`，同样不受影响；
  - `magnitudeBounds(0)` 在 `minco_optimizer.cpp` 中仅用于位置罚项（`safe_dist = magnitudeBounds[0]`），未参与速度/加速度约束与时间屏障。
  - 结论：该改动**只改优化器的软惩罚目标**，不含速度项，因此不产生「降速收益」，不会像上一条记录的 `speed_aware_clearance` 那样拖慢全局。
- 未改动：`collision_dist`、`replan_react_time`、`speed_aware_clearance`（仍为 `false`）、`clearance_optimizer_margin`、`ray_range`、两个 `unknown_as_occupied`、`scan_z_min_abs` 均未动；本次不涉及代码，无需重新编译。
- 验证：`yaml.safe_load` 解析通过并打印确认（`safe_dist = 0.5`、`collision_dist = 0.3`、`replan_react_time = 0.15`、`speed_aware_clearance = False`，交叉点计算为 1.333 m/s）。
- 未验证：未跑实车——窄道是否不再卡、窄缝中优化器追不到 0.50 目标是否导致 `MINCO path generation failed` 增多，均未实测；若出现后者，应把 `safe_dist` 回调到 0.45 再试。

## 2026-09-15 — 回退速度感知净空的启用（全程变慢），保留代码与参数

- 现象：上一条记录启用 `speed_aware_clearance`（含 `clearance_optimizer_margin: 0.05`）后，实车**全程速度都变慢**，不只是窄道。
- 原因（两条叠加，均已写入配置注释）：
  1. 目标净空由固定 `safe_dist = 0.40` 变为 `0.35 + 0.15v`，在 `v > 0.33 m/s` 之后**一律高于原目标**（v=0.5 → 0.425、v=1.0 → 0.50），整体净空要求被抬高；
  2. `penalty_weight_pos = 50000` 对 `penalty_weight_time = 100`（相差 500 倍），**减速几乎不花代价**，优化器因而选择「降速」而不是「绕开」来降低位置惩罚，等效于全局刹车。这是权重比例的结构性问题，不是余量取值能调好的。
- 修改：`planner_params.yaml` 中 `speed_aware_clearance` 由 `true` 改回 **`false`**，恢复固定 `safe_dist` 的原有行为；注释完整记录上述成因与重新启用前必须先提高 `penalty_weight_time` 的前提。`clearance_optimizer_margin`（0.05）与上一条记录新增的代码路径**保留不动**，仅在开关开启时生效，关闭时不进入任何计算。
- 未改动：`safe_dist`、`collision_dist`、`replan_react_time`、`ray_range`、两个 `unknown_as_occupied`、`scan_z_min_abs` 均未动；本次不涉及代码改动，无需重新编译。
- 验证：`yaml.safe_load` 解析通过并逐项打印确认（`speed_aware_clearance = False`、`safe_dist = 0.4`、`collision_dist = 0.3`、`replan_react_time = 0.15`、`penalty_weight_time = 100.0`、`penalty_weight_pos = 50000.0`）。
- 未验证：未跑实车确认速度已恢复；「窄道卡一下」在该配置下仍会存在（优化器固定目标 0.40 与硬判据 0.30+0.15v 在 v > 0.67 m/s 处交叉的问题未解决），是否需要提高 `safe_dist` 把交叉点推高，待实测后决定。

## 2026-09-15 — 修复窄道「卡一下」：开启速度感知净空并给优化器软目标加余量

- 现象：路一变窄就卡一下，但该处明显够宽、本可快速通过。19:12 次运行（`mas2027_nav_executor_node_17539_1789470653217.log`）只剩 2 次急停，两个失败样本都是**毫米级擦边**：净空 0.396 vs 要求 0.406（反推 v≈0.71 m/s，位置 (1.44,-0.04)）、净空 0.425 vs 要求 0.427（v≈0.85 m/s，位置 (0.53,-0.02)）。
- 成因：优化器位置罚项用**固定**软目标 `safe_dist = 0.40`，检查器硬判据是 `0.30 + 0.15v`。当 `v > 0.67 m/s` 时硬判据反超软目标，优化器按 0.40 规划出的解必然差几毫米被 `validateTrajectory` 否掉 → 急停 → 卡一下。旧工程配置注释早已写明这一关系：「safe_dist 是优化器的软目标净空，collision_dist 是 validateTrajectory 的硬判据……优化器只能渐近逼近 safe_dist」。
- 修改（代码 + 配置）：
  1. `minco_optimizer.hpp`：`Config` 新增 `clearance_optimizer_margin{0.05}`，`ClearanceModel` 新增 `optimizer_margin{0.05}`，并在构造函数与 `setConfig` 的聚合初始化中一并赋值。
  2. `minco_optimizer.cpp::constraintsFunctional`：速度感知目标改为 `collision_dist + max(|v|·react_time, monitor_margin) + optimizer_margin`，使软目标恒高于硬判据。
  3. `minco_planner.cpp` / `minco_planner.hpp`：新增参数 `minco_optimizer.clearance_optimizer_margin`（默认 0.05，非法值回落 0.05），成员 `clearance_optimizer_margin_`，并在两处 `minco_config` 构建点写入。
  4. `planner_params.yaml`：`speed_aware_clearance` 由 `false` 改为 **`true`**（现场已出现该开关所针对的场景），新增 `clearance_optimizer_margin: 0.05`。
- 未改动：`collision_dist`、`safe_dist`、`replan_react_time`、`ray_range`、两个 `unknown_as_occupied`、`scan_z_min_abs` 均未动。
- 验证：`mas2027_nav_executor` Release 构建通过（1 分 40 秒）；执行器 7 项 CTest 全部通过；YAML 用 `yaml.safe_load` 解析并打印确认（`speed_aware_clearance = True`、`clearance_optimizer_margin = 0.05`、`replan_react_time = 0.15`）。
- 未验证：未跑实车——窄道是否不再卡、窄处主动减速后整体通行时间如何变化，均未实测；余量 0.05 是否合适未调参（调大更保守、更早减速）；开启 `speed_aware_clearance` 后的 L-BFGS 收敛性与轨迹质量未评估（该路径此前只验证到能正常启动）。

## 2026-09-15 — 对齐旧工程的未知区域语义与投影 Z 下限（提速）

- 背景：`/home/mas/mas_nav_2027`（旧工程）在同一场地能跑得很快。逐项对比后确认**限速公式与数值两边一致**（`safe_dist: 0.40` / `collision_dist: 0.30` / `replan_react_time` / `max_velocity: 3.0`），差别在**地图语义**：旧工程 `projection.unknown_as_occupied: false`、`scan_z_min_abs: -1.2`，本工程为 `true`、`-0.5`。
- 现场证据（19:05 次运行 `mas2027_nav_executor_node_11405_1789470310433.log`）：速度已由 0.40 提到 0.81 m/s，但**失败样本全贴在目标上** —— 目标 (2.07,-0.24) 的失败样本在 (2.06,-0.24)（1 cm）、目标 (0.74,0.07) 在 (0.76,0.02)/(0.75,-0.01)、目标 (0.46,-0.10) 在 (0.63,-0.12)，而这些点的可用净空只有 0.299~0.42 m。目标附近多为未观测区域，被判成障碍后通往目标的走廊塌缩到约 0.3 m，`required = 0.30 + 0.15v` 在 v 稍大时即无法满足。
- 改动（`planner_params.yaml` 三项，均对齐旧工程取值）：
  1. `planner.rog_map.projection.unknown_as_occupied`：`true` → **`false`**。按 `projection_layer.cpp::applyValueAndMask()`，该开关同时决定二维 ESDF 的 mask（0=障碍）与 value（254=`kLethalCost` / 255=no-information），是本工程净空读数普遍只有 0.3~0.45 m 的主要来源。
  2. `planner.exploration.unknown_as_occupied`：`true` → **`false`**。该项经 `planner_mode_context` 控制全局搜索的 `smacTraversableCost()` 与目标可通行判定（`global_path_searcher.cpp:72,349`），**必须与第 1 项一致**，否则同一格会被搜索判为不可通行、却被距离场判为自由。
  3. `planner.rog_map.projection.scan_z_min_abs`：`-0.5` → **`-1.2`**。窗口相对雷达，雷达离地超过 0.5 m 时地面会掉出窗口，地面柱子随即成为 UNKNOWN。
- 未改动：`ray_range` 保持 `[0.3, 10.0]`（0.3 m 盲区已由同日的「近距盲区清空」修复解决，退回 0.01 会重新引入车身点云写入）；`collision_dist`、`safe_dist`、`max_velocity`、`replan_react_time` 均未动。
- 验证：`yaml.safe_load` 解析通过并逐项打印确认（两个 `unknown_as_occupied` 均为 `False`、`scan_z_min_abs = -1.2`、`ray_range = [0.3, 10.0]`）；`install/` 下为符号链接，改源码即时生效，无需重新编译。
- 未验证：未跑实车，提速幅度未测；`unknown_as_occupied: false` 后**规划可能穿过尚未观测的区域**（旧工程既有行为，靠地形先验图兜底），该风险未评估；`QueryAdapter::isFree()` 判据是 `value < 253`，UNKNOWN(255) 在 `path_planner` 的目标准入（`path_planner.cpp:123`）与 `local_path_processor` 的种子路径裁剪（`local_path_processor.cpp:26`）处仍被视为"不自由"——与改前一致、无回归，但若目标附近仍失败，应优先查这两处。

## 2026-09-15 — 提速：下调制动距离预算与线速度死区，并新增速度感知净空开关

- 背景（2026-09-15 18:46 次实车日志 `mas2027_nav_executor_node_44323_1789469167367.log`）：车已能到达目标（出现 `Navigation goal reached`），但表现为「慢慢挪动」。用日志里的 `required` 反推被命令的速度只有 0.04–0.40 m/s，约为 `max_velocity: 3.0` 的 13%；37 秒内 19 次 `MINCO path generation failed`、8 次 `not published: COLLISION`、5 次急停。
- 成因：净空要求随速度增长 —— `required = collision_dist + max(v * replan_react_time, monitor_margin)`（`minco_planner.cpp:1588`），而优化器的 ESDF 位置罚项使用**固定**目标 `safe_dist`（`minco_optimizer.cpp` 原 `violaPos = safe_dist - esdf_dist`）。两者在 `v = (safe_dist - collision_dist) / replan_react_time = (0.40-0.30)/0.35 ≈ 0.29 m/s` 处交叉：超过该速度，优化器规划出的轨迹必然被检查器否决 → 急停 → 从静止重规划。现场可用净空约 0.45 m 时模型预测上限 0.43 m/s，与实测 0.40 m/s 吻合。
- 改动一（配置）：`planner_params.yaml` 中 `minco_optimizer.replan_react_time` 0.35 → **0.15**，同样净空下速度上限提到约 1.0 m/s；注释写入计算依据与回退方法。
- 改动二（配置）：`mpc_params.yaml` 中 `deadzone_speed_threshold` 0.1 → **0.02**。`path_executor.cpp:116` 中线速度低于该死区即被清零（角速度仍发出），0.1 m/s 会让低速段「给一点速度就被清掉」，表现为起步顿挫、贴障挪不动。
- 改动三（新增能力，默认关闭）：**速度感知净空** —— 让优化器用与检查器相同的口径。`minco_optimizer.hpp` 新增 `Config::speed_aware_clearance` / `clearance_collision_dist` / `clearance_react_time` / `clearance_monitor_margin` 与 `ClearanceModel` 结构（罚函数是静态成员，故按值传参）；`minco_optimizer.cpp::constraintsFunctional` 在开启时把位置罚项目标净空改为 `required(v)`，并补上 `d(cost)/dv = w · pena' · t · v̂` 一项，使优化器在窄处主动减速、一次规划出可通过检查的轨迹；`minco_planner.cpp` 声明参数 `minco_optimizer.speed_aware_clearance` 并在两处 `minco_config` 构建点写入该口径；`planner_params.yaml` 增加同名参数（`false`）。关闭时行为与改动前完全一致。
- 验证：`mas2027_nav_executor` Release 构建通过；执行器 7 项 CTest 全部通过；隔离 `ROS_DOMAIN_ID=230` 直启 `mas2027_nav_executor_node`（补 `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu`，与 launch 中的设置一致）8 秒，`[ROGMap Config]` 与 `[MincoPlanner]` 正常初始化、无异常退出，说明新参数声明与 `minco_config` 接线不破坏启动；参数路径静态核对与已生效的 `replan_react_time` 同构。
- 未验证：未在实车测速，**0.15 s 的制动距离预算是否够用未经验证**（若出现制动不及，应先回调该项）；速度感知净空只验证到「关闭时行为不变 + 开启后能正常启动」，**开启后的轨迹质量、L-BFGS 收敛性与实际提速效果均未验证**；沙箱内 `ros2 param get` 因 `~/.ros` 只读无法运行，运行时参数取值未实际读出。

## 2026-09-15 — 底盘转发桥改为默认启动（use_ros2_comm）

- 现象：`/cmd_vel` 持续有非零值（当日 18:38 次运行日志中 `required` 反推线速度约 0.45–0.55 m/s）但车不动。
- 原因：`ros2_comm_node` 未运行，而它是全仓库 `/cmd_vel` 的唯一消费者（`mas2027_utils/ros2_comm/src/ros2_comm.cpp:68`；另外两处引用分别是烟测脚本与性能分析器），不启动则执行器的速度指令没有任何接收方。`nav_executor_launch.py` 中 `use_ros2_comm` 默认 `False`（原注释 "Keep hardware output opt-in for the first on-robot test"），默认 launch 不含该节点；18:38 次运行的日志目录中确实没有 `ros2_comm_node_*.log`。
- 修改：`mas2027_nav_bringup/launch/nav_executor_launch.py` 将 `use_ros2_comm` 默认值由 `False` 改为 `True`，注释改为说明它是 `/cmd_vel` 唯一消费者、协议只发 `vx`/`vy`/`nav_state`、只看不发车时可显式 `use_ros2_comm:=False`。**未改动 `ros2_comm.cpp`，协议字段保持原样（仍不含角速度）**：按本次要求不向底盘发送角速度数值，`/cmd_vel.angular.z` 依旧不出桥。`install/` 下该 launch 为符号链接，改源码即时生效。
- 验证：单独启动 `ros2 run ros2_comm ros2_comm_node`（隔离 `ROS_DOMAIN_ID=232`，`ROS_LOG_DIR`/`ROS_HOME` 指向可写目录）通过——日志确认 `UDP initialized. Target: 127.0.0.1:8889`、`Receive thread running`、`UDP Bridge Node started. Listening on port 8888`；`ss -lunp` 确认 8888 已绑定、对端 8889 在监听；`ros2 topic info /cmd_vel` 订阅数为 1；无发布者时按设计约 20 Hz 补发零速并打印 `/cmd_vel timeout, sending zero velocity`（测试期间未发布任何非零速度，车不会动）；测试后进程已终止、8888 已释放。**未验证**：8 秒内未收到对端回传的 `referee_data`（台架上通常无裁判数据），因此「底盘确实收到并执行了指令」仍未确认；未在整机 launch 下级联验证，也未在真车确认能否起步。

## 2026-09-15 — 修复近距盲区把车体自身判成障碍（cmd_vel 恒 0、轨迹闪烁）

- 现象（2026-09-15 18:25–18:30 实车日志 `~/.ros/log/mas2027_nav_executor_node_13541_1789467929023.log`）：发出目标点后车不动，日志反复出现 `Trajectory clearance 0.302 m below required 0.303 m`、`MINCO trajectory not published: COLLISION`（60 次）、`MINCO path generation failed; retrying`（141 次）、`Committed path became unsafe; braking and replanning`（12 次）、`Publishing emergency stop`（12 次）；并出现**负净空** `start clearance -0.073 m`、`Trajectory clearance -0.020 m below required 0.000 m`，即二维距离场认为**车体自己就在障碍物内部**。
- 根因：`raycast_range_min`（当前 0.3 m）以内的体素既不会产生命中（`prob_map.cpp` 中近距点被 `continue` 跳过），也不会被射线扫到（射线自 `raycast_start = (p - cur_odom).normalized() * raycast_range_min + cur_odom` 才开始推进），因此永远停留在 UNKNOWN；而 `projection_layer.cpp::applyValueAndMask()` 在 `unknown_as_occupied: true`（`planner_params.yaml` 当前取值）下把 UNKNOWN 列写成 `mask=0`（二维 ESDF 的障碍源）与 `value=254`（`kLethalCost`），于是 ESDF 在车体自身位置为 0 或负值，`QueryAdapter::isFree()`（`value < 253`）在 `path_planner` 目标准入处亦为假。唯一清理该盲区的代码块（原注释「For the first frame, clear all unknown around the robot」）被 `static bool first` 限制为**只在首帧执行一次**：机器人一旦移动或旋转离开开机位置，就重新落进未观测区并把自己关死。
- 该结论同时解释了同日早前那次 `ray_range` 下限恢复为何没能解决问题：把下限由 0.01 调回 0.3 消除了车身点云写入，但同时把不可观测盲区从 0.01 m 撑到 0.3 m，问题以另一种机制继续存在。
- 修改：`prob_map.cpp` 将首帧一次性清理改为**传感器每移动超过半个体素就重清一次**（半径与原来一致，仍为 `raycast_range_min`）；`prob_map.h` 新增 `last_near_field_clear_pos_` 与 `near_field_cleared_` 两个成员；`ProbMap::resetLocalMap()` 中复位该状态。新增 `insideLocalMap()` 过滤——`SlidingMap::getLocalIndexHash()` 不做边界检查，反复清理后越界写入会踩内存（原实现靠开机时车在图中心侥幸规避）。
- 未改变：清理半径、`unknown_as_occupied`、`ray_range`、`collision_dist` 与近场放宽判据均未改动，「未观测视为障碍」的语义保留。
- 验证：`rog_map` 与 `mas2027_nav_executor` Release 构建通过；执行器 7 项 CTest 全部通过。**未验证**：未做离线复现（`rog_map` 包没有测试目标），未在真机确认车体所在格已变为 FREE、也未确认日志不再刷 COLLISION；需重启 `nav_executor` 后先静止观察 `/rog_map/layer_type` 在车体中心格是 `33`(FREE) 而非 `-1`(UNKNOWN)，再发目标点确认 `/opt_path` 与 `/cmd_vel` 有输出。

## 2026-09-15 — 恢复 ROGMap ray_range 近距下限，避免车身自碰锁死

- `planner_params.yaml` 中 `planner.rog_map.raycasting.ray_range` 由 `[0.01, 10.0]` 改回 `[0.3, 10.0]`（与 `rog_map` 核心默认一致）。涉及 `mas2027_nav_executor/config/planner_params.yaml`。
- 现象：RViz 发目标点后目标已入队，但车不动。日志反复出现 `Trajectory clearance ≈0.28 m below required 0.30 m at (-0.01, -0.01)`、`MINCO trajectory not published: COLLISION`、`Committed path became unsafe; braking and replanning`，失败点几乎贴在车体原点。
- 原因：`ray_range` 下限过小（0.01 m）时，近距车身点云不再被跳过，会写入 ROGMap；距离场在机体系原点附近只剩约 0.28 m，低于 `collision_dist` 0.30 m 与监控阈值，规划与执行层持续急停。`prob_map` 对 `sqr_dis < sqr_raycast_range_min` 的点会 `continue` 跳过，故下限应至少覆盖车体半径。
- 验证：YAML 已改为 `[0.3, 10.0]`，`install/` 下配置副本一并同步；`git diff --check` 预期通过。需重启 `nav_executor` 后重发目标点实车确认；未改规划/控制代码，未跑 CTest。

## 2026-09-15 — RViz 增加备份安全盒（SFC）可视化

- 新增 `/nav_executor/debug/safe_corridor` 话题（`visualization_msgs/msg/MarkerArray`，Transient Local），用青色线框立方体画出 `SimpleCorridorGenerator::generateSafeBox()` 产出的安全盒，并附带 `SFC half=… m` 的半边长文字标注。发布点在 `MincoPlanner::generateBackupTraj()` 生成 SFC 之后，因此随重规划周期刷新；`cleanup()` 中一并 reset。涉及 `minco_planner.hpp`、`minco_planner.cpp`。
- 背景说明（本次排查结论）：本仓库的「走廊」只有这一个盒子，且仅作为备份（急停）轨迹优化器的约束（`backup_opt_->setPolygons({safe_poly})`，`minco_planner.cpp`）。主轨迹优化不使用盒子走廊，`minco_optimizer.cpp` 中没有任何 polygon/halfspace/SFC 代码，只用 ESDF 罚项。本次**仅新增可视化**，未改动盒子生成逻辑、约束构造或任何规划行为。
- `nav_executor_view.rviz` 新增默认启用的 `Safe Corridor (backup SFC)` 显示项（`MarkerArray`，Transient Local + Reliable，与 `/nav_executor/debug/minco_trajectory` 保持一致）。
- 验证：`mas2027_nav_executor`、`mas2027_nav_bringup` Release 构建通过；执行器 6 项 CTest 全部通过（首次运行 `rog_map_command_safety` 因沙箱内 `~/.ros` 只读、spdlog 无法写日志而失败，将 `ROS_LOG_DIR`/`ROS_HOME` 指向可写目录后通过，与本次改动无关）；RViz YAML 可解析且新显示项的 `Enabled`/`Value` 一致；`install/` 下 RViz 配置副本已确认；`git diff --check` 通过。
- 未验证：未在真机 RViz 确认立方体与文字的实际观感；未实测话题在真实重规划循环中的发布频率，以及规划停止后最后一个盒子会因 Transient Local 保留在 RViz 中这一行为。

## 2026-09-15 — 区分 layer_type 的 UNKNOWN 配色并默认显示分类层

- `/rog_map/layer_type` 中 UNKNOWN 的发布值由 `0` 改为 `-1`：新增 `ROGMapROS::kUnknownTypeValue = 255` 作为内部哨兵，`fillLayerGrid()` 对该值透传为 `-1`，其余数值仍夹到不超过 100 后按 `int8_t` 发布。此前 UNKNOWN 发 `0`，而 OccupancyGrid 语义里 `0` 是"自由"，导致 RViz 中未知区域与 FREE 同色、看起来像已探明。涉及 `rog_map_ros2.hpp`。
- 编码现为 `-1`=UNKNOWN、`33`=FREE、`66`=PASSABLE、`100`=OCCUPIED。已核对全仓库引用：`layer_type` 只用于可视化，没有规划器、测试或其他节点依赖其数值。
- `nav_executor_view.rviz` 中 `ROGMAP/Layer Type` 与 `ROGMAP/Height Analysis` 由默认关闭改为默认开启（`Enabled` 与末尾 `Value` 同步置 `true`），Alpha 分别由 0.55/0.8 调整为 0.45/0.6 以减少对其他图层的遮挡；ROGMAP 分组其余显示项与几何参数未改动。
- `rog_map/README.md` 的话题表补充 `layer_type` 的数值编码说明。
- 验证：`rog_map`、`mas2027_nav_executor`、`mas2027_nav_bringup` Release 构建通过；RViz YAML 可解析，ROGMAP 分组 8 项的 `Enabled`/`Value` 逐项核对一致；`install/` 下 RViz 配置副本已确认为新值；`git diff --check` 通过。未在真机 RViz 确认四档配色的实际观感，也未验证 `costmap` 配色下 UNKNOWN 灰度的可辨识度。

## 2026-09-15 — 提高 ROGMap 写入与可视化的 Z 范围

- `planner_params.yaml` 中 `planner.rog_map` 的三项 Z 参数调整：`raycasting.local_update_box` 由 2.5 提到 4.0、`visualization.range` 由 1.5 提到 4.0、`map_size` 由 2.5 改回 4.5。目标是让 `/rog_map/occupied` 能显示更高的墙体（原上限约 ±0.75 m）。涉及 `mas2027_nav_executor/config/planner_params.yaml`。
- 排查结论（已写入配置注释）：Z 方向可用高度由四道限制串联决定——`raycasting.local_update_box`/2 决定点云能否写入（`prob_map.cpp` 中超出盒子的点被投影到盒壁并记 `update_hit=false`，永不成为占据体素）、`visualization.range`/2 决定能否发布、`virtual_ceil_height`/`virtual_ground_height` 是绝对上下界、`map_size[2]`/2 是容器上限（`updateLocalBox()` 与 `boundBoxByLocalMap()` 都会按局部地图边界夹取）。因此只抬高其中一两项不会生效，`map_size` 是其余各项的天花板。
- 调整后实际能力：可写入与可显示 z 半高均为 2.00 m，扣除 `map_sliding.threshold` 0.2 的跟随滞后约 1.80 m；下边界由 `virtual_ground_height` 限制在 -1.00 m。更早一版曾只改前两项而 `map_size[2]` 为 2.5，当时上限仍被夹在 ±1.25 m，故补上 `map_size` 一并调整。
- 验证：YAML 可解析，四道限制的取值用脚本逐项核对通过；`mas2027_nav_executor` 构建通过，`install/` 下配置副本已确认同步为 4.5/4.0/4.0。未在真机 RViz 观察紫色块高度变化；未评估抬高 `local_update_box` 后 raycast 清空射线的耗时增量，也未验证提升 z 范围后的建图帧耗时。

## 2026-09-15 — 为 ROGMap ROS 2 适配层补充中文注释

## 2026-09-15 — 为 ROGMap ROS 2 适配层补充中文注释

- 仅新增注释，未改动任何代码逻辑、接口、参数或话题。`rog_map_ros2.hpp` 增加文件级总览（类职责、线程模型、关键约定），并为成员变量、`getPriorMapTransform`/`bindNode`、`create*` 封装、`ROSCallback` 接力区、`odomCallback`/`cloudCallback`/`watchdogCallback`、`updateWorkerLoop`、`captureVizFrame`/`vizCallback`、`hasVisualizationSubscriber`、各 `fill*` 与 `initializeRos`、构造函数与静态 `visualize*` 辅助函数补充逐段说明。
- 注释中记录了若干阅读时易踩的既有约定：`updateMap()` 在 `ros_callback_en` 下不生效、`map_io_mutex_` 目前仅更新线程持有、`updete_lock` 为上游拼写、可视化 heavy 层限频 0.5 s、`fillLayerGrid` 会把 254/255 一并夹到 100、`/rog_map/layer_value` 实际发布二值 mask、`fillLayerHeightDeltaCloud` 对 FREE 单元会发布未定义的 z。
- 验证：`rog_map` 与 `mas2027_nav_executor` Release 构建通过，`git diff --check` 通过。注释内容为静态阅读所得，未在真机运行时逐条复核（如 `use_intra_process_comm` 在当前跨进程部署下的实际效果）。

## 2026-09-14 — 修正 ROGMap 边界标记时间戳并重建

- `rog_map_ros2.hpp` 的 Map Bound 标记直接传递 `rclcpp::Time`，不再把秒数误作纳秒；修正边界线、文字和原点标记的时间戳。未更改地图或规划逻辑。
- 验证：`interfaces`、`rog_map`、`mas2027_nav_executor` Release 构建通过，`git diff --check` 通过。当前运行的导航进程未重启，尚未在重启后的 RViz 验证 `Map Bound` 状态。

## 2026-09-14 — ROGMap 占据点改为立方体显示

- RViz 的 `ROGMAP/Occupied` 由 `Points` 改为 `Boxes`，保留 0.05 m 尺寸，与 ROGMap 当前 0.05 m 体素分辨率一致；仅影响渲染，不改地图、规划或 `/rog_map/occupied` 消息。涉及 `nav_executor_view.rviz` 与 README。
- 验证：RViz YAML 可解析，`Occupied` 的 `Boxes` 样式、0.05 m 尺寸及默认启用状态校验通过；`mas2027_nav_bringup` 构建与 `git diff --check` 通过。未在真机 RViz 检查视觉效果，上一轮日志中的 QoS 不兼容需单独确认。

## 2026-09-14 — RViz 增加 ROGMap Map Bound

- `ROGMAP` 分组新增默认启用的 `MarkerArray` 显示项 `/rog_map/map_bound`，使用发布器匹配的 Best Effort/Volatile QoS；显示可视范围、局部地图范围及更新范围，保留已有占据点、距离场和轨迹显示。涉及 `nav_executor_view.rviz` 与 README。
- 验证：RViz YAML 可解析，`Map Bound` 的类型、启用状态、话题及 QoS 校验通过；`mas2027_nav_bringup` 构建与 `git diff --check` 通过。未连接真机检查渲染效果。

## 2026-09-14 — 参考 navi_minco_bit 整理 ROGMap RViz 显示

- 将原单项 `/rog_map/occupied` 改为 `ROGMAP` 分组，参照 `navi_minco_bit` 配置动态/静态/合成投影栅格、类型图、2D 距离场、高度分析和占据点；话题与本仓库 `ROGMapVisualizer` 发布器逐项核对，统一使用 Best Effort/Volatile。占据点和低透明度距离场默认启用，诊断栅格默认关闭且绘制在背景，保留全局路径和 MINCO 轨迹可见性。涉及 `nav_executor_view.rviz` 和 README。
- 验证：`mas2027_nav_bringup` 构建通过；RViz YAML 可解析，`ROGMAP` 分组含 7 个预期话题且启用状态符合配置；`git diff --check` 通过。未在真机 RViz 渲染、TF 和实时点云上实测；ROGMap 先验图融合关闭，静态层显示项仅供诊断。

## 2026-09-14 — 将 ROGMap 接回在线规划与制动

- `nav_executor_planner` 重新在进程内构造 ROGMap，订阅 `/cloud_registered` 和 `/Odometry`；MINCO 走廊、优化与轨迹安全改用其在线占据和距离查询。全向 Kino A* 在 ROGMap 滑窗内叠加其占据判断，滑窗外继续使用静态地形与 `map_server` 当前动态代价图；MPC 下一段指令增加可调净距 `node.rog_map_clearance` 检查，缺失查询时制动，并拒绝 ROGMap/规划/里程计帧名不一致的配置。底盘 `/cmd_vel` 接口与无预测行为不变。涉及执行器规划、搜索、安全监测、参数、CMake/package 依赖。
- 将 MINCO 的同名兼容查询接口改为包含 ROGMap 原生接口，避免重复定义和 ABI 分裂；RViz 增加紫色 `/rog_map/occupied` 在线点层，README 更新链路与实车标定说明。
- 验证：`mas2027_nav_executor` 与 `mas2027_nav_bringup` Release 编译通过；执行器 6 项 CTest（含 ROGMap 净距制动回归）通过；参数/RViz YAML 与 `git diff --check` 通过；隔离 ROS 域启动执行器 10 秒，确认加载 ROGMap 配置和注入 MINCO 查询，因未接入点云出现预期的无输入警告。未连接双雷达、未验证地面投影/外参及真车闭环；启动前需静止检查 `/rog_map/occupied` 与轨迹净距。

## 2026-09-14 — 稳定当前动态障碍的短时漏检

- `map_server` 在点数与连通域筛选后、代价膨胀前，对已检出的栅格保留最近观测时间；新障碍立即进入 `/dynamic_cost_map`，短时漏检默认保留 0.3 s，持续无观测则清除。参数 `local_map.dropout_hold_seconds` 可调，设为 0 可关闭。规划器与 RViz 共用同一稳定后的地图；未把持续不匹配的静止障碍从避障图中删除。涉及 `map_server_node.cpp/.hpp`、点云烟测和 README。
- 验证：`map_server` Release 编译通过；隔离 `ROS_DOMAIN_ID=229` 的合成点云烟测覆盖首次检出、短时漏检保留和超时清除；`git diff --check` 通过。未重启当前真机导航进程、未实测先验 PCD 与实时点云配准或 RViz 闪烁改善。

## 2026-09-13 — 接入全向 Kino 搜索与弧长速度种子

- 全局搜索改为全向底盘的速度向量格点 A*：以位置、运动方向、速度档为状态，按速度/加速度可行性扩展，逐边检查静态地形方向、当前动态障碍及软代价；从实测里程计速度出发，不引入未来障碍预测。涉及 `path_planner/search/omni_kino_astar.*`、`global_path_searcher.*`、`minco_planner.cpp`、CMake。
- 在 MINCO 分段时间分配前加入弧长速度剖面：按速度平方前后向传播、角点速度上限和加速/巡航/制动时间形成时间种子；优化后的 MINCO 轨迹仍是唯一执行与安全检查时间轴。起始速度与可用刹车距离不相容时保留原保守时间种子。涉及 `path_planner/trajectory/arc_length_speed_profile.*`、`minco_planner.cpp`、README 及启动日志。
- 验证：`mas2027_nav_executor` 编译、地形/MPC/全向搜索/弧长速度剖面共五项单元测试、隔离 ROS 域双目标抢占烟测及 `git diff --check` 通过；烟测在第二目标后曾出现净距 0.490 m 低于运行监测阈值 0.500 m 的安全制动警告，需实车标定地图与余量。未连接真机验证全向加减速、贴障绕行或实地语义方向。原 HW 曲率原语与指导走廊、台阶速度窗、后置 MINCO 速度优化和腿式 LPV/FDDP 未原样迁入，当前不宣称完整 HW 等价。

## 2026-09-13 — 适配全向底盘转向与航向跨界

- 确认底盘执行 `/cmd_vel.linear.y` 后，保留 MPC 的二维全局速度到车体系 `x/y` 转换；解除配置中角速度/角加速度的零锁定，让规划航向可以由 MPC 跟踪，并对叠加 `/cmd_spin` 后的角速度施加限幅。涉及 `path_executor/mpc/mpc_solver.cpp`、`path_executor/path_executor.cpp`、`config/mpc_params.yaml`。
- MPC 参考航向逐点按最短角差展开，避免穿过 ±π 时绕远路；增加侧向、转向及正反两个跨界方向的回归检查，README 记录全向控制接口与实车限值标定要求。
- 验证：`mas2027_nav_executor` 编译、地形栅格/距离场/MPC 三项单元测试、隔离 ROS 域的双目标抢占烟测、launch 参数解析和 `git diff --check` 通过；未连接真机验证侧向、转向动力学或 `/cmd_spin` 叠加效果。HW 的 Kino A*、弧长速度剖面和 FDDP 仍未移植，不宣称与 HW 全部等价。

## 2026-09-13 — 拆分导航执行器并移除 MincoFsm

- `TaskManager` 接管目标、周期重规划、原有的限时推离恢复与执行许可；删除 `MincoFsm` 及其内部 20 Hz 定时器，`MincoPlanner` 不再管理目标生命周期。新目标抢占后以轨迹时间戳拒绝晚到旧轨迹；规划失败或轨迹失效时停止输出运动命令，里程计超时同样制动。涉及 `task_manager/`、`path_planner/`、`path_executor/`、节点与 `node_params.yaml`。
- 按功能整理 `path_planner/search`、`path_planner/trajectory`、`path_executor/mpc`、`path_executor/monitoring`、`path_executor/state`、`common/environment` 的实际源码与头文件；更新 CMake、单元测试和 README。`vendor/` 保留数值计算后端与 qpOASES。
- 验证：`mas2027_nav_executor` 编译、地形栅格/距离场/MPC 三项单元测试、隔离 ROS 域的单目标与双目标抢占烟测通过，`git diff --check` 通过；未连接真机验证闭环控制或触发贴障推离恢复。HW Kino A*、弧长速度剖面与 FDDP 尚未迁入，当前不宣称算法等价。

## 2026-09-13 — 移除先验定位的 Z 轴锁定

- 删除 `odom_localizer` 的 `update.lock_z` 参数及其变换裁剪代码；GICP 估计的完整六自由度 `map→odom` 变换现在直接进入 EMA 与 TF 发布，允许实时点云与先验 PCD 在地板、天花板高度上对齐。涉及 `params.yaml`、`odom_localizer_node.hpp/.cpp` 与 README。
- 验证：完成源码引用扫描与 `git diff --check`；尚未使用真机点云确认 GICP 的 Z 估计稳定性，启动后需在 RViz 检查先验/实时云重合情况。

## 2026-09-13 — 修复轨迹距离场热路径并记录安全拒绝原因

- `TerrainMapQuery` 不再在每次距离查询时复制整张约束图；`TerrainGrid` 为地图内容维护版本号，只有静态/动态栅格发生变化才重建二维距离场，重复动态帧仅更新新鲜时间戳。涉及 `terrain_grid.hpp/.cpp`、`terrain_map_query.hpp/.cpp` 及单元测试。
- `PathPlanner` 将原“Accepted goal”改为准确的“Queued goal”；`MincoPlanner` 在局部轨迹未发布时节流记录失败原因，安全检查器记录位置、实际净距与所需净距，便于区分规划耗时与安全拒绝。涉及 `path_planner.cpp`、`minco_planner.cpp`、`trajectory_safety_checker.cpp` 和 README。
- 实时只读诊断：RViz Goal、执行器订阅与 RViz 轨迹订阅匹配，起点和目标处于同一可通行区域；当前机器人静态地图净距约 0.255 m，小于配置的 0.30 m 轨迹碰撞距离。因此保留安全阈值，未通过降低半径强行发布轨迹。
- 验证：`mas2027_nav_executor` 重新编译通过；隔离 ROS 域同一目标的 MINCO 优化耗时从约 5.6 s 降到约 0.0005 s，生成 52 点轨迹；地图版本与距离更新测试及执行器全量 3 项单元测试通过；`git diff --check` 通过。未在真机重新发送 Goal 或验证实际墙体距离。

## 2026-09-13 — 忽略编辑器缓存、补足贴地障碍检测及目标诊断

- `.gitignore` 忽略 clangd 生成的 `.cache/`。`map_server` 缩小地面平面内点容差，并对高于地面的低矮点采用更严格的先验 PCD 匹配距离，避免 0.20 m 的原距离阈值把贴地障碍误判为先验地面；保留可调的最小离地高度与匹配距离参数。涉及 `map_server_node.cpp/.hpp`。
- `nav_executor` 在收到 RViz Goal、缺少规划里程计及接受目标时打印明确日志，便于区分“目标未送达”和“地图/TF/里程计不就绪”；涉及 `nav_executor_node.cpp`、`path_planner.cpp`。新增贴地障碍 PCD 夹具、点云回归与隔离 ROS 域的 Goal→轨迹烟测，并更新 README。
- 验证：`map_server`、`mas2027_nav_executor` 编译通过；贴地 11 cm 障碍与地面点云烟测通过；隔离 ROS 域中原 RViz 目标 `(2.56, 0.44)` 生成 52 点轨迹；执行器 3 项单元测试通过；`.cache/` 忽略规则和 `git diff --check` 通过。未连接真机复现当时的动态地图或检验实地低矮障碍，相关高度/配准阈值仍须实测标定。

## 2026-09-13 — RViz 单独显示当前动态障碍

- `nav_executor` 从 `/dynamic_cost_map` 当前帧提取硬阻塞格，发布青色点标记 `/nav_executor/debug/dynamic_obstacles`；收到空帧时发布空标记清除旧点。RViz 默认启用独立的动态障碍点层，避免静态/动态都为红色时难以区分，点层高于地形图且不会整幅遮挡轨迹。
- 验证：执行器编译通过，地形栅格/距离场/MPC 三项单元测试通过，RViz 配置可解析且 `git diff --check` 通过。此轮未重复运行 `map_server` 点云烟测，也未在真机 RViz 中检查实际渲染。

## 2026-09-13 — 移除 ROGMap 在线链路

- `nav_executor` 不再实例化、链接或声明 `rog_map`；移除其启动依赖、规划参数和 RViz 显示。`map_server` 发布的静态地形与当前动态栅格现在是唯一在线地图输入。
- 新增执行器内置 `TerrainMapQuery`：将合并后的二维约束栅格转换为 signed-distance field 与 XY 梯度，并注入 MINCO 的全局搜索、走廊、优化器和轨迹安全检查，替换原 ROGMap distance/gradient 查询。
- 验证：`mas2027_nav_executor`、`mas2027_nav_bringup` 编译通过；地形栅格、二维距离场与 MPC 单元测试通过；RViz YAML 和启动参数可解析。未进行真机闭环验证。

## 2026-09-13 — 移除动态障碍预测并收紧静态图膨胀

- 删除 `CostMaps` 消息、目标跟踪器、预测渲染器及 20 帧预测链路；`map_server` 改为仅发布 `/dynamic_cost_map` 当前帧，`nav_executor` 的全局搜索、MINCO 轨迹验收与 MPC 制动统一读取该当前动态图。
- 默认静态地形图膨胀由 0.20 m 满代价 + 0.60 m 截止调整为原始障碍硬约束 + 0.25 m 软代价，避免过厚的先验地图额外挤占可通行空间；ROGMap 的局部 ESDF 检查仍保留。
- 原因：已配准点云受底盘振动影响时，静止物体会产生虚假速度，未来预测会将其错误扩大；现阶段先验地图与实地地形厚度也未完成标定。
- 验证：`interfaces`、`map_server`、`mas2027_nav_executor` 编译通过；当前动态图烟测确认地面不占用、抬高障碍可检出；执行器地形/MPC 单元测试通过。未进行真机验证。

## 2026-09-13 — 修复规划约束图遮挡轨迹

- 默认 RViz 视图改用 `/planning_constraints_markers` 高对比度点层，并按执行器实际约束着色：红色为当前硬阻塞、橙色为预测占用、黄色为方向受限；点层抬高到地图平面上方，不再被整张不透明栅格遮住路径。
- `/planning_constraints` 完整栅格保留为默认关闭的诊断显示，且始终绘制在轨迹后方；原始地形代价图继续作为半透明背景。
- 验证：待重新编译后运行执行器单元测试与 RViz 配置解析；未连接真机或实际 RViz 渲染检查。

## 2026-09-13 — RViz 高对比度规划约束图

- `nav_executor` 新增瞬态 `/planning_constraints`：红色数值表示静态地形或当前动态层的硬阻塞，橙色表示未来 2.0 s 内的预测占用，黄色表示方向受限地形；图由执行器实际使用的地形/动态判定生成。
- RViz 将原始 `HW Cost Map` 调为半透明背景，并默认启用不透明的 `Planning Constraints` 前景层；README 记录颜色含义。
- 验证：补充地形栅格单元测试，覆盖静态阻塞、方向约束、当前动态与未来动态四种可视化值；尚未启动 RViz 实测配色。

## 2026-09-13 — 动态地图地面误检抑制

- `map_server` 在先验 PCD 差分前，对局部点云做近水平、位于车体下方且具有足够点数的地面平面剔除，避免稀疏或小幅错位的先验点云把地面投影成动态障碍；保留高于地面的障碍点与原有 21 帧预测输出。涉及 `map_server_node.cpp`、PCL 构建依赖及合成点云烟测。
- 验证：`map_server` 编译通过；合成地面单独输入时当前代价图为空，加入抬高障碍物后当前代价图非空且仍为当前帧 + 20 帧、步长 0.1 s。未用真机点云验证，也未排除实车 TF/PCD 配准偏差。

## 2026-09-13 — 静态地形约束与动态预测代价图接入原生执行器

- `map_server` 从 `/cloud_registered` 与先验 `lab3.pcd` 提取动态点，经过 ROI、体素降采样、栅格聚类、目标跟踪和预测渲染，在 `/cost_maps` 发布当前帧 + 20 个未来帧（`prediction_dt=0.1 s`）；`/cost_map`、`/direction_map` 仍是独立静态地形层。
- `mas2027_nav_executor` 新增地形栅格快照：全局 A* 融合静态地形障碍/方向与动态当前帧，局部路径稀疏化和 MINCO 轨迹验收加入地形检查，`/cmd_vel` 跟踪器对地形和预测占用执行制动保护；保留 ROGMap 的局部 ESDF 和原底盘接口，不移植腿部模式。
- MPC 参考时域扩展为 2.0 s，制动后重置求解器的上一帧控制状态；启动文件启用 `map_server` 动态检测并配置静态 PCD 与 PCL 运行库路径；README 说明现有能力及尚未移植的 HW Kino A*、速度剖面、FDDP 与预测障碍绕行优化。
- 验证：`map_server`、`mas2027_nav_executor`、`mas2027_nav_bringup` 编译通过；地形搜索、预测渲染、40 步 MPC 单元测试通过（单次空障碍求解约 2.7 ms）；合成点云烟测收到 21 帧、0.1 s 步长且当前帧有动态占用；launch 参数可解析、`git diff --check` 通过。未做真机控制或完整导航闭环测试。

## 2026-09-12 — 清理独立执行器的遗留表述

- 将 LIO、ROGMap 独立节点、轨迹消息、地形图转换脚本和 MINCO 内部注释改为独立执行器语义，避免继续暗示旧 Nav2 链路仍可运行。
- 更新 Nav2 移除记录的验证结论：`rog_map`、`mas2027_nav_executor`、`mas2027_nav_bringup` 均已编译，独立 launch 参数可解析，包清单不再声明 Nav2 依赖。
- 验证：运行 `git diff --check`、依赖扫描与 `ros2 launch mas2027_nav_bringup nav_executor_launch.py --show-args`；未启动真车硬件节点。

## 2026-09-12 — 移除 Nav2 并完成执行器解耦

- 删除 Nav2 server 启动、参数、Behavior Tree、默认视图以及 `pb_nav2_plugins`、`fake_vel_transform`、航点/BT 编辑器等旧链路。
- 删除 `minco_planner`、`minco_controller` ROS 插件包；把实际使用的 MINCO、MPC、qpOASES 源码迁入 `mas2027_nav_executor/vendor`。
- 规划器固定为 ROGMap `EXPLORATION`，移除 Nav2 Costmap、SMAC、pluginlib 与 `nav2_util` 依赖；ROGMap 参数声明改用标准 rclcpp API。
- `nav_executor_launch.py` 不再使用 `nav2_common/RewrittenYaml`，独立执行器成为唯一在线导航入口。
- 重写根 README 和导航说明，删除已失效的 Nav2 架构文档。
- 验证：`rog_map` 与 `mas2027_nav_executor` 编译通过；`mas2027_nav_bringup` 在删除空 Behavior Tree 安装项后重新验证。

## 2026-09-12 — RViz 增加 HW 地图显示项

- Nav2 与独立 nav_executor 两份 RViz 视图均新增默认启用的 `HW Cost Map`，直接显示 `/cost_map`。
- 两份视图均新增默认关闭的 `HW Direction Map`，需要时可在 Displays 面板勾选并显示 `/direction_map`。
- `nav_executor_launch.py` 现在同步启动 `terrain_map_server`；该模式不启动 Nav2 Costmap server，因此没有 Global/Local Costmap 话题。
- 验证：RViz 配置 YAML 可解析并随 `mas2027_nav_bringup` 安装；未启动图形界面检查渲染效果。

## 2026-09-12 — 固化修改记录规则

- 新增仓库级 `AGENTS.md`：每个产生文件改动的任务在交付前必须更新本文件。
- 将本文件加入文档索引。
- 验证：检查规则文件与文档链接；未涉及运行时行为。

## 2026-09-12 — HW 地形代价接入规划与控制

- `terrain_map_server` 发布的 `/cost_map` 接入 Nav2 全局和局部 `StaticLayer`；关闭三值化以保留连续膨胀代价，供 SMAC 全局搜索使用。
- `MincoMpcController` 增加前视代价采样：普通代价区渐进降速，致命代价区停车；参数位于 `FollowPath.terrain_cost_control`。
- 主导航启动文件自动启动 `terrain_map_server`，默认加载 `lab3_terrain.msgpack`。
- 验证：`map_server`、`minco_controller`、`mas2027_nav_bringup` 编译通过；launch 可解析；`/cost_map` 实测为 reliable + transient-local。
- 未验证：未启动完整真车导航，未向底盘发送速度。

## 2026-09-12 — HW map_server 首版移植

- 新增 `interfaces/msg/navigation/CostMaps.msg` 与 `map_server` ROS 2 包。
- 新增 PGM/YAML 到 HW msgpack 地形格式的转换脚本，并从 `lab3.pgm` 生成 `lab3_terrain.msgpack`。
- 当前转换仅包含 `FLAT` 和 `OBSTACLE`；PGM 无法推导坡道、台阶及方向语义。
- 验证：消息包与 map_server 编译通过；节点成功加载 770×347、0.05 m/px 的 lab3 地图并发布地图话题。
