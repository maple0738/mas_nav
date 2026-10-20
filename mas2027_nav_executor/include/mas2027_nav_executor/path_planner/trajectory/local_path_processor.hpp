#ifndef MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_
#define MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_

#include "minco_core/header.hpp"
#include "mas2027_nav_executor/common/environment/clearance_gate.hpp"

#include <string>

namespace minco_planner {

class PlannerModeContext;

/// 记录"种子净空判据第一次否决"的那一点：clearance 落在 [collision_dist, collision_dist + slack]
/// 且 required == collision_dist 说明两道门门槛不一致（该改判据），明显更低则是真的过不去。
struct SeedRejectInfo
{
  bool valid{false};           ///< 是否真的捕获到了一次否决
  Eigen::Vector3d point{};     ///< 被否的采样点（map/odom 帧，与 buildSeed 入参同帧）
  double clearance{0.0};       ///< 该点实测净空
  double required{0.0};        ///< 该点被判据要求的净空
  double arc_from_start{0.0};  ///< 该点离规划起点的平面距离（近场是按弧长划分的）
  bool near_field_relaxed{false};  ///< required 是否来自近场放宽（而非完整 collision_dist）
  bool terrain_blocked{false};     ///< 是否死在 terrain_segment_free 而不是净空
  /// "只差净空"：该点是自由格、仅净空小于 required，段门不因这类失败否决种子，该标志仅供日志。
  bool clearance_only{false};
  /// 停车/脱困前缀专用：true 表示"第一个净空不足的采样点之前的可用安全段，减去收尾余量后不足
  /// 最小前缀长度"；此时否决的定义是"前方太短"而非"净空不够"，`clearance` 仍照实记录净空。
  bool length_limited{false};
};

/// 自动判读"这条被否的路径说明什么"，写进日志以免人工手算两套判据的差异；种子门已不再用净空
/// 否决，故 `SEED_GATE_STRICTER` 降为自检分支，只在"某道门拿净空否决种子/前缀"回归时报出。
/// `verdict=` 取值一览：
///   - `GEOMETRY`          连近场规则都过不了 → 几何真的过不去，拒绝正确，别动阈值。
///   - `SEED_GATE_STRICTER`近场能过、更严要求过不了且起点净空 ≥ collision_dist → 判据不一致。
///   - `PREFIX_TOO_SHORT`  判据本身满足，否决来自"安全段太短、凑不够最小前缀长度"。
///   - `TERRAIN` / `NONE`  前者死在 terrain/动态层、与净空无关；后者没有捕获到否决点。
std::string classifySeedReject(const SeedRejectInfo & info,
  double start_clearance, double collision_dist, double near_field_slack);

struct LocalPathSeed
{
  bool valid{false};
  bool local_end_is_goal{false};
  // 原局部段被阻挡或净空偏小，已替换为滚动窗口内栅格搜索得到的绕行路径。
  bool used_dynamic_detour{false};
  // 原路径只有净空不足时，找到并采用了满足优化器软目标净空的绕行种子。
  bool used_clearance_detour{false};
  // 稠密参考线本身低于窄道硬地板时，在局部栅格内搜索并替换为满足该地板的路径。
  // 这条路径仍需通过 MINCO 轨迹门和执行前指令门。
  bool used_narrow_clearance_repair{false};
  // 双侧受限的低净空段在进入 MINCO 前沿路径法向小幅移向 ESDF 净空峰值。
  bool used_narrow_centering{false};
  // 当前 ROGMap 窗口内无路可绕，种子停在障碍前的安全点；即使不是任务终点也必须用零末速度。
  bool stop_at_local_end{false};
  // 已生成绕行路径或安全停车前缀但种子仍未过校验：据此区分"修复后被否决"与笼统的 COLLISION。
  bool repair_rejected{false};
  // 正常路径、局部绕行、完整停车前缀三层都失败后退化使用的短距离脱困前缀（creep）：整段落在
  // 近场豁免半径（collision_dist）以内、末速度为零，只用于把车从"贴住障碍停死"的状态挪出来。
  bool used_escape_prefix{false};
  // 稠密种子判据首次否决点：四层兜底全败时说明"为什么全败"，修复成功后也记录以解释兜底决策。
  SeedRejectInfo dense_reject;
  std::vector<Eigen::Vector3d> dense_path;
  std::vector<Eigen::Vector3d> sparse_waypoints;
  std::vector<double> local_magnitudes;
};

class LocalPathProcessor
{
public:
  void configure(double lookahead_dist,
    double max_vel,
    double max_acc,
    double traj_goal_tolerance,
    double collision_dist,
    double preferred_clearance,
    rclcpp::Logger logger);

  void updateLimits(double max_vel, double max_acc, double traj_goal_tolerance);

  void setClearanceDetourEnabled(bool enabled) { clearance_detour_enabled_ = enabled; }

  /// 窄道稀疏化：稀疏折线的跳跃段必须保留不低于该地板的净空。它不是轨迹硬门，
  /// 只避免把绕过窄口的稠密折线简化成切角弦，导致 MINCO 在该段反复降到 floor 以下。
  void setNarrowPassage(bool enable, double minimum_clearance);

  /// 短距离脱困前缀（第四层兜底）参数：enable=false 即完全恢复"三层兜底"行为；min_length/buffer
  /// 是障碍前至少保留的净空与收尾余量，长度上限固定为 collision_dist（整段在近场豁免半径内）。
  void setEscapeOptions(bool enable, double min_length, double buffer);

  /// 折线净空不足**不否决**种子，只有占据 / 地形 / 查询失效才硬否决；切勿再引入"失败后切回净空
  /// 硬否决"的开关，它会在近场把种子永久拒掉。不足处记进 `seed.dense_reject.clearance_only`。
  LocalPathSeed buildSeed(const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
    const geometry_msgs::msg::PoseStamped & current_pose,
    const PlannerModeContext & mode_context,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free = {}) const;

private:
  /// 种子 / 绕行 / 停车前缀共用的「完整净空要求」= collision_dist_ 减 ESDF 抖动余量
  /// （effectiveClearanceThreshold）：必须与发布前整条轨迹检查、执行前指令门同值，
  /// 否则会规划放行、执行刹车；近场半径仍用 collision_dist_（车体安全半径）。
  double fullClearanceRequirement() const
  {
    return mas2027_nav_executor::effectiveClearanceThreshold(collision_dist_);
  }

  std::vector<Eigen::Vector3d> extractLocalPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
    const Eigen::Vector3d & cur_pos) const;

  bool centerNarrowRoute(std::vector<Eigen::Vector3d> & path,
    const Eigen::Vector3d & start,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double start_clearance, bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free) const;

  bool clipLocalPathByRogBoundary(
    std::vector<Eigen::Vector3d> & path, const PlannerModeContext & mode_context) const;

  /// 最后一个参数是可选输出：非空指针时在返回 false 前填入"第一次否决"的那一点，nullptr 零开销。
  /// `enforce_clearance=false`（种子/绕行/稀疏复核）只有占据、地形、查询无效才算否决：净空是
  /// **轨迹**的属性，交给 MINCO 与轨迹级三道门（同旧工程 `isLineFree` 口径）；`=true`（停车前缀）
  /// 则净空不足即否决，前缀末端是"车要停在哪里"，否则停车轨迹会被发布前校验拒掉、车失去指令。
  bool segmentClear(const std::shared_ptr<rog_map::MapQueryInterface> & query,
    const Eigen::Vector3d & from,
    const Eigen::Vector3d & to,
    const Eigen::Vector3d & planning_start,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    SeedRejectInfo * reject_info = nullptr,
    bool enforce_clearance = false,
    double preferred_clearance = 0.0,
    double clearance_override = 0.0) const;

  /// 默认按软口径逐段检查；传入 preferred_clearance 时，额外要求绕行候选达到软目标净空。
  bool pathClear(const std::vector<Eigen::Vector3d> & path,
    const Eigen::Vector3d & planning_start,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    SeedRejectInfo * reject_info = nullptr,
    double preferred_clearance = 0.0,
    double hard_clearance_floor = 0.0) const;

  bool searchDynamicDetour(const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    std::vector<Eigen::Vector3d> & detour,
    SeedRejectInfo * reject_info = nullptr,
    double preferred_clearance = 0.0,
    double hard_clearance_floor = 0.0) const;

  bool buildSafeStoppingPrefix(const Eigen::Vector3d & start,
    const std::vector<Eigen::Vector3d> & path,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    std::vector<Eigen::Vector3d> & prefix,
    SeedRejectInfo * reject_info = nullptr) const;

  /// 停车前缀核心（build*Prefix 共用）：沿 path 前进，在第一个净空不足的采样点之前收尾，得到
  /// 「障碍前 buffer 米、至少 min_length 米、最多 max_length 米（max_length<=0 表示不限）」的前缀。
  bool buildStoppingPrefixCore(const Eigen::Vector3d & start,
    const std::vector<Eigen::Vector3d> & path,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    double buffer,
    double min_length,
    double max_length,
    std::vector<Eigen::Vector3d> & prefix,
    SeedRejectInfo * reject_info = nullptr) const;

  /// 第四层兜底：三层都失败时，用"不比当前净空更差"的短前缀把车挪出贴死状态。
  bool buildEscapePrefix(const Eigen::Vector3d & start,
    const std::vector<Eigen::Vector3d> & path,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    std::vector<Eigen::Vector3d> & prefix,
    SeedRejectInfo * reject_info = nullptr) const;

  double lookahead_dist_{5.0};
  double max_vel_{2.0};
  double max_acc_{4.0};
  double traj_goal_tolerance_{0.5};
  double collision_dist_{0.30};
  double preferred_clearance_{0.35};
  bool narrow_passage_enable_{false};
  double narrow_passage_clearance_{0.0};
  bool clearance_detour_enabled_{true};
  bool escape_enable_{true};
  double escape_min_length_{0.08};
  double escape_buffer_{0.05};
  rclcpp::Logger logger_{rclcpp::get_logger("LocalPathProcessor")};
  // buildSeed() 是 const，日志计数需 mutable：前 N 次逐条打出（首次现场最关键），之后 2 s 节流。
  mutable uint64_t repair_reject_logged_{0};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_
