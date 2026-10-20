#include "mas2027_nav_executor/path_planner/trajectory/local_path_processor.hpp"
#include "mas2027_nav_executor/common/environment/clearance_gate.hpp"
#include "mas2027_nav_executor/common/environment/narrow_passage_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>

namespace minco_planner {

namespace {

/// 近场放宽的抖动余量，与 ESDF 抖动余量同源共用常量，避免两处各写 0.02 后漂移。
constexpr double kNearFieldSlack = mas2027_nav_executor::kEsdfJitterTolerance;
constexpr double kStoppingBuffer = 0.15;
constexpr double kMinimumStoppingPrefix = 0.30;
// "修复后被否决"前 N 次日志不节流：排障最需要第一次的现场，之后退回 2 s 节流。
constexpr uint64_t kRepairRejectLogFirstN = 10;

struct OpenCell {
  size_t index{0U};
  double score{0.0};
  bool operator>(const OpenCell &other) const { return score > other.score; }
};

}  // namespace

std::string classifySeedReject(const SeedRejectInfo &info,
                               double start_clearance,
                               double collision_dist,
                               double near_field_slack) {
  if (!info.valid) {
    return "NONE";
  }
  if (info.terrain_blocked) {
    return "TERRAIN";
  }
  if (info.length_limited) {
    // 否决的定义是"第一个净空不足点之前的可用段太短"，不是该点没有净空。
    return "PREFIX_TOO_SHORT";
  }

  // 近场规则：场外取发布/执行两道硬门统一后的有效阈值，场内为"不比现在更差 - slack"。
  const bool in_near_field = info.arc_from_start < collision_dist;
  const double near_rule_required =
      in_near_field ? std::max(0.0, start_clearance - near_field_slack)
                    : mas2027_nav_executor::effectiveClearanceThreshold(collision_dist);
  // 该点其实能过"近场规则"，被拒就只能是因为种子门用了更严的完整要求。
  const bool passes_near_rule = info.clearance > near_rule_required;

  if (!passes_near_rule) {
    // 连"近场规则"都过不了说明该点真没净空（含起点贴死），判 GEOMETRY 无误且不存在死分支。
    return "GEOMETRY";
  }
  // 该点满足近场规则却不满足更严要求，且种子门只在起点贴死时放宽 —— 两处近场门槛不一致。
  return "SEED_GATE_STRICTER";
}

void LocalPathProcessor::configure(double lookahead_dist, double max_vel,
                                   double max_acc, double traj_goal_tolerance,
                                   double collision_dist, double preferred_clearance,
                                   rclcpp::Logger logger) {
  lookahead_dist_ = lookahead_dist;
  max_vel_ = max_vel;
  max_acc_ = max_acc;
  traj_goal_tolerance_ = traj_goal_tolerance;
  collision_dist_ = std::max(0.0, collision_dist);
  preferred_clearance_ = std::max(fullClearanceRequirement(), preferred_clearance);
  logger_ = logger;
}

void LocalPathProcessor::updateLimits(double max_vel, double max_acc,
                                      double traj_goal_tolerance) {
  max_vel_ = max_vel;
  max_acc_ = max_acc;
  traj_goal_tolerance_ = traj_goal_tolerance;
}

void LocalPathProcessor::setNarrowPassage(bool enable, double minimum_clearance) {
  narrow_passage_enable_ =
    enable && std::isfinite(minimum_clearance) && minimum_clearance > 0.0;
  narrow_passage_clearance_ = narrow_passage_enable_ ? minimum_clearance : 0.0;
}

void LocalPathProcessor::setEscapeOptions(bool enable, double min_length,
                                          double buffer) {
  escape_enable_ = enable;
  escape_min_length_ = std::max(0.0, min_length);
  escape_buffer_ = std::max(0.0, buffer);
}

bool LocalPathProcessor::centerNarrowRoute(
    std::vector<Eigen::Vector3d> &path, const Eigen::Vector3d &start,
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    double start_clearance, bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
        terrain_segment_free) const {
  if (!narrow_passage_enable_ || narrow_passage_clearance_ <= 0.0 ||
      !query || !start.allFinite() || path.size() < 2U ||
      !std::isfinite(query->resolution()) || query->resolution() <= 0.0) {
    return false;
  }

  // SMAC vertices are checked at cell centres; their connecting chords can
  // run closer to one wall. Work at no more than one map cell per sample and
  // keep the measured robot position and local endpoint fixed.
  const double spacing = std::clamp(query->resolution(), 0.025, 0.05);
  std::vector<Eigen::Vector3d> samples{start};
  for (const auto &point : path) {
    if (!point.allFinite()) return false;
    const Eigen::Vector3d from = samples.back();
    const double length = (point - from).head<2>().norm();
    if (length <= 1.0e-6) continue;
    const int count = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int step = 1; step <= count; ++step) {
      samples.push_back(from + (static_cast<double>(step) / count) * (point - from));
    }
  }
  if (samples.size() < 3U) return false;

  constexpr double kMaximumShift = 0.10;
  constexpr double kShiftStep = 0.025;
  constexpr double kMinimumGain = 0.005;
  constexpr double kOffsetPenalty = 0.10;
  std::vector<double> shifts(samples.size(), 0.0);
  std::vector<Eigen::Vector2d> normals(samples.size(), Eigen::Vector2d::Zero());
  for (size_t i = 1U; i + 1U < samples.size(); ++i) {
    const Eigen::Vector2d delta = (samples[i + 1U] - samples[i - 1U]).head<2>();
    if (delta.squaredNorm() <= 1.0e-12) continue;
    const Eigen::Vector2d tangent = delta.normalized();
    const Eigen::Vector2d normal(-tangent.y(), tangent.x());
    normals[i] = normal;
    const auto center = query->query(samples[i]);
    if (!center.ok || !std::isfinite(center.distance) ||
        center.distance < narrow_passage_clearance_ - 0.08 ||
        center.distance >= fullClearanceRequirement() ||
        !mas2027_nav_executor::hasOpposingSideObstacles(
          *query, samples[i].head<2>(), tangent, center.distance)) {
      continue;
    }
    double best_score = center.distance;
    double best_shift = 0.0;
    for (int step = -4; step <= 4; ++step) {
      const double shift = step * kShiftStep;
      if (std::abs(shift) < 1.0e-9 || std::abs(shift) > kMaximumShift) continue;
      const Eigen::Vector2d moved = samples[i].head<2>() + shift * normal;
      unsigned int mx = 0U, my = 0U;
      if (!query->worldToMap(moved.x(), moved.y(), mx, my) || !query->isFree(mx, my)) {
        continue;
      }
      const auto clearance = query->query(Eigen::Vector3d(moved.x(), moved.y(), 0.0));
      if (!clearance.ok || !std::isfinite(clearance.distance)) continue;
      const double score = clearance.distance - kOffsetPenalty * std::abs(shift);
      if (score > best_score + kMinimumGain) {
        best_score = score;
        best_shift = shift;
      }
    }
    shifts[i] = best_shift;
  }

  // At a corridor exit the bilateral test stops as soon as the walls end.
  // Keep the last offset briefly in the open area so the route does not move
  // back toward a wall while it is still inside the narrow section.
  for (size_t i = 1U; i + 1U < samples.size(); ++i) {
    if (std::abs(shifts[i]) >= 1.0e-3 || std::abs(shifts[i - 1U]) < 1.0e-3) {
      continue;
    }
    const auto clearance = query->query(samples[i]);
    if (!clearance.ok || !std::isfinite(clearance.distance) ||
        clearance.distance < fullClearanceRequirement() + 0.10) {
      continue;
    }
    const double step = 0.5 * (samples[i] - samples[i - 1U]).head<2>().norm();
    shifts[i] = std::copysign(std::max(0.0, std::abs(shifts[i - 1U]) - step),
                              shifts[i - 1U]);
  }

  // Rate-limit the lateral offset in both directions. This eases onto the
  // ridge from the measured pose and returns to the unchanged local endpoint.
  for (size_t i = 1U; i + 1U < shifts.size(); ++i) {
    const double max_change = 0.5 * (samples[i] - samples[i - 1U]).head<2>().norm();
    shifts[i] = std::clamp(shifts[i], shifts[i - 1U] - max_change,
      shifts[i - 1U] + max_change);
  }
  for (size_t i = shifts.size() - 2U; i > 0U; --i) {
    const double max_change = 0.5 * (samples[i + 1U] - samples[i]).head<2>().norm();
    shifts[i] = std::clamp(shifts[i], shifts[i + 1U] - max_change,
      shifts[i + 1U] + max_change);
  }

  std::vector<Eigen::Vector3d> centered = samples;
  size_t moved_count = 0U;
  double maximum_shift = 0.0;
  for (size_t i = 1U; i + 1U < centered.size(); ++i) {
    if (std::abs(shifts[i]) < 1.0e-3) continue;
    centered[i].head<2>() += shifts[i] * normals[i];
    ++moved_count;
    maximum_shift = std::max(maximum_shift, std::abs(shifts[i]));
  }
  if (moved_count == 0U) return false;

  // Only altered segments need rechecking here. The existing dense-route
  // repair and final trajectory/command gates handle unrelated tight cells.
  for (size_t i = 1U; i < centered.size(); ++i) {
    if (std::abs(shifts[i - 1U]) < 1.0e-3 && std::abs(shifts[i]) < 1.0e-3) continue;
    SeedRejectInfo reject;
    if (!segmentClear(query, centered[i - 1U], centered[i], start,
          start_clearance, start_clearance_ok, terrain_segment_free, &reject,
          true, 0.0, narrow_passage_clearance_)) {
      RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
        "[MincoPlanner] Narrow centering candidate rejected at (%.2f, %.2f): "
        "clearance %.3f m required %.3f m; keeping the original route.",
        reject.point.x(), reject.point.y(), reject.clearance, reject.required);
      return false;
    }
  }
  path.assign(centered.begin() + 1U, centered.end());
  RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
    "[MincoPlanner] Centered %zu two-sided narrow-route samples by at most %.3f m "
    "before MINCO; trajectory and command clearance gates remain active.",
    moved_count, maximum_shift);
  return true;
}

LocalPathSeed LocalPathProcessor::buildSeed(
    const std::vector<geometry_msgs::msg::PoseStamped> &global_path,
    const geometry_msgs::msg::PoseStamped &current_pose,
    const PlannerModeContext &mode_context,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)>
        &terrain_segment_free) const {
  LocalPathSeed seed;
  if (global_path.empty()) {
    return seed;
  }

  Eigen::Vector3d global_goal(global_path.back().pose.position.x,
                              global_path.back().pose.position.y, 0.0);
  Eigen::Vector3d cur_pos(current_pose.pose.position.x,
                          current_pose.pose.position.y, 0.0);

  seed.dense_path = extractLocalPath(global_path, cur_pos);
  const bool clip_ok =
      clipLocalPathByRogBoundary(seed.dense_path, mode_context);
  if (!clip_ok || seed.dense_path.size() < 2U) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
                         "[MincoPlanner] Local seed path is outside ROGMap "
                         "boundary or too short after clipping.");
    return seed;
  }
  if (seed.dense_path.size() < 2U) {
    return seed;
  }

  const auto query = mode_context.dynamicQuery();
  if (!query || query->resolution() <= 0.0) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
                         "[MincoPlanner] ROGMap query is unavailable while "
                         "building the local seed.");
    return seed;
  }

  const auto start_query = query->query(cur_pos);
  const bool start_clearance_ok =
      start_query.ok && std::isfinite(start_query.distance);
  const double start_clearance =
      start_clearance_ok ? start_query.distance : 0.0;

  seed.used_narrow_centering = centerNarrowRoute(
    seed.dense_path, cur_pos, query, start_clearance, start_clearance_ok,
    terrain_segment_free);

  if (start_clearance_ok && start_clearance < 0.0) {
    unsigned int mx = 0, my = 0;
    const bool inside = query->worldToMap(cur_pos.x(), cur_pos.y(), mx, my);
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "[MincoPlanner] Negative start clearance: world=(%.3f,%.3f) clearance=%.3f "
      "inside_map=%s cell=(%u,%u) cost=%u snapshot=%llu projection=%llu mask=%llu field=%llu "
      "field_age_ms=%.1f; inspect live cloud and projection at this position",
      cur_pos.x(), cur_pos.y(), start_clearance, inside ? "true" : "false", mx, my,
      inside ? static_cast<unsigned int>(query->value(mx, my)) : 255U,
      static_cast<unsigned long long>(start_query.snapshot_sequence),
      static_cast<unsigned long long>(start_query.projection_sequence),
      static_cast<unsigned long long>(start_query.mask_sequence),
      static_cast<unsigned long long>(start_query.field_sequence), start_query.field_age_ms);
  }

  // The global search works on cell centers, but straight chords between its
  // waypoints can cut inside the narrow hard floor even when every waypoint
  // passes. Sample the whole reference before deciding whether to repair it.
  // If no such route exists, the trajectory/command gates still reject any
  // unsafe candidate.
  bool dense_below_narrow_floor = false;
  if (narrow_passage_enable_ && narrow_passage_clearance_ > 0.0 && start_clearance_ok) {
    Eigen::Vector3d previous = cur_pos;
    for (const auto &point : seed.dense_path) {
      SeedRejectInfo reject;
      if (!segmentClear(query, previous, point, cur_pos, start_clearance,
                        start_clearance_ok, terrain_segment_free, &reject,
                        true, 0.0, narrow_passage_clearance_)) {
        dense_below_narrow_floor = reject.valid && reject.clearance_only;
        break;
      }
      previous = point;
    }
  }
  if (dense_below_narrow_floor) {
    std::vector<Eigen::Vector3d> repaired;
    if (searchDynamicDetour(cur_pos, seed.dense_path.back(), query,
                            start_clearance, start_clearance_ok,
                            terrain_segment_free, repaired, nullptr, 0.0,
                            narrow_passage_clearance_)) {
      seed.dense_path = std::move(repaired);
      seed.used_narrow_clearance_repair = true;
      RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
        "[MincoPlanner] Repaired a dense reference below the %.3f m narrow floor "
        "with a floor-aware local route.", narrow_passage_clearance_);
    } else {
      RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
        "[MincoPlanner] No floor-aware route reaches the local endpoint at %.3f m; "
        "the trajectory gate will reject candidates below that floor.",
        narrow_passage_clearance_);
    }
  }

  // 安全检查必须包含机器人实测位置：折线起点取最近栅格顶点，不一定等于机器人当前位置。
  std::vector<Eigen::Vector3d> checked_path;
  checked_path.reserve(seed.dense_path.size() + 1U);
  checked_path.push_back(cur_pos);
  for (const auto &point : seed.dense_path) {
    if ((point - checked_path.back()).head<2>().norm() > 1e-6) {
      checked_path.push_back(point);
    }
  }

  // 四层兜底各自"死在哪一点"：带上这组量，一条日志即可区分净空否决与物理阻断。
  SeedRejectInfo dense_reject;
  SeedRejectInfo prefix_reject;
  SeedRejectInfo detour_reject;
  SeedRejectInfo escape_reject;
  const auto rejectText = [](const SeedRejectInfo &info) {
    if (!info.valid) {
      return std::string("n/a");
    }
    if (info.terrain_blocked) {
      return std::string("terrain");
    }
    char buffer[144];
    std::snprintf(buffer, sizeof(buffer),
                  "(%.2f,%.2f) clear=%.3f req=%.3f arc=%.2f%s%s", info.point.x(),
                  info.point.y(), info.clearance, info.required,
                  info.arc_from_start,
                  info.near_field_relaxed ? " nearfield" : "",
                  info.length_limited ? " len-limited" : "");
    return std::string(buffer);
  };

  if (!pathClear(checked_path, cur_pos, query, start_clearance,
                 start_clearance_ok, terrain_segment_free, &dense_reject)) {
    // 不管后续修复是否成功都要留下种子门的失败点：调用方与测试靠它区分净空否决与物理阻断。
    seed.dense_reject = dense_reject;
    std::vector<Eigen::Vector3d> repaired;
    if (searchDynamicDetour(cur_pos, seed.dense_path.back(), query,
                            start_clearance, start_clearance_ok,
                            terrain_segment_free, repaired, &detour_reject)) {
      seed.dense_path = std::move(repaired);
      seed.used_dynamic_detour = true;
      RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
                           "[MincoPlanner] Repaired a ROGMap-blocked global "
                           "segment with a local grid-search detour.");
    } else if (buildSafeStoppingPrefix(cur_pos, seed.dense_path, query,
                                       start_clearance, start_clearance_ok,
                                       terrain_segment_free, repaired,
                                       &prefix_reject)) {
      seed.dense_path = std::move(repaired);
      seed.stop_at_local_end = true;
      RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
                           "[MincoPlanner] No local route around the live "
                           "obstacle; planning a safe stopping prefix.");
    } else if (escape_enable_ &&
               buildEscapePrefix(cur_pos, seed.dense_path, query,
                                 start_clearance, start_clearance_ok,
                                 terrain_segment_free, repaired,
                                 &escape_reject)) {
      // 第四层兜底：障碍前不足 kMinimumStoppingPrefix + kStoppingBuffer = 0.45 m 时车已贴死，
      // 退化为长度不超过一个近场豁免半径的蠕行前缀，末速为零且不引入新的安全阈值。
      seed.dense_path = std::move(repaired);
      seed.stop_at_local_end = true;
      seed.used_escape_prefix = true;
      double escape_length = 0.0;
      for (size_t i = 1U; i < seed.dense_path.size(); ++i) {
        escape_length += (seed.dense_path[i] - seed.dense_path[i - 1U]).head<2>().norm();
      }
      RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
                           "[MincoPlanner] Live obstacle leaves no full stopping "
                           "prefix; escaping with a %.2f m creep prefix.",
                           escape_length);
    } else {
      RCLCPP_WARN_THROTTLE(
          logger_, *rclcpp::Clock::make_shared(), 2000,
          "[MincoPlanner] Live obstacle blocks the local route and leaves no "
          "safe stopping prefix. [start_clear=%.3f collision_dist=%.3f] "
          "dense_reject=%s detour_reject=%s prefix_reject=%s escape_reject=%s "
          "verdict=%s",
          start_clearance, collision_dist_, rejectText(dense_reject).c_str(),
          rejectText(detour_reject).c_str(), rejectText(prefix_reject).c_str(),
          rejectText(escape_reject).c_str(),
          classifySeedReject(dense_reject, start_clearance, collision_dist_,
                             kNearFieldSlack)
              .c_str());
      seed.dense_reject = dense_reject;
      seed.dense_path.clear();
      return seed;
    }
  } else if (dense_reject.valid && dense_reject.clearance_only) {
    // "贴墙但没堵死"时折线仍自由，种子照常交给 MINCO，仅留一条现场日志以便与真被挡住区分。
    seed.dense_reject = dense_reject;
    RCLCPP_INFO_THROTTLE(
        logger_, *rclcpp::Clock::make_shared(), 2000,
        "[MincoPlanner] Local seed is tight but not blocked "
        "(closest clear=%.3f req=%.3f at (%.2f,%.2f) arc=%.2f); handing it to "
        "MINCO, clearance is enforced on the trajectory instead.",
        dense_reject.clearance, dense_reject.required, dense_reject.point.x(),
        dense_reject.point.y(), dense_reject.arc_from_start);
  }

  seed.local_end_is_goal =
      (global_goal - seed.dense_path.back()).head<2>().norm() <=
      traj_goal_tolerance_;
  seed.sparse_waypoints = utils::getSparseWaypoints(
      seed.dense_path, max_vel_, max_acc_, seed.local_end_is_goal,
      [this, &mode_context, &terrain_segment_free, &cur_pos, start_clearance,
       start_clearance_ok](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        const bool enforce_sparse_clearance =
          narrow_passage_enable_ && narrow_passage_clearance_ > 0.0;
        return segmentClear(mode_context.sparsifyQuery(), a, b, cur_pos,
                            start_clearance, start_clearance_ok,
                            terrain_segment_free, nullptr,
                            enforce_sparse_clearance, 0.0,
                            enforce_sparse_clearance ?
                            narrow_passage_clearance_ : 0.0);
      });
  if (seed.sparse_waypoints.size() < 2U &&
      narrow_passage_enable_ && narrow_passage_clearance_ > 0.0) {
    // Enforcing the narrow floor on every sparse chord can reject even the
    // point-by-point expansion when the dense grid path itself grazes 0.235 m
    // between cell centers. That must not turn a usable seed into
    // LOCAL_SEED_INVALID: keep the floor as an optimization, then let the
    // hard trajectory/command gates remain the final authority.
    seed.sparse_waypoints = utils::getSparseWaypoints(
        seed.dense_path, max_vel_, max_acc_, seed.local_end_is_goal,
        [this, &mode_context, &terrain_segment_free, &cur_pos, start_clearance,
         start_clearance_ok](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
          return segmentClear(mode_context.sparsifyQuery(), a, b, cur_pos,
                              start_clearance, start_clearance_ok,
                              terrain_segment_free);
        });
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "[MincoPlanner] Narrow sparse-clearance simplification rejected the whole "
      "seed; falling back to the unconstrained sparse path. Trajectory/command "
      "clearance gates remain active.");
  }
  seed.valid = seed.sparse_waypoints.size() >= 2U &&
               pathClear(seed.sparse_waypoints, cur_pos, query, start_clearance,
                         start_clearance_ok, terrain_segment_free);
  if (clearance_detour_enabled_ && seed.valid && dense_reject.valid && dense_reject.clearance_only &&
      !seed.stop_at_local_end && preferred_clearance_ > fullClearanceRequirement()) {
    // A tight but free seed can pass the occupancy gate yet leave MINCO no
    // room to smooth. Try a wider grid route only as an improvement: if the
    // route or its sparse chords miss the soft clearance target, keep the
    // original seed and let the unchanged trajectory check decide.
    std::vector<Eigen::Vector3d> wider_path;
    if (searchDynamicDetour(cur_pos, seed.dense_path.back(), query,
                            start_clearance, start_clearance_ok,
                            terrain_segment_free, wider_path, nullptr,
                            preferred_clearance_)) {
      auto wider_sparse = utils::getSparseWaypoints(
        wider_path, max_vel_, max_acc_, seed.local_end_is_goal,
        [this, &mode_context, &terrain_segment_free, &cur_pos, start_clearance,
         start_clearance_ok](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
          return segmentClear(mode_context.sparsifyQuery(), a, b, cur_pos,
                              start_clearance, start_clearance_ok,
                              terrain_segment_free, nullptr, true,
                              preferred_clearance_);
        });
      if (wider_sparse.size() >= 2U && wider_sparse.size() <= 16U &&
          pathClear(wider_sparse, cur_pos, query, start_clearance,
                    start_clearance_ok, terrain_segment_free, nullptr,
                    preferred_clearance_)) {
        seed.dense_path = std::move(wider_path);
        seed.sparse_waypoints = std::move(wider_sparse);
        seed.used_dynamic_detour = true;
        seed.used_clearance_detour = true;
        RCLCPP_INFO_THROTTLE(
          logger_, *rclcpp::Clock::make_shared(), 2000,
          "[MincoPlanner] Replaced a tight local seed with a %.3f m clearance detour.",
          preferred_clearance_);
      }
    }
  }
  if (!seed.valid) {
    // 修复看似成功后种子仍可能不可用：sparse=0 表示稀疏化整体否决，sparse>=2 表示稀疏点没过复核。
    if (seed.used_dynamic_detour || seed.stop_at_local_end) {
      seed.repair_rejected = true;
      if (repair_reject_logged_ < kRepairRejectLogFirstN) {
        ++repair_reject_logged_;
        RCLCPP_WARN(logger_,
                    "[MincoPlanner] Repair rejected: seed invalid after repair "
                    "(dense=%zu sparse=%zu detour=%d stop_prefix=%d, logged %llu/%llu)",
                    seed.dense_path.size(), seed.sparse_waypoints.size(),
                    static_cast<int>(seed.used_dynamic_detour),
                    static_cast<int>(seed.stop_at_local_end),
                    static_cast<unsigned long long>(repair_reject_logged_),
                    static_cast<unsigned long long>(kRepairRejectLogFirstN));
      } else {
        RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
                             "[MincoPlanner] Repair rejected: seed invalid after "
                             "repair (dense=%zu sparse=%zu)",
                             seed.dense_path.size(),
                             seed.sparse_waypoints.size());
      }
    }
    seed.sparse_waypoints.clear();
  }
  return seed;
}

bool LocalPathProcessor::segmentClear(
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    const Eigen::Vector3d &from, const Eigen::Vector3d &to,
    const Eigen::Vector3d &planning_start, double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)>
        &terrain_segment_free,
    SeedRejectInfo *reject_info, bool enforce_clearance,
    double preferred_clearance, double clearance_override) const {
  // 只记整条路径上最早的失败点（reject_info 非空时写第一次）；例外的硬否决可覆盖"只差净空"。
  const auto note_reject = [reject_info, &planning_start](
      const Eigen::Vector3d &point, double clearance, double required,
      bool near_field_relaxed, bool terrain_blocked, bool clearance_only) {
    if (reject_info == nullptr) {
      return;
    }
    if (reject_info->valid && !(reject_info->clearance_only && !clearance_only)) {
      return;
    }
    reject_info->valid = true;
    reject_info->point = point;
    reject_info->clearance = clearance;
    reject_info->required = required;
    // 近场按"离规划起点的距离"划分，判读时需要该量判断此点是否落在近场内。
    reject_info->arc_from_start = (point - planning_start).head<2>().norm();
    reject_info->near_field_relaxed = near_field_relaxed;
    reject_info->terrain_blocked = terrain_blocked;
    reject_info->clearance_only = clearance_only;
    reject_info->length_limited = false;
  };
  if (!query || query->resolution() <= 0.0 || !from.allFinite() ||
      !to.allFinite()) {
    note_reject(from, 0.0, fullClearanceRequirement(), false, false, false);
    return false;
  }
  if (terrain_segment_free && !terrain_segment_free(from, to)) {
    note_reject(from, 0.0, fullClearanceRequirement(), false, true, false);
    return false;
  }
  const double length = (to - from).head<2>().norm();
  const double step = std::max(0.01, 0.5 * query->resolution());
  const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int i = 0; i <= samples; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(samples);
    const Eigen::Vector3d point = from + ratio * (to - from);
    unsigned int mx = 0U;
    unsigned int my = 0U;
    if (!query->worldToMap(point.x(), point.y(), mx, my) ||
        !query->isFree(mx, my)) {
      note_reject(point, 0.0, fullClearanceRequirement(), false, false, false);
      return false;
    }
    const auto clearance = query->query(point);
    if (!clearance.ok || !std::isfinite(clearance.distance)) {
      note_reject(point, 0.0, fullClearanceRequirement(), false, false, false);
      return false;
    }
    // 净空是**轨迹**的属性而非折线的属性：MINCO 的位置罚项会把轨迹推离障碍，用折线净空否决种子
    // 等于要求种子先满足轨迹指标，避障能力作废；故硬否决只保留占据 / 地形 / 查询无效。
    // 净空不足仅记 clearance_only 后继续采样，净空要求仍由发布前整条轨迹检查、
    // 执行前指令门两道硬门把关，此处没有放松。
    double required = clearance_override > 0.0 ?
      clearance_override : fullClearanceRequirement();
    bool relaxed = false;
    if (start_clearance_ok && start_clearance < collision_dist_ &&
        (point - planning_start).head<2>().norm() <= collision_dist_) {
      // 近场只要求“不比现在更差”，但不能比 override 地板更严；否则窄道内
      // 稀疏化会把起点附近的合法短段全部拒掉。
      required = std::min(required, std::max(0.0, start_clearance - kNearFieldSlack));
      relaxed = true;
    } else if (preferred_clearance > 0.0 &&
               (point - planning_start).head<2>().norm() > collision_dist_) {
      required = std::max(required, preferred_clearance);
    }
    if (clearance.distance + mas2027_nav_executor::kClearanceComparisonTolerance < required) {
      note_reject(point, clearance.distance, required, relaxed, false, true);
      // 停车/脱困前缀要在这里停下：前缀末端就是"车要停在哪"，它本身必须满足净空。
      if (enforce_clearance) {
        return false;
      }
    }
  }
  return true;
}

bool LocalPathProcessor::pathClear(
    const std::vector<Eigen::Vector3d> &path,
    const Eigen::Vector3d &planning_start,
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    double start_clearance, bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)>
        &terrain_segment_free,
    SeedRejectInfo *reject_info, double preferred_clearance,
    double hard_clearance_floor) const {
  if (path.size() < 2U) {
    return false;
  }
  for (size_t i = 1U; i < path.size(); ++i) {
    if (!segmentClear(query, path[i - 1U], path[i], planning_start,
                      start_clearance, start_clearance_ok, terrain_segment_free,
                      reject_info, preferred_clearance > 0.0 || hard_clearance_floor > 0.0,
                      preferred_clearance, hard_clearance_floor)) {
      return false;
    }
  }
  return true;
}

bool LocalPathProcessor::searchDynamicDetour(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    double start_clearance, bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)>
        &terrain_segment_free,
    std::vector<Eigen::Vector3d> &detour,
    SeedRejectInfo *reject_info, double preferred_clearance,
    double hard_clearance_floor) const {
  detour.clear();
  if (!query || query->sizeX() == 0U || query->sizeY() == 0U) {
    return false;
  }
  unsigned int sx = 0U, sy = 0U, gx = 0U, gy = 0U;
  if (!query->worldToMap(start.x(), start.y(), sx, sy) ||
      !query->worldToMap(goal.x(), goal.y(), gx, gy)) {
    return false;
  }

  const unsigned int width = query->sizeX();
  const unsigned int height = query->sizeY();
  const size_t cell_count = static_cast<size_t>(width) * height;
  const auto index = [width](unsigned int x, unsigned int y) {
    return static_cast<size_t>(y) * width + x;
  };
  const auto cellPoint = [&query](unsigned int x, unsigned int y) {
    double wx = 0.0;
    double wy = 0.0;
    query->mapToWorld(x, y, wx, wy);
    return Eigen::Vector3d(wx, wy, 0.0);
  };

  std::vector<int8_t> traversable(cell_count, -1);
  const auto cellTraversable = [&](unsigned int x, unsigned int y) {
    const size_t idx = index(x, y);
    if (traversable[idx] >= 0) {
      return traversable[idx] != 0;
    }
    const Eigen::Vector3d point = cellPoint(x, y);
    unsigned int mx = 0U, my = 0U;
    bool safe = query->worldToMap(point.x(), point.y(), mx, my) &&
                query->isFree(mx, my);
    if (safe) {
      const auto result = query->query(point);
      // 必须与种子门/轨迹门用同一有效阈值，否则绕行会拒绝轨迹门接受的窄处而误报"无路可绕"。
      double required = fullClearanceRequirement();
      if (hard_clearance_floor > 0.0) {
        // Narrow repair asks for the hard floor everywhere; the trajectory and
        // command gates still enforce that same floor point by point.
        required = hard_clearance_floor;
      } else if (start_clearance_ok && start_clearance < collision_dist_ &&
                 (point - start).head<2>().norm() <= collision_dist_) {
        required = std::max(0.0, start_clearance - kNearFieldSlack);
      } else if (preferred_clearance > 0.0 &&
                 (point - start).head<2>().norm() > collision_dist_) {
        required = std::max(required, preferred_clearance);
      }
      safe = result.ok && std::isfinite(result.distance) &&
             result.distance > required;
    }
    traversable[idx] = safe ? 1 : 0;
    return safe;
  };

  // 实测起点是权威位置：近场规则接受它时允许起点格，后续格仍须满足上面的净空判据。
  const auto start_result = query->query(start);
  if (!start_result.ok || !std::isfinite(start_result.distance)) {
    return false;
  }
  traversable[index(sx, sy)] = query->isFree(sx, sy) ? 1 : 0;
  if (!cellTraversable(sx, sy)) {
    return false;
  }
  if (!cellTraversable(gx, gy)) {
    // The local endpoint can lie away from its cell center. If that measured
    // point passes the same continuous gate, allow the target cell only as an
    // endpoint; incoming edges are checked against the actual goal below.
    if (!query->isFree(gx, gy) ||
        !segmentClear(query, goal, goal, start, start_clearance,
                      start_clearance_ok, terrain_segment_free, nullptr,
                      true, preferred_clearance, hard_clearance_floor)) {
      return false;
    }
    traversable[index(gx, gy)] = 1;
  }

  const size_t source = index(sx, sy);
  const size_t target = index(gx, gy);
  std::vector<double> g_score(cell_count,
                              std::numeric_limits<double>::infinity());
  std::vector<int> parent(cell_count, -1);
  std::vector<uint8_t> closed(cell_count, 0U);
  std::priority_queue<OpenCell, std::vector<OpenCell>, std::greater<OpenCell>>
      open;
  const auto heuristic = [gx, gy](unsigned int x, unsigned int y) {
    const unsigned int dx = x > gx ? x - gx : gx - x;
    const unsigned int dy = y > gy ? y - gy : gy - y;
    return static_cast<double>(std::max(dx, dy)) +
           0.4142135623730951 * static_cast<double>(std::min(dx, dy));
  };
  g_score[source] = 0.0;
  open.push(OpenCell{source, heuristic(sx, sy)});
  constexpr std::array<std::pair<int, int>, 8> neighbors{
      {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};

  size_t expanded = 0U;
  while (!open.empty()) {
    const size_t current = open.top().index;
    open.pop();
    if (closed[current]) {
      continue;
    }
    // The optional clearance improvement must not monopolize a replan cycle.
    // The original seed remains available if the preferred corridor is costly
    // or impossible to find; normal obstacle detours keep their old behavior.
    if ((preferred_clearance > 0.0 || hard_clearance_floor > 0.0) &&
        ++expanded > 8000U) {
      return false;
    }
    closed[current] = 1U;
    if (current == target) {
      break;
    }
    const unsigned int x = static_cast<unsigned int>(current % width);
    const unsigned int y = static_cast<unsigned int>(current / width);
    // The returned path replaces the endpoint cell centers with the measured
    // start and the requested local endpoint. Check those same connectors
    // during search, not only after selecting a route.
    const Eigen::Vector3d from = current == source ? start : cellPoint(x, y);
    for (const auto &[dx, dy] : neighbors) {
      const int nx_i = static_cast<int>(x) + dx;
      const int ny_i = static_cast<int>(y) + dy;
      if (nx_i < 0 || ny_i < 0 || nx_i >= static_cast<int>(width) ||
          ny_i >= static_cast<int>(height)) {
        continue;
      }
      const auto nx = static_cast<unsigned int>(nx_i);
      const auto ny = static_cast<unsigned int>(ny_i);
      if (!cellTraversable(nx, ny)) {
        continue;
      }
      if (dx != 0 && dy != 0 &&
          (!cellTraversable(static_cast<unsigned int>(static_cast<int>(x) + dx),
                            y) ||
           !cellTraversable(
               x, static_cast<unsigned int>(static_cast<int>(y) + dy)))) {
        continue;
      }
      const size_t next = index(nx, ny);
      const Eigen::Vector3d to = next == target ? goal : cellPoint(nx, ny);
      if (preferred_clearance > 0.0 || hard_clearance_floor > 0.0) {
        // Safe cell centers do not imply a safe chord between them. Reject a
        // low-clearance edge here so A* can explore another route instead of
        // returning the shortest route and discarding it at pathClear().
        if (!segmentClear(query, from, to, start, start_clearance,
                          start_clearance_ok, terrain_segment_free, nullptr,
                          true, preferred_clearance, hard_clearance_floor)) {
          continue;
        }
      } else if (terrain_segment_free && !terrain_segment_free(from, to)) {
        continue;
      }
      const double step_cost = dx != 0 && dy != 0 ? 1.4142135623730951 : 1.0;
      const double tentative = g_score[current] + step_cost;
      if (tentative >= g_score[next]) {
        continue;
      }
      g_score[next] = tentative;
      parent[next] = static_cast<int>(current);
      open.push(OpenCell{next, tentative + heuristic(nx, ny)});
    }
  }
  if (source == target) {
    detour = {start, goal};
    return pathClear(detour, start, query, start_clearance, start_clearance_ok,
                     terrain_segment_free, reject_info, preferred_clearance,
                     hard_clearance_floor);
  }
  if (source != target && parent[target] < 0) {
    return false;
  }

  std::vector<Eigen::Vector3d> reversed;
  for (size_t cursor = target;;) {
    reversed.push_back(cellPoint(static_cast<unsigned int>(cursor % width),
                                 static_cast<unsigned int>(cursor / width)));
    if (cursor == source) {
      break;
    }
    const int previous = parent[cursor];
    if (previous < 0) {
      return false;
    }
    cursor = static_cast<size_t>(previous);
  }
  detour.assign(reversed.rbegin(), reversed.rend());
  detour.front() = start;
  detour.back() = goal;
  return pathClear(detour, start, query, start_clearance, start_clearance_ok,
                   terrain_segment_free, reject_info, preferred_clearance,
                   hard_clearance_floor);
}

bool LocalPathProcessor::buildSafeStoppingPrefix(
    const Eigen::Vector3d &start, const std::vector<Eigen::Vector3d> &path,
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    double start_clearance, bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)>
        &terrain_segment_free,
    std::vector<Eigen::Vector3d> &prefix,
    SeedRejectInfo *reject_info) const {
  return buildStoppingPrefixCore(start, path, query, start_clearance,
                                 start_clearance_ok, terrain_segment_free,
                                 kStoppingBuffer, kMinimumStoppingPrefix,
                                 0.0, prefix, reject_info);
}

bool LocalPathProcessor::buildEscapePrefix(
    const Eigen::Vector3d &start, const std::vector<Eigen::Vector3d> &path,
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    double start_clearance, bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)>
        &terrain_segment_free,
    std::vector<Eigen::Vector3d> &prefix,
    SeedRejectInfo *reject_info) const {
  // 长度上限刻意取 collision_dist：整段落在近场豁免半径内，故不放宽安全阈值也不会被校验否掉。
  return buildStoppingPrefixCore(start, path, query, start_clearance,
                                 start_clearance_ok, terrain_segment_free,
                                 escape_buffer_, escape_min_length_,
                                 collision_dist_, prefix, reject_info);
}

bool LocalPathProcessor::buildStoppingPrefixCore(
    const Eigen::Vector3d &start, const std::vector<Eigen::Vector3d> &path,
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    double start_clearance, bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)>
        &terrain_segment_free,
    double buffer, double min_length, double max_length,
    std::vector<Eigen::Vector3d> &prefix,
    SeedRejectInfo *reject_info) const {
  prefix.clear();
  prefix.push_back(start);
  const double sample_step = std::max(0.02, 0.5 * query->resolution());
  std::vector<Eigen::Vector3d> samples;
  samples.push_back(start);
  Eigen::Vector3d previous = start;
  for (const auto &target : path) {
    const double length = (target - previous).head<2>().norm();
    if (length <= 1e-6) {
      previous = target;
      continue;
    }
    const int count =
        std::max(1, static_cast<int>(std::ceil(length / sample_step)));
    for (int i = 1; i <= count; ++i) {
      const Eigen::Vector3d point =
          previous + (static_cast<double>(i) / static_cast<double>(count)) *
                         (target - previous);
      if (!segmentClear(query, samples.back(), point, start, start_clearance,
                        start_clearance_ok, terrain_segment_free,
                        reject_info, /*enforce_clearance=*/true)) {
        double safe_length = 0.0;
        for (size_t k = 1U; k < samples.size(); ++k) {
          safe_length += (samples[k] - samples[k - 1U]).head<2>().norm();
        }
        double target_length = safe_length - buffer;
        if (max_length > 0.0) {
          target_length = std::min(target_length, max_length);
        }
        if (target_length < min_length) {
          // 否决的定义是"安全段太短"：标出该点，判读时才能与"净空不够"区分开。
          if (reject_info != nullptr) {
            reject_info->length_limited = true;
          }
          prefix.clear();
          return false;
        }
        prefix.clear();
        prefix.push_back(start);
        double accumulated = 0.0;
        for (size_t k = 1U; k < samples.size(); ++k) {
          const double segment =
              (samples[k] - samples[k - 1U]).head<2>().norm();
          if (accumulated + segment >= target_length) {
            const double ratio =
                segment > 1e-9 ? (target_length - accumulated) / segment : 0.0;
            prefix.push_back(samples[k - 1U] +
                             ratio * (samples[k] - samples[k - 1U]));
            return pathClear(prefix, start, query, start_clearance,
                             start_clearance_ok, terrain_segment_free,
                             reject_info);
          }
          prefix.push_back(samples[k]);
          accumulated += segment;
        }
        return prefix.size() >= 2U;
      }
      samples.push_back(point);
    }
    previous = target;
  }
  prefix.clear();
  return false;
}

std::vector<Eigen::Vector3d> LocalPathProcessor::extractLocalPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & global_path, const Eigen::Vector3d & cur_pos) const
{
  std::vector<Eigen::Vector3d> local_segment;
  if (global_path.empty()) {
    return local_segment;
  }

  size_t start_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_path.size(); ++i) {
    const auto & pt = global_path[i].pose.position;
    double dist_sq =
      (cur_pos.x() - pt.x) * (cur_pos.x() - pt.x) + (cur_pos.y() - pt.y) * (cur_pos.y() - pt.y);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      start_idx = i;
    }
  }

  // Overshoot / already at last vertex: nearest pose is the goal. Keep the
  // last segment so the seed is never a single point (buildSeed needs >= 2).
  if (start_idx + 1U >= global_path.size() && global_path.size() >= 2U) {
    start_idx = global_path.size() - 2U;
  }

  double accum_dist = 0.0;
  local_segment.push_back(
    Eigen::Vector3d(global_path[start_idx].pose.position.x, global_path[start_idx].pose.position.y, 0.0));

  for (size_t i = start_idx + 1; i < global_path.size(); ++i) {
    const auto & p1 = global_path[i - 1].pose.position;
    const auto & p2 = global_path[i].pose.position;
    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    accum_dist += dist;

    local_segment.push_back(Eigen::Vector3d(p2.x, p2.y, 0.0));

    if (accum_dist >= lookahead_dist_) {
      break;
    }
  }

  return local_segment;
}

bool LocalPathProcessor::clipLocalPathByRogBoundary(
  std::vector<Eigen::Vector3d> & path, const PlannerModeContext & mode_context) const
{
  if (path.empty()) {
    return false;
  }

  const auto query = mode_context.dynamicQuery();
  if (!query) {
    return false;
  }

  const double margin = mode_context.explorationBoundaryMargin();
  const int margin_cells =
    std::max(0, static_cast<int>(std::ceil(margin / std::max(1e-6, query->resolution()))));
  const int max_x = static_cast<int>(query->sizeX());
  const int max_y = static_cast<int>(query->sizeY());
  if (max_x <= 0 || max_y <= 0) {
    return false;
  }

  const double sample_step = mode_context.explorationBoundarySampleStep();
  const double step = std::max(query->resolution(), std::max(1e-3, sample_step));

  auto inside_boundary = [&query, margin_cells, max_x, max_y](const Eigen::Vector3d & p) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!query->worldToMap(p.x(), p.y(), mx, my)) {
      return false;
    }
    const int ix = static_cast<int>(mx);
    const int iy = static_cast<int>(my);
    return ix >= margin_cells && iy >= margin_cells && ix < max_x - margin_cells &&
           iy < max_y - margin_cells;
  };

  std::vector<Eigen::Vector3d> clipped;
  clipped.reserve(path.size());

  // Robot often sits on the ROG margin at a cruise-window handoff. Drop
  // leading vertices that are outside instead of rejecting the whole seed.
  size_t i0 = 0;
  while (i0 < path.size() && !inside_boundary(path[i0])) {
    ++i0;
  }
  auto clamp_inside = [&query, margin_cells, max_x, max_y](const Eigen::Vector3d & p) {
    const double res = std::max(1e-6, query->resolution());
    const int ix_lo = margin_cells;
    const int iy_lo = margin_cells;
    const int ix_hi = std::max(ix_lo, max_x - margin_cells - 1);
    const int iy_hi = std::max(iy_lo, max_y - margin_cells - 1);
    unsigned int mx = 0;
    unsigned int my = 0;
    int ix = ix_lo;
    int iy = iy_lo;
    if (query->worldToMap(p.x(), p.y(), mx, my)) {
      ix = std::clamp(static_cast<int>(mx), ix_lo, ix_hi);
      iy = std::clamp(static_cast<int>(my), iy_lo, iy_hi);
    } else {
      const double min_wx = query->originX() + (static_cast<double>(ix_lo) + 0.5) * res;
      const double min_wy = query->originY() + (static_cast<double>(iy_lo) + 0.5) * res;
      const double max_wx = query->originX() + (static_cast<double>(ix_hi) + 0.5) * res;
      const double max_wy = query->originY() + (static_cast<double>(iy_hi) + 0.5) * res;
      const double cx = std::clamp(p.x(), min_wx, max_wx);
      const double cy = std::clamp(p.y(), min_wy, max_wy);
      if (query->worldToMap(cx, cy, mx, my)) {
        ix = std::clamp(static_cast<int>(mx), ix_lo, ix_hi);
        iy = std::clamp(static_cast<int>(my), iy_lo, iy_hi);
      }
    }
    double wx = 0.0;
    double wy = 0.0;
    query->mapToWorld(static_cast<unsigned int>(ix), static_cast<unsigned int>(iy), wx, wy);
    return Eigen::Vector3d(wx, wy, 0.0);
  };

  // Entire seed outside the inner ROG box (window-edge handoff). Clamp onto
  // the inner AABB so ReplanLocal still has a 2-point seed instead of failing.
  if (i0 >= path.size()) {
    std::vector<Eigen::Vector3d> clamped;
    clamped.push_back(clamp_inside(path.front()));
    Eigen::Vector3d second =
      path.size() >= 2U ? clamp_inside(path[1]) : clamp_inside(path.front());
    if ((second - clamped.front()).head<2>().norm() < 0.05) {
      Eigen::Vector3d dir = path.back() - path.front();
      if (dir.head<2>().norm() >= 1e-6) {
        dir.head<2>().normalize();
        second = clamp_inside(
          clamped.front() + Eigen::Vector3d(dir.x(), dir.y(), 0.0) * 0.3);
      }
    }
    if ((second - clamped.front()).head<2>().norm() < 0.05) {
      path.clear();
      RCLCPP_WARN_THROTTLE(logger_,
        *rclcpp::Clock::make_shared(),
        2000,
        "[MincoPlanner] Local seed path outside ROGMap; clamp degenerated, reject seed.");
      return false;
    }
    clamped.push_back(second);
    path.swap(clamped);
    RCLCPP_WARN_THROTTLE(logger_,
      *rclcpp::Clock::make_shared(),
      2000,
      "[MincoPlanner] Local seed path outside ROGMap; clamped onto inner boundary.");
    return true;
  }

  for (size_t i = i0; i < path.size(); ++i) {
    if (!inside_boundary(path[i])) {
      if (clipped.empty()) {
        path.clear();
        return false;
      }
      const Eigen::Vector3d a = clipped.back();
      const Eigen::Vector3d b = path[i];
      const Eigen::Vector3d delta = b - a;
      const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() / step)));
      Eigen::Vector3d last_inside = a;
      for (int s = 1; s <= samples; ++s) {
        const double ratio = static_cast<double>(s) / static_cast<double>(samples);
        const Eigen::Vector3d p = a + ratio * delta;
        if (!inside_boundary(p)) {
          break;
        }
        last_inside = p;
      }
      if ((last_inside - a).norm() > 1e-3) {
        clipped.push_back(last_inside);
      }
      break;
    }
    if (i > i0) {
      const Eigen::Vector3d delta = path[i] - path[i - 1U];
      const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() / step)));
      bool left_boundary = false;
      for (int s = 1; s <= samples; ++s) {
        const double ratio = static_cast<double>(s) / static_cast<double>(samples);
        if (!inside_boundary(path[i - 1U] + ratio * delta)) {
          left_boundary = true;
          break;
        }
      }
      if (left_boundary) {
        path.swap(clipped);
        return path.size() >= 2U;
      }
    }
    clipped.push_back(path[i]);
  }

  path.swap(clipped);
  return path.size() >= 2U;
}

}  // namespace minco_planner
