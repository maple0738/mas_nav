#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mas2027_decision/point_table.hpp"
#include "mas2027_decision/types.hpp"

namespace mas2027_decision {

// 假决策测试节点：不接裁判数据，按固定脚本跑一遍完整流程，用来验证
// "决策发目标 -> 导航执行 -> 到达 -> 继续下一个点" 这条链路。
//
// 脚本（时间轴）：
//   1. 启动后原地等 start_delay_s（默认 5 s），不发目标；
//   2. 依次前往 patrol_points 里的点（默认 a1/a2/a3）；
//   3. 每到一个点，驻留 dwell_s（默认 5 s），然后去下一个；
//   4. 走完最后一个点回到 a1，无限循环；
//   5. 任意时刻 hp < heal_enter（默认 0.30）就去 home；
//   6. 在 home 等 hp >= heal_exit（默认 0.60），然后回到当前巡逻点继续。
//
// 到达判定不依赖导航反馈（nav_executor 没有任务状态话题），而是直接看
// 当前位姿与目标点的距离，小于 arrive_tolerance_m 就算到达。
class TestDecisionNode : public rclcpp::Node {
 public:
  // 构造：声明参数 -> 建 TF 监听 -> 订阅里程计 -> 读点表 -> 启动定时器。
  TestDecisionNode() : Node("mas2027_decision_test") {
    declareParameters();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto goal_topic = get_parameter("goal_topic").as_string();
    const auto odom_topic = get_parameter("odom_topic").as_string();
    const double tick_s = std::max(0.02, get_parameter("tick_period_s").as_double());

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("debug/test_state", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_x_ = msg->pose.pose.position.x;
          odom_y_ = msg->pose.pose.position.y;
          have_odom_ = true;
        });

    points_ready_ = loadPoints();
    start_s_ = now().seconds();

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(tick_s)),
        [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
                "test decision up: enabled=%s points=%s goal_frame=%s goal_topic=%s",
                enabled() ? "true" : "false",
                points_ready_ ? "loaded" : "missing",
                get_parameter("goal_frame").as_string().c_str(),
                goal_topic.c_str());
  }

 private:
  // 脚本阶段。
  //   WAIT_START: 启动后的原地等待，不发目标
  //   GO:         正在前往当前巡逻点
  //   DWELL:      已到达当前巡逻点，驻留 dwell_s
  //   HEAL:       低血返回 home，直到 hp 恢复到 heal_exit 以上
  enum class Phase { WAIT_START, GO, DWELL, HEAL };

  // 声明全部参数。默认值刻意保守（enabled=false），只有在测试 launch 里才打开。
  void declareParameters() {
    // ---- 运行与话题 ----
    declare_parameter("enabled", false);            // 安全总开关：false 时不发目标
    declare_parameter("tick_period_s", 0.1);        // 控制周期，秒
    declare_parameter("odom_topic", std::string("/Odometry"));   // 里程计话题
    declare_parameter("odom_frame", std::string("odom"));        // 里程计坐标系
    declare_parameter("goal_frame", std::string("map"));         // 目标坐标系：map 或 odom
    declare_parameter("goal_topic", std::string("/goal_pose"));  // 目标发布话题
    declare_parameter("points_file", std::string(""));           // 测试点表 YAML 路径

    // ---- 脚本参数 ----
    declare_parameter("home_name", std::string("home"));  // 回血点在图里的名字
    declare_parameter("patrol_points", std::vector<std::string>{"a1", "a2", "a3"});
                                                          // 巡逻顺序（按 YAML 里的名字）
    declare_parameter("start_delay_s", 5.0);              // 启动后原地等待秒数
    declare_parameter("dwell_s", 5.0);                    // 每个点的驻留秒数
    declare_parameter("arrive_tolerance_m", 0.4);         // 距目标多少米算到达
    declare_parameter("arrive_timeout_s", 60.0);          // 单点超时后跳过，0 表示不启用
    declare_parameter("heal_enter", 0.30);                // 血量低于它回 home
    declare_parameter("heal_exit", 0.60);                 // 血量高于它恢复巡逻
    declare_parameter("goal_dedupe_m", 0.30);             // 目标移动多少米才重发
    declare_parameter("goal_republish_s", 3.0);           // 兜底重发周期，秒

    // ---- 调试注入：用 ros2 param set /mas2027_decision_test debug_hp_ratio 0.2 ----
    declare_parameter("debug_hp_ratio", 1.0);             // 血量比例 0..1
  }

  // 读安全总开关。每帧读一次，方便运行时用 ros2 param set 打开/关闭。
  bool enabled() { return get_parameter("enabled").as_bool(); }

  // 从 points_file 读入所有命名点，并挑出 home 和巡逻点。
  // 返回 false 表示 home 缺失、巡逻点全部缺失或文件读不了，此时 tick() 不动作。
  bool loadPoints() {
    const std::string path = get_parameter("points_file").as_string();
    std::string error;
    std::map<std::string, Waypoint> points;
    if (!loadNamedPoints(path, &points, &error)) {
      RCLCPP_WARN(get_logger(), "points not loaded: %s", error.c_str());
      return false;
    }

    const std::string home_name = get_parameter("home_name").as_string();
    const auto home_it = points.find(home_name);
    if (home_it == points.end()) {
      RCLCPP_WARN(get_logger(), "home point '%s' missing in %s",
                  home_name.c_str(), path.c_str());
      return false;
    }
    home_ = home_it->second;

    std::vector<std::string> kept_names;
    std::vector<Waypoint> kept_points;
    for (const auto& name : get_parameter("patrol_points").as_string_array()) {
      const auto it = points.find(name);
      if (it == points.end()) {
        RCLCPP_WARN(get_logger(), "patrol point '%s' missing, skipped", name.c_str());
        continue;
      }
      kept_names.push_back(name);
      kept_points.push_back(it->second);
    }
    patrol_names_ = std::move(kept_names);
    patrol_ = std::move(kept_points);

    if (patrol_.empty()) {
      RCLCPP_WARN(get_logger(), "no patrol point available in %s", path.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "loaded %zu points, patrol: %s", points.size(),
                joinNames().c_str());
    return true;
  }

  // 把巡逻点名字拼成 "a1 -> a2 -> a3"，只用于日志。
  std::string joinNames() const {
    std::string out;
    for (const auto& name : patrol_names_) {
      if (!out.empty()) {
        out += " -> ";
      }
      out += name;
    }
    return out;
  }

  // 把里程计位姿换算到 goal_frame（通常是 map）。
  // 返回 false 表示还拿不到位姿（没收到里程计或 TF 不可用），本帧跳过。
  bool updatePose() {
    if (!have_odom_) {
      return false;
    }
    const std::string goal_frame = get_parameter("goal_frame").as_string();
    const std::string odom_frame = get_parameter("odom_frame").as_string();

    if (goal_frame == odom_frame) {
      pose_x_ = odom_x_;
      pose_y_ = odom_y_;
      return true;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(goal_frame, odom_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      return false;
    }

    const double yaw = tf2::getYaw(tf.transform.rotation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    pose_x_ = c * odom_x_ - s * odom_y_ + tf.transform.translation.x;
    pose_y_ = s * odom_x_ + c * odom_y_ + tf.transform.translation.y;
    return true;
  }

  // 当前位姿到目标点的平面距离，米。
  double distanceTo(const Waypoint& wp) const {
    return std::hypot(pose_x_ - wp.x, pose_y_ - wp.y);
  }

  // 当前要去的巡逻点。
  const Waypoint& currentPatrol() const { return patrol_[index_]; }

  // 切换到下一个巡逻点（循环）。
  void advancePatrol() { index_ = (index_ + 1) % patrol_.size(); }

  // 切阶段并打日志，避免每帧刷屏。
  void setPhase(Phase phase, const char* reason) {
    if (phase_ == phase) {
      return;
    }
    phase_ = phase;
    RCLCPP_INFO(get_logger(), "phase -> %s (%s)", phaseName(phase), reason);
  }

  // 阶段名，用于日志和 debug/test_state 话题。
  static const char* phaseName(Phase phase) {
    switch (phase) {
      case Phase::WAIT_START:
        return "WAIT_START";
      case Phase::GO:
        return "GO";
      case Phase::DWELL:
        return "DWELL";
      case Phase::HEAL:
        return "HEAL";
    }
    return "UNKNOWN";
  }

  // 脚本主循环（默认 10 Hz）。
  //   低血优先于一切；其余情况按 WAIT_START -> GO -> DWELL 推进。
  void tick() {
    if (!enabled() || !points_ready_) {
      return;
    }
    if (!updatePose()) {
      return;
    }
    const double t = now().seconds();
    const double hp = get_parameter("debug_hp_ratio").as_double();
    const double heal_enter = get_parameter("heal_enter").as_double();
    const double heal_exit = get_parameter("heal_exit").as_double();

    std_msgs::msg::String state_msg;
    state_msg.data = phaseName(phase_);
    state_pub_->publish(state_msg);

    // 低血：立刻回 home，直到血量恢复。
    if (phase_ != Phase::HEAL && hp < heal_enter) {
      setPhase(Phase::HEAL, "hp low, go home");
      RCLCPP_WARN(get_logger(), "hp %.2f < %.2f -> home", hp, heal_enter);
      publishGoal(home_, true);
      return;
    }

    switch (phase_) {
      case Phase::HEAL:
        if (hp >= heal_exit) {
          setPhase(Phase::GO, "hp recovered, resume patrol");
          RCLCPP_INFO(get_logger(), "hp %.2f >= %.2f -> %s", hp, heal_exit,
                      patrol_names_[index_].c_str());
          go_s_ = t;
          publishGoal(currentPatrol(), true);
        } else {
          // 还没恢复：只在需要时重发 home，避免反复打断导航。
          publishGoal(home_, false);
        }
        break;

      case Phase::WAIT_START:
        if (t - start_s_ >= get_parameter("start_delay_s").as_double()) {
          setPhase(Phase::GO, "start delay done");
          RCLCPP_INFO(get_logger(), "go to %s", patrol_names_[index_].c_str());
          go_s_ = t;
          publishGoal(currentPatrol(), true);
        }
        break;

      case Phase::GO: {
        const double tol = get_parameter("arrive_tolerance_m").as_double();
        const double timeout = get_parameter("arrive_timeout_s").as_double();
        if (distanceTo(currentPatrol()) <= tol) {
          setPhase(Phase::DWELL, "arrived");
          phase_s_ = t;
          RCLCPP_INFO(get_logger(), "arrived %s (dist %.2f m), dwell %.1f s",
                      patrol_names_[index_].c_str(), distanceTo(currentPatrol()),
                      get_parameter("dwell_s").as_double());
        } else if (timeout > 0.0 && t - go_s_ >= timeout) {
          RCLCPP_WARN(get_logger(), "timeout reaching %s (%.0f s), skip",
                      patrol_names_[index_].c_str(), timeout);
          advancePatrol();
          go_s_ = t;
          publishGoal(currentPatrol(), true);
        } else {
          publishGoal(currentPatrol(), false);
        }
        break;
      }

      case Phase::DWELL:
        if (t - phase_s_ >= get_parameter("dwell_s").as_double()) {
          advancePatrol();
          setPhase(Phase::GO, "dwell done");
          RCLCPP_INFO(get_logger(), "go to %s", patrol_names_[index_].c_str());
          go_s_ = t;
          publishGoal(currentPatrol(), true);
        }
        break;
    }
  }

  // 发布目标。force=false 时按 goal_dedupe_m / goal_republish_s 去重，
  // 避免每帧都提交目标（nav_executor 每次重新接受目标会清空运动许可，造成顿挫）。
  void publishGoal(const Waypoint& wp, bool force) {
    const double t = now().seconds();
    const double dedupe_m = get_parameter("goal_dedupe_m").as_double();
    const double republish_s = get_parameter("goal_republish_s").as_double();
    const double moved = std::hypot(wp.x - last_goal_.x, wp.y - last_goal_.y);
    if (!force && has_published_ && moved <= dedupe_m &&
        t - last_publish_s_ <= republish_s) {
      return;
    }

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = get_parameter("goal_frame").as_string();
    msg.pose.position.x = wp.x;
    msg.pose.position.y = wp.y;
    msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, wp.yaw);
    msg.pose.orientation = tf2::toMsg(q);

    goal_pub_->publish(msg);
    last_goal_ = wp;
    last_publish_s_ = t;
    has_published_ = true;
    RCLCPP_INFO(get_logger(), "goal -> (%.2f, %.2f, %.2f)", wp.x, wp.y, wp.yaw);
  }

  Waypoint home_{};                         // 回血点
  std::vector<std::string> patrol_names_;   // 实际可用的巡逻点名字（顺序同下）
  std::vector<Waypoint> patrol_;            // 实际可用的巡逻点坐标
  bool points_ready_{false};                // 点表是否可用

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                 // TF 查询缓冲
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;    // 填充缓冲
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;  // /goal_pose
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;  // debug/test_state
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  // 里程计
  rclcpp::TimerBase::SharedPtr timer_;                         // 控制定时器

  double odom_x_{0.0};    // 里程计系下的 x，米
  double odom_y_{0.0};    // 里程计系下的 y，米
  double pose_x_{0.0};    // goal_frame 下的 x，米
  double pose_y_{0.0};    // goal_frame 下的 y，米
  bool have_odom_{false}; // 是否收到过至少一帧里程计

  Phase phase_{Phase::WAIT_START};  // 当前脚本阶段
  std::size_t index_{0};            // 当前巡逻点下标
  double start_s_{0.0};             // 节点启动时间，秒
  double go_s_{0.0};                // 本次出发时间，秒
  double phase_s_{0.0};             // 进入 DWELL 的时间，秒

  Waypoint last_goal_{};            // 上次发布的目标
  double last_publish_s_{0.0};      // 上次发布目标的时间
  bool has_published_{false};       // 是否发布过目标
};

}  // namespace mas2027_decision

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mas2027_decision::TestDecisionNode>());
  rclcpp::shutdown();
  return 0;
}
