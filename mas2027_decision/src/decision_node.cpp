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
#include <memory>
#include <string>

#include "mas2027_decision/decision_core.hpp"
#include "mas2027_decision/point_table.hpp"
#include "mas2027_decision/types.hpp"

namespace mas2027_decision {

class DecisionNode : public rclcpp::Node {
 public:
  // 构造：声明参数 -> 建 TF 监听 -> 订阅里程计 -> 加载点表 -> 启动决策定时器。
  DecisionNode() : Node("mas2027_decision") {
    declareParameters();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // 独立监听节点，避免在构造函数里传 this 给模板 NodeT。
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto goal_topic = get_parameter("goal_topic").as_string();
    const auto odom_topic = get_parameter("odom_topic").as_string();
    const double tick_s = std::max(0.02, get_parameter("tick_period_s").as_double());

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
    intent_pub_ = create_publisher<std_msgs::msg::String>("debug/decision_intent", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_x_ = msg->pose.pose.position.x;
          odom_y_ = msg->pose.pose.position.y;
          odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
          have_odom_ = true;
        });

    reloadPoints();

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(tick_s)),
        [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
                "decision node up: enabled=%s goal_frame=%s goal_topic=%s",
                enabled() ? "true" : "false",
                get_parameter("goal_frame").as_string().c_str(),
                goal_topic.c_str());
  }

 private:
  // 声明全部 ROS 参数。默认值与 config/decision.yaml 一致，
  // 这样没有参数文件时节点也能启动。
  void declareParameters() {
    // ---- 运行与话题 ----
    declare_parameter("enabled", false);            // 安全总开关：false 时不发布目标
    declare_parameter("tick_period_s", 0.1);        // 决策周期，秒
    declare_parameter("odom_topic", std::string("/Odometry"));   // 里程计话题
    declare_parameter("odom_frame", std::string("odom"));        // 里程计坐标系
    declare_parameter("goal_frame", std::string("map"));         // 目标坐标系：map 或 odom
    declare_parameter("goal_topic", std::string("/goal_pose"));  // 目标发布话题
    declare_parameter("points_file", std::string(""));           // 点表 YAML 路径
    declare_parameter("mirror_points", false);      // 点表是否按场地中线镜像（红蓝方）

    // ---- 决策阈值（详细说明见 docs/tuning.md）----
    declare_parameter("heal_start_s", 60.0);        // 开局多少秒内不回血
    declare_parameter("end_rush_s", 300.0);         // 多少秒后进入末段冲锋
    declare_parameter("heal_enter", 0.30);          // 血量比例低于它开始回血
    declare_parameter("heal_exit", 0.60);           // 回血到它才退出
    declare_parameter("end_rush_hp", 0.50);         // 末段冲锋的血量门槛
    declare_parameter("low_bullet", 20);            // 弹药低于它去回血点
    declare_parameter("base_attack_drop", 300.0);   // 基地一次掉血多少算被攻击
    declare_parameter("base_clear_s", 5.0);         // 守家事件保持秒数
    declare_parameter("patrol_period_s", 20.0);     // 两个巡逻点之间的驻留时间
    declare_parameter("goal_dedupe_m", 0.30);       // 目标移动多少米才重发
    declare_parameter("goal_republish_s", 3.0);     // 兜底重发周期，秒

    // ---- 调试注入：裁判数据接通前用 ros2 param set 手动改 ----
    declare_parameter("debug_game_time_s", -1.0);   // 比赛时间，-1 表示未知
    declare_parameter("debug_hp_ratio", 1.0);       // 血量比例 0..1
    declare_parameter("debug_bullets", 100);        // 剩余弹药
    declare_parameter("debug_ally_base_hp", -1.0);  // 己方基地血量，-1 表示未知
    declare_parameter("debug_ally_outpost_hp", -1.0);   // 己方前哨站血量
    declare_parameter("debug_enemy_outpost_active", false);  // 敌方前哨站是否激活
  }

  // 读安全总开关。每帧读一次，方便运行时用 ros2 param set 打开/关闭。
  bool enabled() { return get_parameter("enabled").as_bool(); }

  // 加载点表到 params_.points，并按 mirror_points 决定是否镜像。
  // 失败时 points_ready_ 保持 false，tick() 不会发目标。
  void reloadPoints() {
    std::string error;
    PointTable table;
    if (!loadPointTable(get_parameter("points_file").as_string(), &table, &error)) {
      RCLCPP_WARN(get_logger(), "points not loaded: %s", error.c_str());
      points_ready_ = false;
      return;
    }
    if (get_parameter("mirror_points").as_bool()) {
      table = mirrorPointTable(table);
    }
    params_.points = table;
    points_ready_ = true;
    core_.setParams(params_);
  }

  // 把 ROS 参数同步到 params_ 并交给 DecisionCore。每帧调用，
  // 所以线上改阈值立即生效，不需要重启节点。点表不在这里刷新。
  void updateParams() {
    params_.heal_start_s = get_parameter("heal_start_s").as_double();
    params_.end_rush_s = get_parameter("end_rush_s").as_double();
    params_.heal_enter = get_parameter("heal_enter").as_double();
    params_.heal_exit = get_parameter("heal_exit").as_double();
    params_.end_rush_hp = get_parameter("end_rush_hp").as_double();
    params_.low_bullet = static_cast<int>(get_parameter("low_bullet").as_int());
    params_.base_attack_drop = get_parameter("base_attack_drop").as_double();
    params_.base_clear_s = get_parameter("base_clear_s").as_double();
    params_.patrol_period_s = get_parameter("patrol_period_s").as_double();
    params_.goal_dedupe_m = get_parameter("goal_dedupe_m").as_double();
    params_.goal_republish_s = get_parameter("goal_republish_s").as_double();
    core_.setParams(params_);
  }

  // 把里程计位姿换算到 goal_frame（通常是 map）。
  // 同一坐标系时直接拷贝；不同坐标系时用 TF 的 goal_frame -> odom_frame 做变换。
  // obs: 输出参数，写入 obs->pos；返回 false 表示还拿不到位姿，本帧跳过。
  bool updatePose(Observation* obs) {
    if (!have_odom_) {
      return false;
    }
    const std::string goal_frame = get_parameter("goal_frame").as_string();
    const std::string odom_frame = get_parameter("odom_frame").as_string();

    if (goal_frame == odom_frame) {
      obs->pos = {odom_x_, odom_y_};
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
    obs->pos = {c * odom_x_ - s * odom_y_ + tf.transform.translation.x,
                s * odom_x_ + c * odom_y_ + tf.transform.translation.y};
    return true;
  }

  // 决策主循环（默认 10 Hz）：
  //   1. 未开启或点表没加载时直接返回；
  //   2. 组装 Observation（当前来自 debug_* 参数，裁判数据接通后改为订阅）；
  //   3. 调 select() 选意图、step() 转目标；
  //   4. 发布意图调试话题；
  //   5. 目标去重后发布 /goal_pose。
  void tick() {
    if (!enabled() || !points_ready_) {
      return;
    }
    updateParams();

    Observation obs;
    obs.now_s = now().seconds();
    if (!updatePose(&obs)) {
      return;
    }
    obs.pose_valid = true;
    obs.game_time_s = get_parameter("debug_game_time_s").as_double();
    obs.hp_ratio = get_parameter("debug_hp_ratio").as_double();
    obs.bullets = static_cast<int>(get_parameter("debug_bullets").as_int());
    obs.ally_base_hp = get_parameter("debug_ally_base_hp").as_double();
    obs.ally_outpost_hp = get_parameter("debug_ally_outpost_hp").as_double();
    obs.enemy_outpost_active =
        get_parameter("debug_enemy_outpost_active").as_bool();
    // TODO(referee): 裁判数据接通后由订阅回调填充 hp/bullets/base_hp/game_time，
    //                并置 referee_valid=true；debug_* 参数退化为初始值。

    const Intent intent = core_.select(obs);
    const Goal goal = core_.step(intent, obs);

    std_msgs::msg::String intent_msg;
    intent_msg.data = toString(intent);
    intent_pub_->publish(intent_msg);

    if (intent != last_intent_) {
      RCLCPP_INFO(get_logger(), "intent -> %s", toString(intent));
      last_intent_ = intent;
    }

    if (!goal.valid) {
      return;
    }

    const double moved = distance({goal.wp.x, goal.wp.y}, last_goal_);
    const bool changed_intent = intent != last_published_intent_;
    const bool moved_enough = !has_published_ || moved > params_.goal_dedupe_m;
    const bool timeout = !has_published_ ||
                         (now().seconds() - last_publish_s_) >
                             params_.goal_republish_s;
    if (!changed_intent && !moved_enough && !timeout) {
      return;
    }

    publishGoal(goal.wp);
    last_goal_ = {goal.wp.x, goal.wp.y};
    last_published_intent_ = intent;
    last_publish_s_ = now().seconds();
    has_published_ = true;
  }

  // 发布一个目标点。frame_id 取 goal_frame；yaw 转成四元数写入 orientation。
  // wp: 目标点（x/y 单位米，yaw 单位弧度）。
  void publishGoal(const Waypoint& wp) {
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
    RCLCPP_INFO(get_logger(), "goal -> (%.2f, %.2f, %.2f)", wp.x, wp.y, wp.yaw);
  }

  DecisionCore core_;               // 纯逻辑决策核心
  DecisionParams params_;           // 参数快照（阈值 + 点表）
  bool points_ready_{false};        // 点表是否加载成功；false 时不发目标

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;        // TF 查询缓冲
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  // 填充缓冲
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;  // /goal_pose
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr intent_pub_;  // 意图调试话题
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  // 里程计
  rclcpp::TimerBase::SharedPtr timer_;  // 决策定时器

  double odom_x_{0.0};       // 里程计系下的 x，米
  double odom_y_{0.0};       // 里程计系下的 y，米
  double odom_yaw_{0.0};     // 里程计系下的朝向，弧度
  bool have_odom_{false};    // 是否收到过至少一帧里程计

  Vec2 last_goal_{};                        // 上次发布的目标位置
  double last_publish_s_{0.0};              // 上次发布目标的时间
  bool has_published_{false};               // 是否发布过目标（首帧要发）
  Intent last_intent_{Intent::IDLE};        // 上一帧意图，用于只打印变化日志
  Intent last_published_intent_{Intent::IDLE};  // 上次发布目标时的意图，用于去重
};

}  // namespace mas2027_decision

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mas2027_decision::DecisionNode>());
  rclcpp::shutdown();
  return 0;
}
