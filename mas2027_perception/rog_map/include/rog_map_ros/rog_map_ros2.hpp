/**
 * This file is part of ROG-Map
 *
 * Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
 * Developed by Yunfan REN <renyf at connect dot hku dot hk>
 * for more information see <https://github.com/hku-mars/ROG-Map>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * ROG-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ROG-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * ROGMapROS —— ROGMap 的 ROS 2 适配层。
 *
 * 职责：把纯算法库 rog_map 挂到任意既有的 rclcpp / rclcpp_lifecycle 节点上（自己【不】持有节点，
 * 也不提供可执行文件），负责订阅、参数、可视化发布与线程调度。
 * 在线导航链路上，它由 mas2027_nav_executor 的 PathPlanner 在进程内构造；
 * rog_map_node 里的独立宿主只用于离线调试建图。
 *
 * 线程模型（建议按此顺序阅读本文件）：
 *   - odomCallback / cloudCallback：订阅回调，只做校验与数据搬运，不做地图计算，必须保持轻量；
 *   - updateWorkerLoop：唯一的后台工作线程，串行消费点云并调用 ROGMap::updateMapInternal()；
 *   - watchdogCallback：1 Hz 定时器，仅用于"收不到点云"告警；
 *   - vizCallback：可视化发布定时器，只 publish 更新线程预先算好的 VizFrame，不与建图抢锁。
 *
 * 关键约定：
 *   - 地图更新只在 updateWorkerLoop 里串行发生，外部（规划器）通过不可变快照读取，不加地图锁；
 *   - 可视化遵循"更新线程算、定时器发"的双缓冲，禁止在 ROS 回调里直接访问地图容器。
 */

// 说明：以下三个标准头位于 include guard（见下方 ROG_MAP_ROS_HPP）之外，沿用上游写法。
// 重复包含由标准库自身的保护宏兜底，此处保持原样以免与上游产生无谓的 diff 漂移。
#include <cstddef>
#include <cstdint>
#include <vector>

#ifndef ROG_MAP_ROS_HPP
#define ROG_MAP_ROS_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rog_map/rog_map.h>
#include <rog_map/rog_map_visualizer.hpp>
#include <super_utils/color_msg_utils.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace rog_map {
using namespace super_utils;

class ROGMapROS : public ROGMap
{
  /// 宿主节点的强引用。两个构造重载恰好只会填充其中一个，另一个保持为空。
  rclcpp::Node::SharedPtr nh_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_nh_;
  /// 只借用宿主的 node interfaces，而不是整节点：这样本类既能挂在普通节点上，
  /// 也能挂在生命周期节点上，且不会与宿主的执行器产生所有权纠缠。
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  /// 与宿主共享的 TF buffer。仅当 prior_map_frame != frame_id（先验图与局部地图不同坐标系）
  /// 时才真正被 getPriorMapTransform() 使用；宿主没传时允许为空。
  std::shared_ptr<tf2_ros::Buffer> tf_;
  /// 可视化发布器集合与限频定时器；visualization.enable 为 false 时保持为空。
  std::unique_ptr<ROGMapVisualizer> visualizer_driver_;

  /// 覆写基类的纯虚时钟：ROGMap 内部所有时间戳统一取 ROS 时钟（可仿真时间），
  /// 而不是 std::chrono 的墙上时间。
  const double getSystemWalltimeNow() override { return now().seconds(); }

  /// 上游遗留的 rclcpp::Time 版本重载，当前仓库内无调用点（保留以兼容上游接口）。
  void getSystemWalltimeNow(rclcpp::Time & _in) { _in = now(); };

  rclcpp::Time now() const { return node_clock_->get_clock()->now(); }

  /// 覆写基类钩子：给出「ROGMap 局部地图坐标系 -> 先验地图坐标系」的二维刚体变换，
  /// 供 rebuildFusedProjection() 把 PGM 先验图贴到当前滑动窗口上。
  /// 返回 false 表示本帧拿不到有效变换，此时融合会退化为「只用在线感知结果」。
  bool getPriorMapTransform(PriorMapTransform2D & transform) override
  {
    // 先验图与局部地图同坐标系时无需 TF，直接用单位变换。
    if (cfg_.prior_map_frame == cfg_.frame_id) {
      transform = PriorMapTransform2D{};
      return true;
    }
    // 不同坐标系则必须依赖宿主注入的 TF buffer；缺失时按 2 s 节流告警，避免刷屏。
    if (!tf_) {
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] prior map TF is unavailable because the shared TF buffer is null");
      return false;
    }
    try {
      // lookupTransform(target, source, ...) returns T_target_source.
      const auto tf_msg = tf_->lookupTransform(cfg_.prior_map_frame, cfg_.frame_id, tf2::TimePointZero);
      transform.tx = tf_msg.transform.translation.x;
      transform.ty = tf_msg.transform.translation.y;
      // 只取 yaw：先验栅格是二维的，roll/pitch 无法表达，这里从四元数投影出平面旋转。
      const auto & q = tf_msg.transform.rotation;
      transform.yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      // 显式做有限性校验：TF 异常值一旦进入融合循环会污染整张先验投影。
      if (std::isfinite(transform.tx) && std::isfinite(transform.ty) &&
          std::isfinite(transform.yaw)) {
        return true;
      }
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] prior map TF from '%s' to '%s' contains a non-finite 2D transform",
        cfg_.frame_id.c_str(),
        cfg_.prior_map_frame.c_str());
      return false;
    } catch (const tf2::TransformException & error) {
      // TF 未就绪属于启动期常见情形，按 2 s 节流告警而不是报错中断。
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] cannot transform projection from '%s' to prior map frame '%s': %s",
        cfg_.frame_id.c_str(),
        cfg_.prior_map_frame.c_str(),
        error.what());
      return false;
    }
  }

  /// 从宿主节点借用各类 node interface。模板化是为了同时接受
  /// rclcpp::Node::SharedPtr 与 rclcpp_lifecycle::LifecycleNode::SharedPtr。
  template <typename NodeT> void bindNode(NodeT node)
  {
    node_base_ = node->get_node_base_interface();
    node_topics_ = node->get_node_topics_interface();
    node_timers_ = node->get_node_timers_interface();
    node_clock_ = node->get_node_clock_interface();
    node_logging_ = node->get_node_logging_interface();
    node_parameters_ = node->get_node_parameters_interface();
  }

  rclcpp::CallbackGroup::SharedPtr createCallbackGroup(rclcpp::CallbackGroupType type)
  {
    return node_base_->create_callback_group(type);
  }

  // 下面四个 create* 封装都用「node interface + rclcpp::create_xxx」的自由函数形式，
  // 而不是 node->create_xxx()：这是为了在没有节点对象、只有 interface 的前提下建实体。
  template <typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr createPublisher(
    const std::string & topic, const rclcpp::QoS & qos)
  {
    return rclcpp::create_publisher<MsgT>(node_parameters_, node_topics_, topic, qos);
  }

  template <typename MsgT, typename CallbackT>
  typename rclcpp::Subscription<MsgT>::SharedPtr createSubscription(const std::string & topic,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const rclcpp::SubscriptionOptions & options = rclcpp::SubscriptionOptions())
  {
    return rclcpp::create_subscription<MsgT>(
      node_parameters_, node_topics_, topic, qos, std::forward<CallbackT>(callback), options);
  }

  /// group 传 nullptr 时定时器落在默认回调组（节点默认是 MutuallyExclusive）。
  template <typename DurationRepT, typename DurationT, typename CallbackT>
  rclcpp::TimerBase::SharedPtr createWallTimer(std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT && callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return rclcpp::create_wall_timer(
      period, std::forward<CallbackT>(callback), group, node_base_.get(), node_timers_.get());
  }

  /// 本类发布的可视化话题句柄集合，由 initializeRos() 从 visualizer_driver_ 拷入。
  ROGMapVisualizer::Publishers vm_;

  /// 点云接力区：ROS 回调只把最新一帧放进这里，重活交给 updateWorkerLoop。
  /// 这样即使建图一帧耗时超过点云周期，订阅回调也不会阻塞执行器。
  struct ROSCallback
  {
    /// odom 与 cloud 各自独立互斥组，避免两者互相排队；update 组给看门狗定时器用。
    rclcpp::CallbackGroup::SharedPtr odom_me_cbk_group, cloud_me_cbk_group, update_cbk_group;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    /// 自上次被工作线程取走后累积的帧数；>1 表示发生了丢帧（只保留最新帧，丢弃中间帧）。
    int unfinished_frame_cnt{0};
    /// 与 pc 配对的位姿，取的是「点云回调时刻」的最近 odom，而非点云自带时间戳。
    Pose pc_pose;
    /// Only the ROS callbacks access this input pose, under updete_lock.
    /// The base class robot_state_ belongs to the map update worker.
    Pose latest_odom_pose;
    double latest_odom_received_s{0.0};
    bool has_latest_odom{false};
    PointCloud pc;
    /// 点云回调时刻的 odom 年龄（毫秒），用于性能统计判断里程计是否滞后。
    double pc_odom_age_ms{0.0};
    rclcpp::TimerBase::SharedPtr update_timer;
    /// 注意：updete_lock 是上游的拼写（应为 update_lock），为兼容上游此处不改名。
    std::mutex updete_lock;
    std::condition_variable cv;
    /// 置位并 notify_all() 即可让工作线程退出（见 stopUpdateWorker）。
    std::atomic<bool> stop_worker{false};
    std::thread update_worker;
  } rc_;
  /// 串行化整次地图更新。当前只有更新线程获取；预留给后续外部地图读写（如离线保存/加载）接入。
  std::mutex map_io_mutex_;
  /// 保护 viz_frame_ / viz_heavy_ 这两个双缓冲指针（更新线程写、发布定时器读）。
  std::mutex viz_frame_mutex_;
  /// 一帧可视化数据的快照。更新线程构建完成后整体 swap 进 viz_frame_，
  /// 发布定时器只读不改，从而避免发布时与建图线程争用地图容器。
  struct VizFrame
  {
    /// 点云类输出：占据、原始占据、未知、膨胀占据/未知、前沿、ESDF、高度差、距离场、衰减单元。
    sensor_msgs::msg::PointCloud2 occ, raw_occ, unknown, occ_inf, unknown_inf, frontier, esdf,
      height_delta, field, decay;
    /// 栅格类输出：融合 value、动态 value、静态先验、四分类类型图、分类置信度。
    nav_msgs::msg::OccupancyGrid layer_value, layer_dynamic, layer_static, layer_type,
      layer_confidence, terrain_label;
    visualization_msgs::msg::MarkerArray markers, terrain_markers;
    // 以下 has_* 表示「本帧确实填充了对应字段」。发布端据此跳过未填充的字段，
    // 而填充本身只在对应话题有订阅者时才做（见 captureVizFrame），以此省掉无谓的拷贝。
    bool has_occ{false};
    bool has_raw_occ{false};
    bool has_unknown{false};
    bool has_occ_inf{false};
    bool has_unknown_inf{false};
    bool has_frontier{false};
    bool has_esdf{false};
    bool has_height_delta{false};
    bool has_field{false};
    bool has_decay{false};
    bool has_layer_value{false};
    bool has_layer_dynamic{false};
    bool has_layer_static{false};
    bool has_layer_type{false};
    bool has_layer_confidence{false};
    bool has_terrain_label{false};
    bool has_markers{false};
    bool has_terrain_markers{false};
  };
  /// 双缓冲：更新线程写、vizCallback 读，读写都持 viz_frame_mutex_。
  /// viz_heavy_ 只承载高成本项（未知/前沿/膨胀/ESDF），限频 2 Hz；为 null 表示本轮跳过重活。
  std::shared_ptr<const VizFrame> viz_frame_;
  std::shared_ptr<const VizFrame> viz_heavy_;
  /// 上次构建 heavy 帧的 ROS 时间戳（秒），用于 0.5 s 限频。
  double last_heavy_viz_s_{0.0};
  /// 可视化发布周期（秒，来自 visualization.rate）与上次构建快照的时间戳（秒）。
  /// 更新频率通常远高于发布频率，用它对快照构建限频，避免为一次发布重复构建多帧。
  double viz_publish_period_s_{0.0};
  double last_viz_build_s_{0.0};
  /// 本次 captureVizFrame 的耗时（ms），写进性能统计的 last_viz_time_ms。
  /// 这段开销发生在 updateMapInternal() 之后，不在 total_update_time 里——它正是
  /// update_unaccounted_ms（真实周期 − 自报耗时）的主要来源之一。
  double last_viz_time_ms_{0.0};

  /// 里程计回调只更新供点云配对的输入位姿；地图状态由更新线程独占。
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    if (performance_monitor_) {
      performance_monitor_->recordOdom(now().seconds());
    }
    std::lock_guard<std::mutex> lk(rc_.updete_lock);
    rc_.latest_odom_pose = std::make_pair(
      Vec3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z),
      Quatf(odom_msg->pose.pose.orientation.w,
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z));
    rc_.latest_odom_received_s = now().seconds();
    rc_.has_latest_odom = true;
  }

  /// 点云回调：只做四件事——校验、转换、入槽、唤醒工作线程。
  /// 全程不触碰地图容器，唯一的重量级操作是 pcl::fromROSMsg（点云反序列化）。
  /// 采用 take-latest 语义：持续到来的点云会覆盖槽位，工作线程永远只处理最新一帧。
  void cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr cloud_msg)
  {
    const double cbk_t = now().seconds();
    const double msg_stamp = rclcpp::Time(cloud_msg->header.stamp).seconds();
    // 队列时延 = 回调时刻 - 消息时间戳；stamp 为 0（未打时间戳）时记为 0 不参与统计。
    const double queue_delay_ms = msg_stamp > 0.0 ? std::max(0.0, cbk_t - msg_stamp) * 1000.0 : 0.0;
    const double msg_points =
      static_cast<double>(cloud_msg->width) * static_cast<double>(cloud_msg->height);
    if (performance_monitor_) {
      performance_monitor_->recordCloudCallback(cbk_t, msg_points, queue_delay_ms, 0.0);
    }
    // 丢弃原因都会被性能监视器分类计数，便于区分「没数据」和「数据被主动丢掉」。
    if (msg_points <= 0.0) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropEmpty();
      }
      return;
    }
    // Copy one consistent odom sample before the potentially slow conversion.
    Pose paired_pose;
    double paired_odom_received_s = 0.0;
    bool has_paired_odom = false;
    {
      std::lock_guard<std::mutex> lk(rc_.updete_lock);
      has_paired_odom = rc_.has_latest_odom;
      if (has_paired_odom) {
        paired_pose = rc_.latest_odom_pose;
        paired_odom_received_s = rc_.latest_odom_received_s;
      }
    }
    // 没有里程计就无法把点云插入地图（ROGMap 需要位姿做 raycast 起点）。
    if (!has_paired_odom) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropNoOdom();
      }
      std::cout << YELLOW << " -- [ROS] No odom received, skip cloud callback." << RESET << std::endl;
      return;
    }
    // odom 超时同样跳过：宁可丢帧，也不要用过期位姿把点云写到错误位置。
    if (cbk_t - paired_odom_received_s > cfg_.odom_timeout) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropOdomTimeout();
      }
      std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
      return;
    }
    // 反序列化放在锁外，避免长时间持锁阻塞 odomCallback。
    PointCloud temp_pc;
    const auto convert_start = std::chrono::steady_clock::now();
    pcl::fromROSMsg(*cloud_msg, temp_pc);
    const double convert_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - convert_start).count();
    if (performance_monitor_) {
      performance_monitor_->recordCloudConvertTime(convert_time_ms);
    }
    if (temp_pc.empty()) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropEmpty();
      }
      return;
    }
    // 入槽：整体替换上一帧未处理的数据，并记录与点云配对的那一帧 odom 的时间新鲜度。
    {
      std::lock_guard<std::mutex> lk(rc_.updete_lock);
      rc_.pc = std::move(temp_pc);
      rc_.pc_pose = paired_pose;
      rc_.pc_odom_age_ms = std::max(0.0, cbk_t - paired_odom_received_s) * 1000.0;
      rc_.unfinished_frame_cnt++;
      map_empty_ = false;
    }
    rc_.cv.notify_one();
  }

  /// 1 Hz 看门狗：只在「一帧点云都没收到过」时提示话题配置可能有误。
  /// 注意它不负责重启或恢复，纯粹的首次配置排障提示。
  void watchdogCallback()
  {
    if (map_empty_ && cfg_.ros_callback_en) {
      std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET
                << std::endl;
    }
  }

  /// 析构路径：置停止位并唤醒，等 updateWorkerLoop 自然退出后 join。
  void stopUpdateWorker()
  {
    rc_.stop_worker.store(true);
    rc_.cv.notify_all();
    if (rc_.update_worker.joinable()) {
      rc_.update_worker.join();
    }
  }

  /// 后台更新线程主循环——整个在线建图链路的唯一入口。
  /// 语义：等待有点云 -> 取走最新一帧 -> 串行更新地图 -> （有订阅者时）构建可视化快照。
  /// 因为整轮更新串行执行，ROGMap 内部的地图容器不需要额外的线程安全保护。
  void updateWorkerLoop()
  {
    while (!rc_.stop_worker.load()) {
      PointCloud temp_pc;
      Pose temp_pose;
      double temp_odom_age_ms = 0.0;
      int dropped = 0;
      {
        // 无数据时在此休眠，不占 CPU；唤醒条件是「有未处理帧」或「要求退出」。
        std::unique_lock<std::mutex> lk(rc_.updete_lock);
        rc_.cv.wait(lk, [this]() {
          return rc_.stop_worker.load() || rc_.unfinished_frame_cnt > 0;
        });
        if (rc_.stop_worker.load()) {
          break;
        }
        // 取出最新一帧并把计数清零：此处的 dropped 即「本次更新期间被覆盖掉的帧数 + 1」。
        dropped = rc_.unfinished_frame_cnt;
        temp_pc.swap(rc_.pc);
        temp_pose = rc_.pc_pose;
        temp_odom_age_ms = rc_.pc_odom_age_ms;
        rc_.unfinished_frame_cnt = 0;
      }
      // 丢帧说明建图已经跟不上点云频率，按 1 s 节流提示，避免刷屏。
      if (dropped > 1) {
        static double last_warn_t = 0.0;
        const double cur_t = now().seconds();
        if (cur_t - last_warn_t > 1.0) {
          std::cout << YELLOW << " -- [ROG WARN] Unfinished frame cnt > 1 (dropped "
                    << (dropped - 1) << "), the map may not work in real-time" << RESET
                    << std::endl;
          last_warn_t = cur_t;
        }
      }
      if (performance_monitor_) {
        performance_monitor_->recordValidCloud(temp_odom_age_ms);
      }
      {
        // map_io_mutex_ 覆盖整次更新（含 raycast、二维投影、ESDF 与快照生成）。
        std::lock_guard<std::mutex> io(map_io_mutex_);
        updateMapInternal(temp_pc, temp_pose);
      }
      // 可视化快照在更新之后、仍在同一线程内构建，保证与刚写入的地图状态一致。
      // hasVisualizationSubscriber() 为假时整段跳过，无 RViz 时零开销。
      //
      // 还要按**发布频率**限频：vizCallback 是 visualization.rate（现场 5 Hz）的定时器，
      // 而更新跑在点云频率（20 Hz）上——此前每轮都重建整套快照，其中约 3/4 的结果在下次发布
      // 之前就被新帧覆盖，纯属白烧更新线程的时间（RViz 订阅了 /rog_map/layer_* 时每轮要遍历
      // 整张 200×200 投影层，多项叠加就是十万级格）。这里只在「距上次构建已达一个发布周期」
      // 时才建：RViz 侧看到的刷新率不变（仍是 5 Hz 定时器在发），更新线程省掉这部分开销。
      if (cfg_.visualization_en && hasVisualizationSubscriber()) {
        if (viz_publish_period_s_ <= 0.0) {
          // 周期只算一次；cfg_.visualization_rate 在 config 校验里已保证 > 0。
          viz_publish_period_s_ =
            cfg_.visualization_rate > 0.0 ? 1.0 / cfg_.visualization_rate : 0.2;
        }
        const double viz_now_s = now().seconds();
        if (last_viz_build_s_ <= 0.0 || (viz_now_s - last_viz_build_s_) >= viz_publish_period_s_) {
          last_viz_build_s_ = viz_now_s;
          if (performance_monitor_) {
            performance_monitor_->recordVizFrameBuilt();
          }
          captureVizFrame();
          // 把这段耗时并入下一次统计：它不在 total_update_time 内，正是
          // update_unaccounted_ms 的主要成分，单独成列后才能判断限频省了多少。
          if (performance_monitor_) {
            performance_monitor_->recordVizFrameTime(last_viz_time_ms_);
          }
        } else if (performance_monitor_) {
          // 被限频跳过的轮数：配合 last_viz_time_ms / update_unaccounted_ms 判断这块还值不值得压。
          performance_monitor_->recordVizFrameSkipped();
        }
      }
    }
  }

  /// 收集可视化范围内的占据点。开启衰减活跃表时直接遍历活跃单元（O(活跃数)），
  /// 否则退化为对整个包围盒做 boxSearch（O(包围盒体积)）——这是 decay_active_list_en 的主要收益。
  void collectOccupiedForViz(const Vec3f & box_min, const Vec3f & box_max, vec_E<Vec3f> & out)
  {
    out.clear();
    if (!cfg_.decay_active_list_en) {
      boxSearch(box_min, box_max, OCCUPIED, out);
      return;
    }
    out.reserve(active_ids_.size());
    for (const int hash_id : active_ids_) {
      // 三重校验：hash 合法、仍处于活跃表、且当前确实被判为占据。
      if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size()) ||
          hash_id >= static_cast<int>(active_flags_.size()) || !active_flags_[hash_id] ||
          !isOccupied(occupancy_buffer_[hash_id])) {
        continue;
      }
      Vec3f pos;
      hashIdToPos(hash_id, pos);
      if ((pos.array() < box_min.array()).any() || (pos.array() > box_max.array()).any()) {
        continue;
      }
      out.push_back(pos);
    }
  }

  /// /rog_map/layer_type 中表示 UNKNOWN 的内部哨兵值。取 255 是为了让它与
  /// FREE(33)/PASSABLE(66)/OCCUPIED(100) 都不同，发布时再由 fillLayerGrid
  /// 按 OccupancyGrid 约定转成 -1（未知）。若直接发 0，RViz 会把它当成"自由"渲染，
  /// 未知区域就会和 FREE 同色。
  static constexpr uint8_t kUnknownTypeValue = 255U;

  /// 在更新线程内构建一帧可视化快照。四个要点：
  ///   1) 只有存在可视化订阅者时才会被调用，无 RViz 时完全零开销；
  ///   2) 调用侧已按发布频率限频，因此这里每一帧都会被 vizCallback 真正发出去；
  ///   3) 每一项都单独判断「该话题是否有订阅者」，没订阅就不算、不拷贝；
  ///   4) 低成本项每次构建，高成本项（unknown/frontier/膨胀/ESDF）限频 0.5 s 放进 heavy 帧。
  void captureVizFrame()
  {
    // 一帧点云都没收到时不发布，避免 RViz 里出现一堆空帧。
    if (map_empty_) {
      last_viz_time_ms_ = 0.0;
      return;
    }
    const auto viz_start = std::chrono::steady_clock::now();
    auto frame = std::make_shared<VizFrame>();
    // 显示范围以机器人为中心，再裁剪到局部地图边界内。
    Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;
    Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;

    boundBoxByLocalMap(box_min, box_max);
    if ((box_max - box_min).minCoeff() <= 0) {
      cout << YELLOW << " -- [ROGMap] Visualization range is too small." << RESET << endl;
      return;
    }

    // ---- 二维投影层相关的栅格/点云（低成本，每轮都构建）----
    if (layer_ && !layer_->empty()) {
      // 融合层：在线感知与静态先验合并后的二值占据图（由 fused_projection_mask_ 转成 0/100）。
      if (vm_.layer_value_pub && vm_.layer_value_pub->get_subscription_count() >= 1) {
        fillLayerMaskGrid(fused_projection_mask_, frame->layer_value);
        frame->has_layer_value = true;
      }
      // 仅在线动态层：直接取 projection layer 自身的 mask。
      if (vm_.layer_value_dynamic_pub &&
          vm_.layer_value_dynamic_pub->get_subscription_count() >= 1) {
        fillLayerMaskGrid(layer_->mask(), frame->layer_dynamic);
        frame->has_layer_dynamic = true;
      }
      // 仅静态先验层：先验图未启用时该 mask 全为自由，仅供诊断（默认在 RViz 中关闭）。
      if (vm_.layer_value_static_pub &&
          vm_.layer_value_static_pub->get_subscription_count() >= 1) {
        fillLayerMaskGrid(prior_projection_mask_, frame->layer_static);
        frame->has_layer_static = true;
      }
      // 四分类类型图：UNKNOWN=kUnknownTypeValue(→发布为 -1)、FREE=33、PASSABLE=66、OCCUPIED=100，
      // 这样 RViz 用 costmap 配色即可把四类分成四档显示。
      if (vm_.layer_type_pub && vm_.layer_type_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> types(layer_->cells().size(), kUnknownTypeValue);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          switch (layer_->cells()[i].type) {
          case CellType::UNKNOWN:
            types[i] = kUnknownTypeValue;
            break;
          case CellType::FREE:
            types[i] = 33U;
            break;
          case CellType::PASSABLE:
            types[i] = 66U;
            break;
          case CellType::OCCUPIED:
            types[i] = 100U;
            break;
          }
        }
        fillLayerGrid(types, frame->layer_type);
        frame->has_layer_type = true;
      }
      const bool want_terrain_grid = vm_.terrain_label_pub &&
        vm_.terrain_label_pub->get_subscription_count() >= 1;
      const bool want_terrain_markers = vm_.terrain_markers_pub &&
        vm_.terrain_markers_pub->get_subscription_count() >= 1;
      if (want_terrain_grid || want_terrain_markers) {
        const auto snapshot = query_ ? query_->snapshot() : nullptr;
        const auto labels = snapshot && snapshot->terrain_labels.size() ==
          static_cast<size_t>(layer_->width()) * layer_->height() ?
          snapshot->terrain_labels : layer_->terrainLabels();
        if (want_terrain_grid) {
          fillLayerGrid(labels, frame->terrain_label);
          frame->has_terrain_label = true;
        }
        if (want_terrain_markers) {
          fillTerrainMarkers(labels, frame->terrain_markers);
          frame->has_terrain_markers = true;
        }
      }
      // 分类置信度按 0~100 百分数发布（CellData::confidence 本身是 0~1 的 float）。
      if (vm_.layer_confidence_pub && vm_.layer_confidence_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> confidence(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          confidence[i] = static_cast<uint8_t>(
            std::clamp(static_cast<int>(std::round(layer_->cells()[i].confidence * 100.0f)), 0, 100));
        }
        fillLayerGrid(confidence, frame->layer_confidence);
        frame->has_layer_confidence = true;
      }
      if (vm_.layer_height_delta_pub && vm_.layer_height_delta_pub->get_subscription_count() >= 1) {
        fillLayerHeightDeltaCloud(frame->height_delta);
        frame->has_height_delta = true;
      }
    }

    if (field_ && field_->isValid() && vm_.field_pub && vm_.field_pub->get_subscription_count() >= 1) {
      fillFieldCloud(frame->field);
      frame->has_field = true;
    }
    if (vm_.decay_cells_pub && vm_.decay_cells_pub->get_subscription_count() >= 1) {
      fillDecayCellsCloud(frame->decay);
      frame->has_decay = true;
    }

    vec_E<Vec3f> occ_map;
    if ((vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) ||
        (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1)) {
      collectOccupiedForViz(box_min, box_max, occ_map);
    }
    if (vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) {
      vecEVec3fToPC2(occ_map, frame->occ);
      frame->has_occ = true;
    }
    if (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1) {
      vecEVec3fToPC2(occ_map, frame->raw_occ);
      frame->has_raw_occ = true;
    }

    // ---- 调试辅助 Marker（包围盒 / 文字 / 原点）----
    // 依次画出：可视化范围（紫）、局部地图范围（橙）、本轮 raycast 更新范围（绿）、
    // 局部地图原点（红点），以及 ESDF 更新范围（蓝，仅 esdf_en 时）。
    if (vm_.mkr_arr_pub && vm_.mkr_arr_pub->get_subscription_count() >= 1) {
      visualization_msgs::msg::MarkerArray mkr_arr;
      visualizeBoundingBox(
        mkr_arr, now(), box_min, box_max, "Visualization Range", Color::Purple());
      visualizeText(mkr_arr,
        now(),
        "Visualization Range Text",
        "Visualization Range",
        box_max + Vec3f(0, 0, 0.5),
        Color::Purple(),
        0.6,
        0);

      Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
      boundBoxByLocalMap(local_map_min, local_map_max);
      visualizeBoundingBox(
        mkr_arr, now(), local_map_min, local_map_max, "Local Map Range", Color::Orange());
      visualizeText(mkr_arr,
        now(),
        "Local Map Range Text",
        "Local Map Range",
        local_map_max + Vec3f(0, 0, 1.0),
        Color::Orange(),
        0.6,
        0);

      visualizeBoundingBox(mkr_arr,
        now(),
        raycast_data_.cache_box_min,
        raycast_data_.cache_box_max,
        "Updating Range",
        Color::Green());
      visualizeText(mkr_arr,
        now(),
        "Updating Range Text",
        "Updating Range",
        raycast_data_.cache_box_max + Vec3f(0, 0, 0.5),
        Color::Green(),
        0.6,
        0);

      visualizePoint(
        mkr_arr, now(), local_map_origin_d_, Color::Red(), "Local Map Origin", 0.2, 0);

      if (cfg_.esdf_en) {
        Vec3f esdf_box_max, esdf_box_min;
        esdf_map_->getUpdatedBbox(esdf_box_min, esdf_box_max);
        visualizeText(mkr_arr,
          now(),
          "ESDF Map Text",
          "ESDF Map",
          esdf_box_max + Vec3f(0, 0, 1.0),
          Color::Blue(),
          0.6,
          0);
        visualizeBoundingBox(
          mkr_arr, now(), esdf_box_min, esdf_box_max, "ESDF Updating Range", Color::Blue());
      }

      // marker 先以 "world" 为 frame 构建，这里统一改写成配置的可视化坐标系。
      for (auto & marker : mkr_arr.markers) {
        marker.header.frame_id = cfg_.visualization_frame_id;
      }
      frame->markers = std::move(mkr_arr);
      frame->has_markers = true;
    }

    // ---- 重活层：限频构建 ----
    // unknown / frontier / 膨胀层 / ESDF 需要遍历包围盒或做点云转换，成本远高于上面几项，
    // 因此单独放进 heavy 帧并限制到 2 Hz；RViz 里这几层的刷新慢半拍属于预期行为。
    const double now_s = now().seconds();
    const bool heavy_due = (now_s - last_heavy_viz_s_) >= 0.5;
    std::shared_ptr<VizFrame> heavy;
    if (heavy_due) {
      last_heavy_viz_s_ = now_s;
      heavy = std::make_shared<VizFrame>();
      if (vm_.unknown_pub && vm_.unknown_pub->get_subscription_count() >= 1) {
        vec_E<Vec3f> unknown_map;
        boxSearch(box_min, box_max, UNKNOWN, unknown_map);
        vecEVec3fToPC2(unknown_map, heavy->unknown);
        heavy->has_unknown = true;
      }
      if (cfg_.unk_inflation_en && vm_.unknown_inf_pub &&
          vm_.unknown_inf_pub->get_subscription_count() >= 1) {
        vec_E<Vec3f> inf_unknown_map;
        boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
        vecEVec3fToPC2(inf_unknown_map, heavy->unknown_inf);
        heavy->has_unknown_inf = true;
      }
      if (cfg_.frontier_extraction_en && vm_.frontier_pub &&
          vm_.frontier_pub->get_subscription_count() >= 1) {
        vec_E<Vec3f> frontier_map;
        boxSearch(box_min, box_max, FRONTIER, frontier_map);
        vecEVec3fToPC2(frontier_map, heavy->frontier);
        heavy->has_frontier = true;
      }
      if (vm_.occ_inf_pub && vm_.occ_inf_pub->get_subscription_count() >= 1) {
        vec_E<Vec3f> inf_occ_map;
        boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
        vecEVec3fToPC2(inf_occ_map, heavy->occ_inf);
        heavy->has_occ_inf = true;
      }
      if (cfg_.esdf_en && vm_.esdf_pub && vm_.esdf_pub->get_subscription_count() >= 1) {
        PointCloud pc;
        esdf_map_->getPositiveESDFPointCloud(box_min, box_max, robot_state_.p.z() - 0.5, pc);
        pcl::toROSMsg(pc, heavy->esdf);
        heavy->esdf.header.frame_id = cfg_.visualization_frame_id;
        heavy->esdf.header.stamp = now();
        heavy->has_esdf = true;
      }
    }

    // 整体替换双缓冲。持锁时间极短（只做两次 shared_ptr 赋值），
    // 发布端拿到的是 const 快照，之后更新线程继续改地图也不会影响正在发布的数据。
    {
      std::lock_guard<std::mutex> lk(viz_frame_mutex_);
      viz_frame_ = std::move(frame);
      // heavy 为空表示本轮被限频跳过，此时保留上一轮的重活结果继续发布。
      if (heavy) {
        viz_heavy_ = std::move(heavy);
      }
    }
    // 记下耗时，供 updateWorkerLoop 写进性能统计（这段不在 total_update_time 内）。
    last_viz_time_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - viz_start).count();
  }

  /// 可视化发布定时器回调（周期由 visualization.rate 决定，默认 5 Hz）。
  /// 本函数【只读快照、只做 publish】，不做任何地图计算，因此可以与建图线程并行。
  void vizCallback()
  {
    if (!cfg_.visualization_en) {
      return;
    }
    if (!hasVisualizationSubscriber()) {
      return;
    }
    // 在锁内只取指针，锁外再逐个 publish，避免发布耗时阻塞更新线程。
    std::shared_ptr<const VizFrame> frame;
    std::shared_ptr<const VizFrame> heavy;
    {
      std::lock_guard<std::mutex> lk(viz_frame_mutex_);
      frame = viz_frame_;
      heavy = viz_heavy_;
    }
    // 更新线程还没产出第一帧时直接返回。
    if (!frame && !heavy) {
      return;
    }
    if (heavy) {
      if (heavy->has_unknown && vm_.unknown_pub) {
        vm_.unknown_pub->publish(heavy->unknown);
      }
      if (heavy->has_unknown_inf && vm_.unknown_inf_pub) {
        vm_.unknown_inf_pub->publish(heavy->unknown_inf);
      }
      if (heavy->has_frontier && vm_.frontier_pub) {
        vm_.frontier_pub->publish(heavy->frontier);
      }
      if (heavy->has_occ_inf && vm_.occ_inf_pub) {
        vm_.occ_inf_pub->publish(heavy->occ_inf);
      }
      if (heavy->has_esdf && vm_.esdf_pub) {
        vm_.esdf_pub->publish(heavy->esdf);
      }
    }
    if (!frame) {
      return;
    }
    if (frame->has_layer_value && vm_.layer_value_pub) {
      vm_.layer_value_pub->publish(frame->layer_value);
    }
    if (frame->has_layer_dynamic && vm_.layer_value_dynamic_pub) {
      vm_.layer_value_dynamic_pub->publish(frame->layer_dynamic);
    }
    if (frame->has_layer_static && vm_.layer_value_static_pub) {
      vm_.layer_value_static_pub->publish(frame->layer_static);
    }
    if (frame->has_layer_type && vm_.layer_type_pub) {
      vm_.layer_type_pub->publish(frame->layer_type);
    }
    if (frame->has_layer_confidence && vm_.layer_confidence_pub) {
      vm_.layer_confidence_pub->publish(frame->layer_confidence);
    }
    if (frame->has_terrain_label && vm_.terrain_label_pub) {
      vm_.terrain_label_pub->publish(frame->terrain_label);
    }
    if (frame->has_height_delta && vm_.layer_height_delta_pub) {
      vm_.layer_height_delta_pub->publish(frame->height_delta);
    }
    if (frame->has_field && vm_.field_pub) {
      vm_.field_pub->publish(frame->field);
    }
    if (frame->has_decay && vm_.decay_cells_pub) {
      vm_.decay_cells_pub->publish(frame->decay);
    }
    if (frame->has_occ && vm_.occ_pub) {
      vm_.occ_pub->publish(frame->occ);
    }
    if (frame->has_raw_occ && vm_.raw_occ_pub) {
      vm_.raw_occ_pub->publish(frame->raw_occ);
    }
    if (frame->has_markers && vm_.mkr_arr_pub) {
      vm_.mkr_arr_pub->publish(frame->markers);
    }
    if (frame->has_terrain_markers && vm_.terrain_markers_pub) {
      vm_.terrain_markers_pub->publish(frame->terrain_markers);
    }
  }

  /// 任一可视化话题有订阅者即返回 true。更新线程用它决定是否值得构建快照，
  /// 是本文件里「无人观看就零成本」策略的总开关。
  bool hasVisualizationSubscriber()
  {
    return (vm_.unknown_pub && vm_.unknown_pub->get_subscription_count() >= 1) ||
           (vm_.unknown_inf_pub && vm_.unknown_inf_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_pub && vm_.layer_value_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_dynamic_pub &&
            vm_.layer_value_dynamic_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_static_pub &&
            vm_.layer_value_static_pub->get_subscription_count() >= 1) ||
           (vm_.layer_type_pub && vm_.layer_type_pub->get_subscription_count() >= 1) ||
           (vm_.terrain_label_pub && vm_.terrain_label_pub->get_subscription_count() >= 1) ||
           (vm_.terrain_markers_pub && vm_.terrain_markers_pub->get_subscription_count() >= 1) ||
           (vm_.layer_confidence_pub && vm_.layer_confidence_pub->get_subscription_count() >= 1) ||
           (vm_.layer_height_delta_pub && vm_.layer_height_delta_pub->get_subscription_count() >= 1) ||
           (vm_.field_pub && vm_.field_pub->get_subscription_count() >= 1) ||
           (vm_.decay_cells_pub && vm_.decay_cells_pub->get_subscription_count() >= 1) ||
           (vm_.frontier_pub && vm_.frontier_pub->get_subscription_count() >= 1) ||
           (vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) ||
           (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1) ||
           (vm_.occ_inf_pub && vm_.occ_inf_pub->get_subscription_count() >= 1) ||
           (vm_.esdf_pub && vm_.esdf_pub->get_subscription_count() >= 1) ||
           (vm_.mkr_arr_pub && vm_.mkr_arr_pub->get_subscription_count() >= 1);
  }

  /// 把 ROGMap 内部的点集转成 ROS 点云，并统一填上可视化坐标系与当前时间戳。
  /// 注意：时间戳取 now()（发布侧时间），不是点云采集时间，仅供 RViz 显示使用。
  void vecEVec3fToPC2(const vec_E<Vec3f> & points, sensor_msgs::msg::PointCloud2 & cloud)
  {
    // 设置header信息
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());
    for (long unsigned int i = 0; i < points.size(); i++) {
      pcl_cloud[i].x = static_cast<float>(points[i][0]);
      pcl_cloud[i].y = static_cast<float>(points[i][1]);
      pcl_cloud[i].z = static_cast<float>(points[i][2]);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  /// 通用 OccupancyGrid 填充：几何信息直接取 projection layer 的当前滑窗几何。
  /// 数值夹到不超过 100 后按 int8_t 发布，因此 ROGMap 内部的 254(OCCUPIED)
  /// 在 RViz 中显示为 100（满占据）。
  /// 例外：kUnknownTypeValue 按 OccupancyGrid 约定发成 -1（未知），否则会被夹成 100
  /// 而在图上与占据混淆。当前只有 layer_type 会传入该值，其余调用方只传 0~100。
  void fillLayerGrid(const std::vector<uint8_t> & data, nav_msgs::msg::OccupancyGrid & grid)
  {
    grid.header.stamp = now();
    grid.header.frame_id = cfg_.visualization_frame_id;
    grid.info.resolution = static_cast<float>(layer_->resolution());
    grid.info.width = static_cast<uint32_t>(layer_->width());
    grid.info.height = static_cast<uint32_t>(layer_->height());
    grid.info.origin.position.x = layer_->origin().x();
    grid.info.origin.position.y = layer_->origin().y();
    // OccupancyGrid 默认按 xy 平面解读，因此把姿态显式置为单位四元数。
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
      grid.data[i] = data[i] == kUnknownTypeValue ?
        static_cast<int8_t>(-1) : static_cast<int8_t>(std::min<int>(100, data[i]));
    }
  }

  /// 在线特殊地形以两组彩色方块显示；每帧覆盖上一帧，空组显式删除旧标记。
  void fillTerrainMarkers(const std::vector<uint8_t> & labels,
    visualization_msgs::msg::MarkerArray & markers)
  {
    markers.markers.resize(2);
    const auto stamp = now();
    const double resolution = layer_->resolution();
    for (size_t i = 0; i < markers.markers.size(); ++i) {
      auto & marker = markers.markers[i];
      marker.header.stamp = stamp;
      marker.header.frame_id = cfg_.visualization_frame_id;
      marker.ns = "online_terrain";
      marker.id = static_cast<int32_t>(i);
      marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = resolution;
      marker.scale.y = resolution;
      marker.scale.z = 0.03;
      marker.color.r = i == 0 ? 1.0F : 0.0F;
      marker.color.g = i == 0 ? 0.55F : 0.85F;
      marker.color.b = i == 0 ? 0.0F : 1.0F;
      marker.color.a = 0.8F;
      marker.lifetime = rclcpp::Duration::from_seconds(
        std::max(0.5, 2.5 / std::max(1.0, cfg_.visualization_rate)));
    }
    const int width = layer_->width();
    for (size_t index = 0; index < labels.size(); ++index) {
      const size_t marker_index = labels[index] == 5U ? 0U : labels[index] == 6U ? 1U : 2U;
      if (marker_index >= markers.markers.size()) continue;
      geometry_msgs::msg::Point point;
      point.x = layer_->origin().x() + (static_cast<int>(index % width) + 0.5) * resolution;
      point.y = layer_->origin().y() + (static_cast<int>(index / width) + 0.5) * resolution;
      point.z = 0.06;
      markers.markers[marker_index].points.push_back(point);
    }
    for (auto & marker : markers.markers) {
      if (marker.points.empty()) marker.action = visualization_msgs::msg::Marker::DELETE;
    }
  }

  /// 把 0/1 的二维 mask 转成 OccupancyGrid：mask==0（障碍）映射为 100，其余为 0。
  /// 即 /rog_map/layer_value 与 /rog_map/layer_value_dynamic 实际发布的是【二值占据图】，
  /// 名字里的 value 是历史沿用；真正的代价分布要看 /rog_map/layer_type。
  void fillLayerMaskGrid(const std::vector<uint8_t> & mask, nav_msgs::msg::OccupancyGrid & grid)
  {
    std::vector<uint8_t> occupancy(mask.size(), 0U);
    for (size_t i = 0; i < mask.size(); ++i) {
      occupancy[i] = mask[i] == 0U ? 100U : 0U;
    }
    fillLayerGrid(occupancy, grid);
  }

  /// 高度差点云：每个单元一个点，z 取该柱内占据的最高绝对高度，intensity 取 height_delta。
  /// 用于在 RViz 里直接观察"薄表面 / 实心墙 / 中空隧道"的分类依据。
  /// 注意 FREE 单元没有占据体素，其 occupied_z_max_abs 保持未定义（NaN），会被一并发布。
  void fillLayerHeightDeltaCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    const auto & cells = layer_->cells();
    pcl_cloud.reserve(cells.size());
    for (int y = 0; y < layer_->height(); ++y) {
      for (int x = 0; x < layer_->width(); ++x) {
        const size_t idx =
          static_cast<size_t>(y) * static_cast<size_t>(layer_->width()) + static_cast<size_t>(x);
        if (idx >= cells.size()) {
          continue;
        }
        const auto & cell = cells[idx];
        // 未观测区域没有分类意义，直接跳过。
        if (cell.type == CellType::UNKNOWN) {
          continue;
        }
        pcl::PointXYZI p;
        // 取单元中心（+0.5 个分辨率）而不是栅格角点，与 ESDF 的采样约定保持一致。
        p.x =
          static_cast<float>(layer_->origin().x() + (static_cast<double>(x) + 0.5) * layer_->resolution());
        p.y =
          static_cast<float>(layer_->origin().y() + (static_cast<double>(y) + 0.5) * layer_->resolution());
        p.z = cell.occupied_z_max_abs;
        p.intensity = cell.height_delta;
        pcl_cloud.push_back(p);
      }
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  /// 距离场点云：z 固定为 0（二维场），intensity 取该单元的 ESDF 距离值。
  /// 非有限值（未初始化/不可达单元）直接跳过，避免发布无效点。
  void fillFieldCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    const auto distances = field_->distances();
    const int width = field_->width();
    const int height = field_->height();
    const double resolution = field_->resolution();
    const Eigen::Vector2d origin = field_->origin();
    pcl_cloud.reserve(distances.size());
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
        if (idx >= distances.size()) {
          continue;
        }
        const double d = distances[idx];
        if (!std::isfinite(d)) {
          continue;
        }
        pcl::PointXYZI p;
        p.x = static_cast<float>(origin.x() + (static_cast<double>(x) + 0.5) * resolution);
        p.y = static_cast<float>(origin.y() + (static_cast<double>(y) + 0.5) * resolution);
        p.z = 0.0f;
        p.intensity = static_cast<float>(d);
        pcl_cloud.push_back(p);
      }
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  /// 衰减单元点云：只遍历活跃表（而非整个概率地图），intensity 取"距上次命中经过的秒数"，
  /// 用于诊断动态障碍是否按 decay 参数正常消退。
  void fillDecayCellsCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl_cloud.reserve(active_ids_.size());
    const double now_s = now().seconds();
    for (const int hash_id : active_ids_) {
      if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size()) || !active_flags_[hash_id] ||
          !isOccupied(occupancy_buffer_[hash_id])) {
        continue;
      }
      Vec3f pos;
      hashIdToPos(hash_id, pos);
      pcl::PointXYZI p;
      p.x = static_cast<float>(pos.x());
      p.y = static_cast<float>(pos.y());
      p.z = static_cast<float>(pos.z());
      p.intensity = static_cast<float>(std::max(0.0, now_s - static_cast<double>(last_hit_time_[hash_id])));
      pcl_cloud.push_back(p);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  /// 构造期初始化总入口，由两个构造函数共同调用。执行顺序即依赖顺序：
  /// 加载先验图 -> ROGMap::init() 建核心数据结构 -> 建可视化 -> 建订阅与更新线程。
  void initializeRos()
  {
    // TODO: The current implementation uses a lenient QoS configuration for message transmission.
    // 统一的宽松 QoS：BestEffort + KeepLast(1) + Volatile。
    // 对建图而言"最新一帧"远比"不丢帧"重要，因此不要求可靠传输。
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());

    // 先验栅格（PGM+YAML）必须在 init() 之前加载：init() 内部会读取 prior_map_ 决定是否建融合层。
    if (cfg_.prior_map_enable) {
      prior_map_ = loadPriorMap(cfg_.prior_map_yaml_path, cfg_.prior_map_pgm_path);
      RCLCPP_INFO(node_logging_->get_logger(),
        "[ROGMap] loaded prior map %dx%d at %.3f m/cell in frame '%s'",
        prior_map_.width,
        prior_map_.height,
        prior_map_.resolution,
        cfg_.prior_map_frame.c_str());
    }
    init();
    /// Initialize visualization module
    /// 发布器由 ROGMapVisualizer 统一创建（含各自的 QoS 与 5 Hz 定时器），
    /// 这里只把句柄拷进 vm_ 供 captureVizFrame/vizCallback 使用；未启用时 vm_ 全为空，
    /// 后续所有可视化分支都会因空指针判断而整体跳过。
    if (cfg_.visualization_en) {
      visualizer_driver_ = std::make_unique<ROGMapVisualizer>();
      visualizer_driver_->configure(node_base_,
        node_parameters_,
        node_topics_,
        node_timers_,
        cfg_,
        std::bind(&ROGMapROS::vizCallback, this));
      const auto & pubs = visualizer_driver_->publishers();
      vm_.occ_pub = pubs.occ_pub;
      vm_.raw_occ_pub = pubs.raw_occ_pub;
      vm_.unknown_pub = pubs.unknown_pub;
      vm_.esdf_neg_pub = pubs.esdf_neg_pub;
      vm_.esdf_occ_pub = pubs.esdf_occ_pub;
      vm_.occ_inf_pub = pubs.occ_inf_pub;
      vm_.unknown_inf_pub = pubs.unknown_inf_pub;
      vm_.frontier_pub = pubs.frontier_pub;
      vm_.esdf_pub = pubs.esdf_pub;
      vm_.layer_height_delta_pub = pubs.layer_height_delta_pub;
      vm_.field_pub = pubs.field_pub;
      vm_.decay_cells_pub = pubs.decay_cells_pub;
      vm_.layer_value_pub = pubs.layer_value_pub;
      vm_.layer_value_dynamic_pub = pubs.layer_value_dynamic_pub;
      vm_.layer_value_static_pub = pubs.layer_value_static_pub;
      vm_.layer_type_pub = pubs.layer_type_pub;
      vm_.terrain_label_pub = pubs.terrain_label_pub;
      vm_.layer_confidence_pub = pubs.layer_confidence_pub;
      vm_.mkr_arr_pub = pubs.mkr_arr_pub;
      vm_.terrain_markers_pub = pubs.terrain_markers_pub;
    }

    if (cfg_.ros_callback_en) {
      // OpenMP 仅用于 raycast 并行；这里限制为 2 线程，避免与宿主进程里的其他 OpenMP 团队
      // （如 odom_localizer）争抢核心。set_dynamic(0) 禁用动态调整，保证线程数稳定可预期。
#ifdef _OPENMP
      omp_set_dynamic(0);
      omp_set_num_threads(2);
#endif
      // odom 与 cloud 各自独立互斥组：两者回调可以并行，但同类回调不会重入。
      rc_.odom_me_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.cloud_me_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions so;
      so.callback_group = rc_.odom_me_cbk_group;
      rc_.odom_sub = createSubscription<nav_msgs::msg::Odometry>(
        cfg_.odom_topic, qos, std::bind(&ROGMapROS::odomCallback, this, std::placeholders::_1), so);
      so.callback_group = rc_.cloud_me_cbk_group;
      // 开启进程内通信：点云发布者与本对象同进程时可省掉一次序列化/反序列化；
      // 跨进程（当前 nav_executor 与 small_point_lio 分属不同进程）时该设置自动失效，无副作用。
      so.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
      // 点云采用 SensorDataQoS + keep_last(1) + UniquePtr 回调：只保留最新一帧，且尽量零拷贝。
      rc_.cloud_sub = createSubscription<sensor_msgs::msg::PointCloud2>(
        cfg_.cloud_topic,
        rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) { this->cloudCallback(std::move(msg)); },
        so);
      // 看门狗用独立回调组，保证它不会被长时间的点云/更新处理阻塞而漏报。
      rc_.update_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.update_timer = createWallTimer(std::chrono::seconds(1),
        std::bind(&ROGMapROS::watchdogCallback, this),
        rc_.update_cbk_group);
      // 启动唯一的后台更新线程；其生命周期由构造/析构配对管理（见 stopUpdateWorker）。
      rc_.stop_worker.store(false);
      rc_.update_worker = std::thread([this]() { this->updateWorkerLoop(); });
    }
  }

public:
  typedef shared_ptr<ROGMapROS> Ptr;

  /// 禁止拷贝：本类持有线程、互斥量与订阅句柄，复制没有意义。
  ROGMapROS(const ROGMapROS &) = delete;
  ROGMapROS & operator=(const ROGMapROS &) = delete;

  /// 析构必须回收更新线程，否则线程会在对象销毁后继续访问成员。
  ~ROGMapROS()
  {
    stopUpdateWorker();
  }

  /// 生命周期节点版本：适用于把地图挂到 nav2 风格的 LifecycleNode 上。
  /// tf 仅在先验图与局部地图不同坐标系时才需要传入。
  ROGMapROS(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh,
    const rog_map::Config & cfg,
    const std::shared_ptr<tf2_ros::Buffer> & tf = nullptr)
  : lifecycle_nh_(nh), tf_(tf)
  {
    bindNode(lifecycle_nh_);
    cfg_ = cfg;
    initializeRos();
  }

  /// 普通节点版本：当前 mas2027_nav_executor 走的是这一条（PathPlanner 进程内持有）。
  ROGMapROS(const rclcpp::Node::SharedPtr nh,
    const rog_map::Config & cfg,
    const std::shared_ptr<tf2_ros::Buffer> & tf = nullptr)
  : nh_(nh), tf_(tf)
  {
    bindNode(nh_);
    cfg_ = cfg;
    initializeRos();
  }

  /// 对外暴露规划器使用的只读地图接口（内部为 getQueryInterface() 的别名）。
  /// 规划侧拿到的只是不可变快照，不参与建图的加锁协议。
  std::shared_ptr<rog_map::MapQueryInterface> queryInterface() const
  {
    return getQueryInterface();
  }

private:
  /// 以 LINE_STRIP 画一个立方体线框（可视化范围 / 局部地图范围 / 更新范围都用它）。
  /// 注意：8 个角点按 LINE_STRIP 顺序相连只能得到部分棱边，因此后面又补了几个点
  /// 把 12 条棱补全——改动点的顺序时要连同补点一起调整，否则线框会缺边。
  /// marker 的 frame_id 这里先写成 "world"，实际由 captureVizFrame 统一改写为配置值。
  static void visualizeBoundingBox(visualization_msgs::msg::MarkerArray & mkrarr,
    const rclcpp::Time & stamp,
    const Vec3f & box_min,
    const Vec3f & box_max,
    const string & ns,
    const Color & color,
    const double & size_x = 0.1,
    const double & alpha = 1.0,
    const bool & print_ns = true)
  {
    Vec3f size = (box_max - box_min) / 2;
    Vec3f vis_pos_world = (box_min + box_max) / 2;
    double width = size.x();
    double length = size.y();
    double hight = size.z();

    // Publish Bounding box
    int id = 0;
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.stamp = stamp;
    line_strip.header.frame_id = "world";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.ns = ns;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id++;  // unique id, useful when multiple markers exist.
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;  // marker type
    line_strip.scale.x = size_x;

    line_strip.color = color;
    line_strip.color.a = alpha;  //不透明度，设0则全透明
    geometry_msgs::msg::Point p[8];

    // vis_pos_world是目标物的坐标
    p[0].x = vis_pos_world(0) - width;
    p[0].y = vis_pos_world(1) + length;
    p[0].z = vis_pos_world(2) + hight;
    p[1].x = vis_pos_world(0) - width;
    p[1].y = vis_pos_world(1) - length;
    p[1].z = vis_pos_world(2) + hight;
    p[2].x = vis_pos_world(0) - width;
    p[2].y = vis_pos_world(1) - length;
    p[2].z = vis_pos_world(2) - hight;
    p[3].x = vis_pos_world(0) - width;
    p[3].y = vis_pos_world(1) + length;
    p[3].z = vis_pos_world(2) - hight;
    p[4].x = vis_pos_world(0) + width;
    p[4].y = vis_pos_world(1) + length;
    p[4].z = vis_pos_world(2) - hight;
    p[5].x = vis_pos_world(0) + width;
    p[5].y = vis_pos_world(1) - length;
    p[5].z = vis_pos_world(2) - hight;
    p[6].x = vis_pos_world(0) + width;
    p[6].y = vis_pos_world(1) - length;
    p[6].z = vis_pos_world(2) + hight;
    p[7].x = vis_pos_world(0) + width;
    p[7].y = vis_pos_world(1) + length;
    p[7].z = vis_pos_world(2) + hight;
    // LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
    for (int i = 0; i < 8; i++) {
      line_strip.points.push_back(p[i]);
    }
    //为了保证矩形框的八条边都存在：
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[3]);
    line_strip.points.push_back(p[2]);
    line_strip.points.push_back(p[5]);
    line_strip.points.push_back(p[6]);
    line_strip.points.push_back(p[1]);
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[7]);
    line_strip.points.push_back(p[4]);
    mkrarr.markers.push_back(line_strip);
  }

  /// 朝向相机、始终面向观察者的文字标注。
  /// id < 0 时使用函数内静态计数器自增，保证同一帧内多个标注 id 不冲突；
  /// 这些 static 变量只被更新线程访问，因此无需加锁。
  static void visualizeText(visualization_msgs::msg::MarkerArray & mkr_arr,
    const rclcpp::Time & stamp,
    const std::string & ns,
    const std::string & text,
    const Vec3f & position,
    const Color & c = Color::White(),
    const double & size = 0.6,
    const int & id = -1)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = stamp;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.ns = ns.c_str();
    if (id >= 0) {
      marker.id = id;
    } else {
      static int id = 0;
      marker.id = id++;
    }
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = size;
    marker.color = c;
    marker.text = text;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;
    mkr_arr.markers.push_back(marker);
  };

  /// 画一个球体标记（当前用于局部地图原点）；print_ns 为真时在点上方附带同名文字标签。
  /// 坐标含 NaN 时直接跳过——地图未初始化时原点位置可能是 NaN，发出去会让 RViz 报错。
  static void visualizePoint(visualization_msgs::msg::MarkerArray & mkr_arr,
    const rclcpp::Time & stamp,
    const Vec3f & pt,
    Color color = Color::Pink(),
    std::string ns = "pt",
    double size = 0.1,
    int id = -1,
    const bool & print_ns = true)
  {
    visualization_msgs::msg::Marker marker_ball;
    static int cnt = 0;
    Vec3f cur_pos = pt;
    if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
      return;
    }
    marker_ball.header.frame_id = "world";
    marker_ball.header.stamp = stamp;
    marker_ball.ns = ns.c_str();
    marker_ball.id = id >= 0 ? id : cnt++;
    marker_ball.action = visualization_msgs::msg::Marker::ADD;
    marker_ball.pose.orientation.w = 1.0;
    marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
    marker_ball.scale.x = size;
    marker_ball.scale.y = size;
    marker_ball.scale.z = size;
    marker_ball.color = color;

    geometry_msgs::msg::Point p;
    p.x = cur_pos.x();
    p.y = cur_pos.y();
    p.z = cur_pos.z();

    marker_ball.pose.position = p;
    mkr_arr.markers.push_back(marker_ball);

    // add test
    if (print_ns) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = stamp;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.ns = ns + "_text";
      if (id >= 0) {
        marker.id = id;
      } else {
        static int id = 0;
        marker.id = id++;
      }
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 0.6;
      marker.color = color;
      marker.text = ns;
      marker.pose.position.x = cur_pos.x();
      marker.pose.position.y = cur_pos.y();
      marker.pose.position.z = cur_pos.z() + 0.5;
      marker.pose.orientation.w = 1.0;
      mkr_arr.markers.push_back(marker);
    }
  }
};
}  // namespace rog_map
#endif  // ROG_MAP_ROS_HPP
