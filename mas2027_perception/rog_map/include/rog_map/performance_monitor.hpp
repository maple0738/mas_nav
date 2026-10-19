#pragma once

#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace rog_map {

struct RuntimeStats
{
  double stamp{0.0};
  double update_seq{0.0};
  double cloud_callback_count{0.0};
  double cloud_callback_hz{0.0};
  double cloud_msg_points{0.0};
  double cloud_msg_points_avg{0.0};
  double cloud_msg_points_max{0.0};
  double cloud_convert_time_ms{0.0};
  double cloud_queue_delay_ms{0.0};
  double valid_cloud_count{0.0};
  double valid_update_hz{0.0};
  double dropped_cloud_empty_count{0.0};
  double dropped_cloud_no_odom_count{0.0};
  double dropped_cloud_odom_timeout_count{0.0};
  double odom_received_count{0.0};
  double odom_hz{0.0};
  double odom_lookup_time_ms{0.0};
  double odom_age_ms{0.0};
  double total_update_time{0.0};
  double update_robot_state_time{0.0};
  double raycast_time{0.0};
  double prob_update_time{0.0};
  double inflation_time{0.0};
  double raycast_parallel_time{0.0};
  double raycast_merge_time{0.0};
  double decay_time{0.0};
  double projection_time{0.0};
  double field_time{0.0};
  double query_refresh_time{0.0};
  double input_point_count{0.0};
  double raycast_input_point_count{0.0};
  double raycast_used_point_count{0.0};
  double raycast_skipped_near_count{0.0};
  double raycast_skipped_far_count{0.0};
  double raycast_skipped_outside_count{0.0};
  double cache_count{0.0};
  double inflation_count{0.0};
  double hit_count{0.0};
  double miss_count{0.0};
  double active_cell_count{0.0};
  double dirty_column_count_from_probmap{0.0};
  double refresh_layers_time{0.0};
  double projection_total_time{0.0};
  double projection_sequence{0.0};
  double projection_sequence_delta{0.0};
  double projection_config_time{0.0};
  double projection_scanner_time{0.0};
  double projection_update_full_time{0.0};
  double projection_update_dirty_time{0.0};
  double projection_mask_filter_time{0.0};
  double projection_value_mask_time{0.0};
  double projection_count_cells_time{0.0};
  double projection_cell_count{0.0};
  double projection_z_min_id{0.0};
  double projection_z_max_id{0.0};
  double projection_z_layers{0.0};
  double projection_scanned_voxel_estimate{0.0};
  double projection_force_full_refresh{0.0};
  double projection_geometry_changed{0.0};
  double projection_full_layer_required{0.0};
  double projection_dirty_column_enabled{0.0};
  double projection_dirty_over_ratio{0.0};
  double projection_no_update_count{0.0};
  double projection_thin_surface_count{0.0};
  double projection_vertical_wall_count{0.0};
  double projection_hollow_tunnel_count{0.0};
  double projection_ambiguous_occupied_count{0.0};
  double projection_empty_column_count{0.0};
  double projection_insufficient_observation_count{0.0};
  // Counts from ProjectionLayer::terrainGeometry(), not projection classification.
  // They expose whether online terrain labels are being generated over flat ground.
  double terrain_label_flat_count{0.0};
  double terrain_label_slope_count{0.0};
  double terrain_label_tunnel_count{0.0};
  std::string projection_refresh_reason{"none"};
  double layer_mask_changed{0.0};
  double mask_sequence{0.0};
  double mask_sequence_delta{0.0};
  double layer_mask_free_count{0.0};
  double layer_mask_occupied_count{0.0};
  double layer_value_free_count{0.0};
  double layer_value_passable_count{0.0};
  double layer_value_occupied_count{0.0};
  double layer_value_unknown_count{0.0};
  double occupied_count{0.0};
  double unknown_count{0.0};
  double passable_count{0.0};
  double free_count{0.0};
  double decayed_count{0.0};
  double dirty_column_count{0.0};
  double dirty_expanded_column_count{0.0};
  /// 本次更新走到 force_full_refresh 的原因（互斥，按 rog_map.cpp 的判断顺序）：
  /// geometry_changed（滑动窗口自身要求全量）/ explicit_full / dirty_disabled / dirty_over_ratio。
  double full_reason_geometry{0.0};
  double full_reason_explicit{0.0};
  double full_reason_dirty_disabled{0.0};
  double full_reason_dirty_over_ratio{0.0};
  /// 周期性兜底触发的全量（cfg_.dirty_full_period_s 到期）。
  double full_reason_periodic{0.0};
  /// 取走全量标记时它是否为真。为真却走了脏列分支说明标记被推迟，下一帧必强制全量。
  double full_layer_flag_at_consume{0.0};
  /// markDirtyColumn 的三个出口计数：正常入列 / 越界置全量标记 / 表未初始化或列号非法置全量标记。
  /// 越界计数持续 >0 说明「射线打到滑动窗口外」在每帧把整张二维图打成全量。
  double mark_dirty_ok_count{0.0};
  double mark_dirty_out_of_map_count{0.0};
  double mark_dirty_invalid_count{0.0};
  /// refreshLayers 的细分：配置/滑窗同步、脏列合并排序、掩码重建、逐格统计。
  double layer_config_sync_time{0.0};
  double layer_dirty_merge_time{0.0};
  double fused_projection_time{0.0};
  double full_layer_refresh_count{0.0};
  double dirty_layer_update_count{0.0};
  double field_enabled{0.0};
  double field_dirty_before{0.0};
  double field_period_ready{0.0};
  double field_should_update{0.0};
  double field_actual_update{0.0};
  std::string field_skip_reason{"none"};
  double field_skipped_count{0.0};
  double field_update_from_mask_time{0.0};
  double field_edt_positive_time{0.0};
  double field_inverse_mask_time{0.0};
  double field_edt_negative_time{0.0};
  double field_distance_fill_time{0.0};
  double field_copy_time{0.0};
  double field_sequence{0.0};
  double field_sequence_delta{0.0};
  double field_update_count{0.0};
  double field_skip_layer_empty_count{0.0};
  double field_skip_disabled_count{0.0};
  double field_update_interval_ms{0.0};
  double field_update_hz_window{0.0};
  /// 相邻两次 updateMapInternal 返回之间的真实墙钟间隔（ms）。与 total_update_time 的差
  /// 是它没覆盖到的开销（可视化快照构建、掉帧空档），只盯它会低估地图的真实更新周期。
  double update_period_ms{0.0};
  /// update_period_ms - total_update_time，即上面那段不可见开销。
  double update_unaccounted_ms{0.0};
  /// 最近一次可视化快照构建耗时（ms），发生在 updateMapInternal() 之后，是
  /// update_unaccounted_ms 的主要成分；按发布频率限频正是为了省掉这份重复遍历投影层的开销。
  double last_viz_time_ms{0.0};
  /// 本统计窗口内：可视化快照实际构建次数 / 因限频跳过的次数（窗口累计，写进 CSV）。
  /// 两者之比即是限频省下来的比例：点云 20 Hz、发布 5 Hz 时约为 1:3。
  double viz_built_count{0.0};
  double viz_skipped_count{0.0};
  double query_snapshot_alloc_time{0.0};
  double query_copy_values_time{0.0};
  double query_copy_types_height_delta_confidence_time{0.0};
  double cpu_thread_hint{0.0};
};

struct PerformanceConfig
{
  bool enable{true};
  bool detailed_csv_enable{false};
  std::string detailed_csv_path{"/tmp/rog_map_perf_detailed.csv"};
  bool summary_csv_enable{false};
  std::string summary_csv_path{"/tmp/rog_map_perf_summary.csv"};
  std::string run_id{};
  std::string scenario{};
  std::string variant{};
  int csv_flush_every_n{30};
  bool print_enable{false};
  double summary_rate{1.0};
};

class PerformanceMonitor
{
public:
  class ScopedTimer
  {
  public:
    ScopedTimer(PerformanceMonitor * monitor, double RuntimeStats::*field);
    ~ScopedTimer();
    ScopedTimer(const ScopedTimer &) = delete;
    ScopedTimer & operator=(const ScopedTimer &) = delete;

  private:
    PerformanceMonitor * monitor_{nullptr};
    double RuntimeStats::*field_{nullptr};
    std::chrono::steady_clock::time_point start_{};
  };

  void configure(const PerformanceConfig & config);
  bool enabled() const { return config_.enable; }
  bool detailedCsvEnabled() const { return config_.enable && config_.detailed_csv_enable; }
  bool summaryCsvEnabled() const { return config_.enable && config_.summary_csv_enable; }
  bool printEnabled() const { return config_.enable && config_.print_enable; }

  RuntimeStats & stats() { return stats_; }
  const RuntimeStats & stats() const { return stats_; }
  void resetStats() { stats_ = RuntimeStats{}; }

  ScopedTimer scoped(double RuntimeStats::*field) { return ScopedTimer(this, field); }

  void recordCloudCallback(double stamp, double points, double queue_delay_ms, double convert_time_ms);
  void recordCloudConvertTime(double convert_time_ms);
  void recordCloudDropEmpty();
  void recordCloudDropNoOdom();
  void recordCloudDropOdomTimeout();
  void recordValidCloud(double odom_age_ms);
  void recordOdom(double stamp);
  /// 可视化快照：本次构建了几次、因限频跳过几次。限频跳过的次数远多于构建次数说明
  /// visualization.rate 明显低于点云频率，省下来的正是重复构建的开销。
  void recordVizFrameBuilt();
  void recordVizFrameSkipped();
  /// 记录最近一次可视化快照构建耗时（ms）。写入 updateWorkerLoop 所在线程，
  /// 由下一次 fillInputStats() 带进 CSV，因此 CSV 里该列可能落后一行，按 1 s 窗口看无影响。
  void recordVizFrameTime(double ms)
  {
    if (!enabled()) {
      return;
    }
    last_viz_time_ms_ = ms;
  }
  void fillInputStats(RuntimeStats & stats);
  void observeUpdate(const RuntimeStats & stats);
  void close();

private:
  struct WindowAccumulator
  {
    double start_stamp{0.0};
    double last_stamp{0.0};
    double cloud_callbacks{0.0};
    double valid_updates{0.0};
    double field_updates{0.0};
    double update_count{0.0};
  };

  void addElapsed(double RuntimeStats::*field, double elapsed_ms);
  void writeDetailedHeader();
  void writeSummaryHeader();
  void writeDetailedRow(const RuntimeStats & stats);
  void maybeWriteSummary(double stamp);
  void resetWindow(double stamp);
  void flushIfNeeded(std::ofstream & stream, int & row_count);
  std::string sanitize(const std::string & value) const;

  PerformanceConfig config_{};
  RuntimeStats stats_{};
  std::ofstream detailed_csv_;
  std::ofstream summary_csv_;
  int detailed_csv_rows_{0};
  int summary_csv_rows_{0};
  double first_cloud_stamp_{0.0};
  double last_cloud_stamp_{0.0};
  double cloud_callback_count_{0.0};
  double cloud_points_sum_{0.0};
  double cloud_points_max_{0.0};
  double last_cloud_points_{0.0};
  double last_cloud_queue_delay_ms_{0.0};
  double last_cloud_convert_time_ms_{0.0};
  double valid_cloud_count_{0.0};
  double dropped_cloud_empty_count_{0.0};
  double dropped_cloud_no_odom_count_{0.0};
  double dropped_cloud_odom_timeout_count_{0.0};
  double first_odom_stamp_{0.0};
  double last_odom_stamp_{0.0};
  double odom_received_count_{0.0};
  double last_odom_age_ms_{0.0};
  double first_valid_update_stamp_{0.0};
  double last_valid_update_stamp_{0.0};
  /// 上一次 observeUpdate 的 steady_clock 纳秒值，用于算 update_period_ms。
  long long last_observe_ns_{0};
  /// 最近一次可视化快照构建耗时（ms，见 recordVizFrameTime）与窗口内计数。
  double last_viz_time_ms_{0.0};
  /// 窗口累计（不清零）与「自上次取走后新增」（取走即清零）两份计数。
  double viz_built_total_{0.0};
  double viz_skipped_total_{0.0};
  double viz_built_since_take_{0.0};
  double viz_skipped_since_take_{0.0};
  WindowAccumulator window_{};
  std::mutex mutex_;
};

}  // namespace rog_map
