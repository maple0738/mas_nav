#include <rog_map/performance_monitor.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace rog_map {

namespace {

double elapsedMs(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

long long steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double hzFrom(double count, double first_stamp, double last_stamp)
{
  const double dt = last_stamp - first_stamp;
  if (count <= 1.0 || dt <= 1.0e-6) {
    return 0.0;
  }
  return (count - 1.0) / dt;
}

std::string num(double value)
{
  if (!std::isfinite(value)) {
    return "NaN";
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6) << value;
  return ss.str();
}

std::string runCsvPath(const std::string & path, long long run_stamp_us)
{
  const size_t slash = path.find_last_of('/');
  const size_t dot = path.find_last_of('.');
  const size_t split = dot != std::string::npos &&
    (slash == std::string::npos || dot > slash) ? dot : path.size();
  return path.substr(0, split) + "-" + std::to_string(run_stamp_us) + path.substr(split);
}

void writeCsvLine(std::ofstream & stream, const std::vector<std::string> & fields)
{
  for (size_t i = 0; i < fields.size(); ++i) {
    stream << fields[i];
    if (i + 1 < fields.size()) {
      stream << ',';
    }
  }
  stream << '\n';
}

}  // namespace

PerformanceMonitor::ScopedTimer::ScopedTimer(PerformanceMonitor * monitor, double RuntimeStats::*field)
: monitor_(monitor), field_(field)
{
  if (monitor_ && monitor_->enabled() && field_) {
    start_ = std::chrono::steady_clock::now();
  } else {
    monitor_ = nullptr;
    field_ = nullptr;
  }
}

PerformanceMonitor::ScopedTimer::~ScopedTimer()
{
  if (!monitor_ || !field_) {
    return;
  }
  monitor_->addElapsed(field_, elapsedMs(start_));
}

void PerformanceMonitor::configure(const PerformanceConfig & config)
{
  close();
  config_ = config;
  if (config_.csv_flush_every_n <= 0) {
    config_.csv_flush_every_n = 30;
  }
  resetStats();
  resetWindow(0.0);
  detailed_csv_rows_ = 0;
  summary_csv_rows_ = 0;
  last_observe_ns_ = 0;

  if (!config_.enable) {
    return;
  }
  const auto run_stamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  if (detailedCsvEnabled()) {
    config_.detailed_csv_path = runCsvPath(config_.detailed_csv_path, run_stamp_us);
    detailed_csv_.open(config_.detailed_csv_path, std::ios::out | std::ios::app);
    if (!detailed_csv_.is_open()) {
      std::cerr << "[ROGMapPerf] failed to open detailed_csv_path: " << config_.detailed_csv_path
                << std::endl;
    } else {
      std::cout << "[ROGMapPerf] detailed CSV: " << config_.detailed_csv_path << std::endl;
      writeDetailedHeader();
    }
  }
  if (summaryCsvEnabled()) {
    config_.summary_csv_path = runCsvPath(config_.summary_csv_path, run_stamp_us);
    summary_csv_.open(config_.summary_csv_path, std::ios::out | std::ios::app);
    if (!summary_csv_.is_open()) {
      std::cerr << "[ROGMapPerf] failed to open summary_csv_path: " << config_.summary_csv_path
                << std::endl;
    } else {
      std::cout << "[ROGMapPerf] summary CSV: " << config_.summary_csv_path << std::endl;
      writeSummaryHeader();
    }
  }
}

void PerformanceMonitor::recordCloudCallback(
  double stamp, double points, double queue_delay_ms, double convert_time_ms)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (first_cloud_stamp_ <= 0.0) {
    first_cloud_stamp_ = stamp;
  }
  last_cloud_stamp_ = stamp;
  cloud_callback_count_ += 1.0;
  cloud_points_sum_ += points;
  cloud_points_max_ = std::max(cloud_points_max_, points);
  last_cloud_points_ = points;
  last_cloud_queue_delay_ms_ = queue_delay_ms;
  last_cloud_convert_time_ms_ = convert_time_ms;
  if (window_.start_stamp <= 0.0) {
    resetWindow(stamp);
  }
  window_.cloud_callbacks += 1.0;
  window_.last_stamp = stamp;
}

void PerformanceMonitor::recordCloudConvertTime(double convert_time_ms)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  last_cloud_convert_time_ms_ = convert_time_ms;
}

void PerformanceMonitor::recordCloudDropEmpty()
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  dropped_cloud_empty_count_ += 1.0;
}

void PerformanceMonitor::recordCloudDropNoOdom()
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  dropped_cloud_no_odom_count_ += 1.0;
}

void PerformanceMonitor::recordCloudDropOdomTimeout()
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  dropped_cloud_odom_timeout_count_ += 1.0;
}

void PerformanceMonitor::recordValidCloud(double odom_age_ms)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  valid_cloud_count_ += 1.0;
  last_odom_age_ms_ = odom_age_ms;
  const double stamp = last_cloud_stamp_ > 0.0 ? last_cloud_stamp_ : last_odom_stamp_;
  if (first_valid_update_stamp_ <= 0.0) {
    first_valid_update_stamp_ = stamp;
  }
  last_valid_update_stamp_ = stamp;
}

void PerformanceMonitor::recordVizFrameBuilt()
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  viz_built_total_ += 1.0;
  viz_built_since_take_ += 1.0;
}

void PerformanceMonitor::recordVizFrameSkipped()
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  viz_skipped_total_ += 1.0;
  viz_skipped_since_take_ += 1.0;
}

void PerformanceMonitor::recordOdom(double stamp)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (first_odom_stamp_ <= 0.0) {
    first_odom_stamp_ = stamp;
  }
  last_odom_stamp_ = stamp;
  odom_received_count_ += 1.0;
}

void PerformanceMonitor::fillInputStats(RuntimeStats & stats)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  stats.cloud_callback_count = cloud_callback_count_;
  stats.cloud_callback_hz = hzFrom(cloud_callback_count_, first_cloud_stamp_, last_cloud_stamp_);
  stats.cloud_msg_points = last_cloud_points_;
  stats.cloud_msg_points_avg =
    cloud_callback_count_ > 0.0 ? cloud_points_sum_ / cloud_callback_count_ : 0.0;
  stats.cloud_msg_points_max = cloud_points_max_;
  stats.cloud_convert_time_ms = last_cloud_convert_time_ms_;
  stats.cloud_queue_delay_ms = last_cloud_queue_delay_ms_;
  stats.valid_cloud_count = valid_cloud_count_;
  stats.valid_update_hz = hzFrom(valid_cloud_count_, first_valid_update_stamp_, last_valid_update_stamp_);
  stats.dropped_cloud_empty_count = dropped_cloud_empty_count_;
  stats.dropped_cloud_no_odom_count = dropped_cloud_no_odom_count_;
  stats.dropped_cloud_odom_timeout_count = dropped_cloud_odom_timeout_count_;
  stats.odom_received_count = odom_received_count_;
  stats.odom_hz = hzFrom(odom_received_count_, first_odom_stamp_, last_odom_stamp_);
  stats.odom_age_ms = last_odom_age_ms_;
  stats.last_viz_time_ms = last_viz_time_ms_;
  // 取走「自上次取走后新增」的次数：fillInputStats 在每次更新开头都会被调用，
  // 用增量才能让 detailed CSV 每行显示的就是那一次更新对应的构建/跳过情况。
  stats.viz_built_count = viz_built_since_take_;
  stats.viz_skipped_count = viz_skipped_since_take_;
  viz_built_since_take_ = 0.0;
  viz_skipped_since_take_ = 0.0;
}

void PerformanceMonitor::observeUpdate(const RuntimeStats & raw_stats)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  RuntimeStats stats = raw_stats;
  // 真实更新周期：从上次更新结束到这次更新结束的墙钟差。total_update_time 只覆盖
  // updateMapInternal，不含其后的可视化快照构建（captureVizFrame），两者之差即那段不可见开销。
  const long long now_ns = steadyNowNs();
  if (last_observe_ns_ > 0) {
    const double period_ms = static_cast<double>(now_ns - last_observe_ns_) / 1.0e6;
    stats.update_period_ms = period_ms;
    stats.update_unaccounted_ms = period_ms - stats.total_update_time;
  }
  last_observe_ns_ = now_ns;
  stats_ = stats;
  if (first_valid_update_stamp_ <= 0.0) {
    first_valid_update_stamp_ = stats.stamp;
  }
  last_valid_update_stamp_ = stats.stamp;
  if (window_.start_stamp <= 0.0) {
    resetWindow(stats.stamp);
  }
  window_.last_stamp = stats.stamp;
  window_.valid_updates += 1.0;
  window_.field_updates += stats.field_actual_update != 0.0 ? 1.0 : 0.0;
  window_.update_count += 1.0;

  if (detailedCsvEnabled() && detailed_csv_.is_open()) {
    writeDetailedRow(stats);
  }
  maybeWriteSummary(stats.stamp);
}

void PerformanceMonitor::close()
{
  if (detailed_csv_.is_open()) {
    detailed_csv_.flush();
    detailed_csv_.close();
  }
  if (summary_csv_.is_open()) {
    summary_csv_.flush();
    summary_csv_.close();
  }
}

void PerformanceMonitor::addElapsed(double RuntimeStats::*field, double elapsed_ms)
{
  if (!field) {
    return;
  }
  stats_.*field += elapsed_ms;
}

void PerformanceMonitor::writeDetailedHeader()
{
  writeCsvLine(detailed_csv_,
    {"run_id",
      "scenario",
      "variant",
      "stamp_ros",
      "stamp_steady_ns",
      "cloud_callback_hz",
      "map_update_hz",
      "input_points",
      "cloud_convert_time_ms",
      "raycast_time_ms",
      "prob_update_time_ms",
      "decay_time_ms",
      "projection_time_ms",
      "field_time_ms",
      "snapshot_time_ms",
      "total_update_time_ms",
      "free_count",
      "passable_count",
      "occupied_count",
      "unknown_count",
      "reason_insufficient_observation",
      "reason_empty_column",
      "reason_thin_surface",
      "reason_solid_vertical_wall",
      "reason_hollow_tunnel",
      "reason_ambiguous_occupied",
      "raycast_parallel_time_ms",
      "raycast_merge_time_ms",
      "cache_count",
      "hit_count",
      "miss_count",
      "active_cell_count",
      "dirty_column_count",
      "dirty_expanded_column_count",
      "dirty_column_enabled",
      "full_reason_geometry",
      "full_reason_explicit",
      "full_reason_dirty_disabled",
      "full_reason_dirty_over_ratio",
      "full_reason_periodic",
      "full_layer_flag_at_consume",
      "mark_dirty_ok_count",
      "mark_dirty_out_of_map_count",
      "mark_dirty_invalid_count",
      "layer_config_sync_time_ms",
      "layer_dirty_merge_time_ms",
      "projection_update_full_time_ms",
      "projection_update_dirty_time_ms",
      "projection_mask_filter_time_ms",
      "projection_value_mask_time_ms",
      "projection_count_cells_time_ms",
      "fused_projection_time_ms",
      "projection_scanned_voxel_estimate",
      "projection_z_layers",
      "update_robot_state_time_ms",
      "update_period_ms",
      "update_unaccounted_ms",
      "last_viz_time_ms",
      "viz_built_count",
      "viz_skipped_count",
      "terrain_label_flat",
      "terrain_label_slope",
      "terrain_label_tunnel"});
}

void PerformanceMonitor::writeSummaryHeader()
{
  writeCsvLine(summary_csv_,
    {"run_id",
      "scenario",
      "variant",
      "window_start_stamp",
      "window_duration_sec",
      "cloud_callback_hz",
      "map_update_hz",
      "field_update_hz",
      "last_total_update_time_ms",
      "last_raycast_time_ms",
      "last_projection_time_ms",
      "last_field_time_ms",
      "last_prob_update_time_ms",
      "last_decay_time_ms",
      "last_query_refresh_time_ms",
      "last_update_period_ms",
      "last_update_unaccounted_ms",
      "last_viz_time_ms",
      "last_viz_built",
      "last_viz_skipped",
      "last_full_reason_dirty_over_ratio",
      "last_full_reason_periodic",
      "last_full_reason_explicit",
      "last_full_reason_geometry",
      "last_dirty_column_count",
      "last_projection_z_layers",
      "last_free_count",
      "last_passable_count",
      "last_occupied_count",
      "last_unknown_count",
      "last_terrain_label_flat",
      "last_terrain_label_slope",
      "last_terrain_label_tunnel"});
}

void PerformanceMonitor::writeDetailedRow(const RuntimeStats & s)
{
  writeCsvLine(detailed_csv_,
    {sanitize(config_.run_id),
      sanitize(config_.scenario),
      sanitize(config_.variant),
      num(s.stamp),
      std::to_string(steadyNowNs()),
      num(s.cloud_callback_hz),
      num(s.valid_update_hz),
      num(s.input_point_count),
      num(s.cloud_convert_time_ms),
      num(s.raycast_time),
      num(s.prob_update_time),
      num(s.decay_time),
      num(s.projection_total_time),
      num(s.field_time),
      num(s.query_refresh_time),
      num(s.total_update_time),
      num(s.free_count),
      num(s.passable_count),
      num(s.occupied_count),
      num(s.unknown_count),
      num(s.projection_insufficient_observation_count),
      num(s.projection_empty_column_count),
      num(s.projection_thin_surface_count),
      num(s.projection_vertical_wall_count),
      num(s.projection_hollow_tunnel_count),
      num(s.projection_ambiguous_occupied_count),
      num(s.raycast_parallel_time),
      num(s.raycast_merge_time),
      num(s.cache_count),
      num(s.hit_count),
      num(s.miss_count),
      num(s.active_cell_count),
      num(s.dirty_column_count),
      num(s.dirty_expanded_column_count),
      num(s.projection_dirty_column_enabled),
      num(s.full_reason_geometry),
      num(s.full_reason_explicit),
      num(s.full_reason_dirty_disabled),
      num(s.full_reason_dirty_over_ratio),
      num(s.full_reason_periodic),
      num(s.full_layer_flag_at_consume),
      num(s.mark_dirty_ok_count),
      num(s.mark_dirty_out_of_map_count),
      num(s.mark_dirty_invalid_count),
      num(s.layer_config_sync_time),
      num(s.layer_dirty_merge_time),
      num(s.projection_update_full_time),
      num(s.projection_update_dirty_time),
      num(s.projection_mask_filter_time),
      num(s.projection_value_mask_time),
      num(s.projection_count_cells_time),
      num(s.fused_projection_time),
      num(s.projection_scanned_voxel_estimate),
      num(s.projection_z_layers),
      num(s.update_robot_state_time),
      num(s.update_period_ms),
      num(s.update_unaccounted_ms),
      num(s.last_viz_time_ms),
      num(s.viz_built_count),
      num(s.viz_skipped_count),
      num(s.terrain_label_flat_count),
      num(s.terrain_label_slope_count),
      num(s.terrain_label_tunnel_count)});
  flushIfNeeded(detailed_csv_, detailed_csv_rows_);
}

void PerformanceMonitor::maybeWriteSummary(double stamp)
{
  if (window_.start_stamp <= 0.0) {
    resetWindow(stamp);
    return;
  }
  const double period = config_.summary_rate > 0.0 ? 1.0 / config_.summary_rate : 1.0;
  const double duration = std::max(0.0, stamp - window_.start_stamp);
  if (duration + 1.0e-9 < period) {
    return;
  }
  const double cloud_hz = duration > 1.0e-6 ? window_.cloud_callbacks / duration : 0.0;
  const double valid_hz = duration > 1.0e-6 ? window_.valid_updates / duration : 0.0;
  const double field_hz = duration > 1.0e-6 ? window_.field_updates / duration : 0.0;

  if (summaryCsvEnabled() && summary_csv_.is_open()) {
    writeCsvLine(summary_csv_,
      {sanitize(config_.run_id),
        sanitize(config_.scenario),
        sanitize(config_.variant),
        num(window_.start_stamp),
        num(duration),
        num(cloud_hz),
        num(valid_hz),
        num(field_hz),
        num(stats_.total_update_time),
        num(stats_.raycast_time),
        num(stats_.projection_total_time),
        num(stats_.field_time),
        // summary 补齐 prob_update/decay/query 与「周期 − 自报」两段：原先只报三段
        // 耗时，这几项排障时不可见；补上后未开 detailed CSV 的运行也能留下可判读数据。
        num(stats_.prob_update_time),
        num(stats_.decay_time),
        num(stats_.query_refresh_time),
        num(stats_.update_period_ms),
        num(stats_.update_unaccounted_ms),
        num(stats_.last_viz_time_ms),
        num(stats_.viz_built_count),
        num(stats_.viz_skipped_count),
        num(stats_.full_reason_dirty_over_ratio),
        num(stats_.full_reason_periodic),
        num(stats_.full_reason_explicit),
        num(stats_.full_reason_geometry),
        num(stats_.dirty_column_count),
        num(stats_.projection_z_layers),
        num(stats_.free_count),
        num(stats_.passable_count),
        num(stats_.occupied_count),
        num(stats_.unknown_count),
        num(stats_.terrain_label_flat_count),
        num(stats_.terrain_label_slope_count),
        num(stats_.terrain_label_tunnel_count)});
    flushIfNeeded(summary_csv_, summary_csv_rows_);
  }

  if (printEnabled()) {
    std::cerr << "[ROGMapPerf] win=" << std::fixed << std::setprecision(2) << duration
              << "s cloud_cb_hz=" << cloud_hz << " map_update_hz=" << valid_hz
              << "Hz field_update=" << field_hz << "Hz\n"
              << "  time_ms: total=" << stats_.total_update_time
              << " raycast=" << stats_.raycast_time
              << " projection=" << stats_.projection_total_time
              << " field=" << stats_.field_time
              << '\n'
              << "  cells: free=" << stats_.free_count
              << " passable=" << stats_.passable_count
              << " occupied=" << stats_.occupied_count
              << " unknown=" << stats_.unknown_count << '\n'
              << "  classification: thin=" << stats_.projection_thin_surface_count
              << " solid_wall=" << stats_.projection_vertical_wall_count
              << " hollow_tunnel=" << stats_.projection_hollow_tunnel_count
              << " ambiguous=" << stats_.projection_ambiguous_occupied_count
              << " empty=" << stats_.projection_empty_column_count
              << " insufficient=" << stats_.projection_insufficient_observation_count << std::endl;
  }
  resetWindow(stamp);
}

void PerformanceMonitor::resetWindow(double stamp)
{
  window_ = WindowAccumulator{};
  window_.start_stamp = stamp;
  window_.last_stamp = stamp;
}

void PerformanceMonitor::flushIfNeeded(std::ofstream & stream, int & row_count)
{
  ++row_count;
  if (config_.csv_flush_every_n > 0 && row_count % config_.csv_flush_every_n == 0) {
    stream.flush();
  }
}

std::string PerformanceMonitor::sanitize(const std::string & value) const
{
  std::string out = value;
  std::replace(out.begin(), out.end(), ',', '_');
  return out;
}

}  // namespace rog_map
