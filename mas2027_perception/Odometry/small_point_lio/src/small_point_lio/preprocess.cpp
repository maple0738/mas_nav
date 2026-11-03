/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "preprocess.h"
#include "parameters.h"

namespace small_point_lio {

    void Preprocess::reset() {
        adaptive_sampler.reset();
        sampled_ranges.clear();
        imu_deque.clear();
        point_deque.clear();
        dense_point_deque.clear();
        dense_frame_remaining.clear();
        last_timestamp_lidar = -1;
        last_timestamp_imu = -1;
        last_timestamp_dense_point = -1;
    }

    void Preprocess::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        dense_points.clear();
        dense_points.reserve(pointcloud.size());
        filtered_points.clear();
        filtered_points.reserve(pointcloud.size());
        const bool adaptive = parameters->space_downsample;
        double sampling_begin = std::numeric_limits<double>::infinity();
        double sampling_end = -std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < pointcloud.size(); i++) {
            const auto &point = pointcloud[i];
            // Measure acquisition span before range/stride filtering, without another input pass.
            if (adaptive && std::isfinite(point.timestamp) && point.timestamp >= last_timestamp_lidar) {
                sampling_begin = std::min(sampling_begin, point.timestamp);
                sampling_end = std::max(sampling_end, point.timestamp);
            }
            // 盲区（min_distance）以 blind_center 为球心量，量程（max_distance）仍以雷达原点量：
            // 前者要贴合车身，后者是传感器能力。blind_center 默认 (0,0,0) 时两者同心，行为不变。
            const float dist_from_blind_center = (point.position - parameters->blind_center).squaredNorm();
            const float dist_from_lidar = point.position.squaredNorm();
            const bool in_distance_range =
                    dist_from_blind_center >= parameters->min_distance_squared &&
                    dist_from_lidar <= parameters->max_distance_squared;
            // dense_points 只用于发布 /cloud_registered_full（及兼容话题 /cloud_registered），同样要过滤半径，
            // 否则车体自身（雷达上方的云台）会进入地形分析并被标成障碍
            if (point.timestamp >= last_timestamp_dense_point && in_distance_range) {
                dense_points.push_back(point);
            }
            if (i % parameters->point_filter_num != 0) {
                continue;
            }
            if (point.timestamp < last_timestamp_lidar) {
                continue;
            }
            if (!in_distance_range) {
                continue;
            }
            filtered_points.push_back(point);
        }
        if (parameters->space_downsample) {
            const double leaf = sampling_leaf_size();
            downsampler.voxelgrid_sampling(filtered_points, processed_pointcloud, leaf);
            if (!filtered_points.empty()) {
                sampled_ranges.clear();
                sampled_ranges.reserve(processed_pointcloud.size());
                for (const auto &point : processed_pointcloud) {
                    sampled_ranges.push_back(point.position.norm());
                }
                adaptive_sampler.observe(sampled_ranges, sampling_end - sampling_begin, parameters->adaptive_sampling);
                RCLCPP_DEBUG(rclcpp::get_logger("small_point_lio"),
                    "Adaptive sampling: input=%zu sampled=%zu target=%.1f leaf=%.3f next_leaf=%.3f",
                    filtered_points.size(), processed_pointcloud.size(), adaptive_sampler.target(),
                    leaf, sampling_leaf_size());
            }
        } else {
            processed_pointcloud = std::move(filtered_points);
        }
        sort(dense_points.begin(), dense_points.end(),
             [](const auto &x, const auto &y) {
                 return x.timestamp < y.timestamp;
             });
        sort(processed_pointcloud.begin(), processed_pointcloud.end(),
             [](const auto &x, const auto &y) {
                 return x.timestamp < y.timestamp;
             });
        // 水位线按本批全部点推进，不受半径过滤影响，避免下一批重复发布同一段点
        for (const auto &point: pointcloud) {
            if (point.timestamp > last_timestamp_dense_point) {
                last_timestamp_dense_point = point.timestamp;
            }
        }
        if (!dense_points.empty()) {
            dense_frame_remaining.push_back(dense_points.size());
            dense_point_deque.insert(dense_point_deque.end(), dense_points.begin(), dense_points.end());
        }
        if (!processed_pointcloud.empty()) {
            last_timestamp_lidar = processed_pointcloud.back().timestamp;
            point_deque.insert(point_deque.end(), processed_pointcloud.begin(), processed_pointcloud.end());
        }
    }

    void Preprocess::on_imu_callback(const common::ImuMsg &imu_msg) {
        if (imu_msg.timestamp < last_timestamp_imu) {
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "imu loop back");
            return;
        }
        imu_deque.emplace_back(imu_msg);
        last_timestamp_imu = imu_msg.timestamp;
    }

    bool Preprocess::finish_dense_point() {
        if (dense_point_deque.empty() || dense_frame_remaining.empty()) {
            throw std::logic_error("dense point/frame boundary mismatch");
        }
        dense_point_deque.pop_front();
        auto &remaining = dense_frame_remaining.front();
        if (--remaining == 0) {
            dense_frame_remaining.pop_front();
            return true;
        }
        return false;
    }

}// namespace small_point_lio
