/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "parameters.h"
#include "util/voxelgrid_sampling.h"

namespace small_point_lio {

    class Preprocess {
    public:
        Parameters *parameters = nullptr;
        std::deque<common::Point> point_deque;
        std::deque<common::Point> dense_point_deque;
        // 与 dense_point_deque 同序，记录每次雷达输入还未去畸变的有效点数。
        std::deque<size_t> dense_frame_remaining;
        std::deque<common::ImuMsg> imu_deque;

    private:
        double last_timestamp_lidar = -1;
        double last_timestamp_dense_point = -1;
        double last_timestamp_imu = -1;
        AdaptiveSampling adaptive_sampler;
        std::vector<double> sampled_ranges;
        util::VoxelgridSampling downsampler;
        std::vector<common::Point> filtered_points;
        std::vector<common::Point> dense_points;
        std::vector<common::Point> processed_pointcloud;

    public:
        Preprocess() = default;

        // Leaf used by the next input; adaptive sampling is always active when downsampling.
        double sampling_leaf_size() {
            return adaptive_sampler.leaf(parameters->space_downsample_leaf_size, parameters->adaptive_sampling);
        }
        void reset();
        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);
        void on_imu_callback(const common::ImuMsg &imu_msg);
        // 调用方去畸变并取用 front() 后调用；返回该点是否结束一帧雷达输入。
        bool finish_dense_point();
    };

}// namespace small_point_lio
