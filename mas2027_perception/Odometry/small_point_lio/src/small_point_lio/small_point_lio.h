/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "estimator.h"
#include "parameters.h"
#include "preprocess.h"
#include "output_prediction.h"
#include <pch.h>

namespace small_point_lio {

    class SmallPointLio {
    private:
        Parameters parameters;
        Preprocess preprocess;
        Estimator estimator;
        AdaptiveTiming adaptive_timing;
        OutputPrediction output_prediction;
        double time_current = 0.0;
        double last_odom_publish_time = -std::numeric_limits<double>::infinity();
        std::vector<Eigen::Vector3f> pointcloud_odom_frame;
        std::function<void(const std::vector<Eigen::Vector3f> &pointcloud, double stamp)> pointcloud_callback;
        std::function<void(const common::Odometry &odometry)> odometry_callback;
        bool is_init = false;
        std::size_t point_batches_ = 0, accepted_points_ = 0;


    public:
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> Q;

        explicit SmallPointLio(rclcpp::Node &node);

        void reset();
        // Cumulative completed batch/match counters for profiling and health checks.
        std::size_t point_batches() const { return point_batches_; }
        std::size_t accepted_points() const { return accepted_points_; }
        std::size_t plane_cache_hits() const { return estimator.plane_cache.hits; }
        std::size_t plane_cache_misses() const { return estimator.plane_cache.misses; }


        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);

        void on_imu_callback(const common::ImuMsg &imu_msg);

        void handle_once();

        void set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud, double stamp)> &pointcloud_callback);

        void set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback);

    private:
        void publish_odometry(double timestamp);
        void publish_odometry(double timestamp, const state &snapshot);
    };

}// namespace small_point_lio
