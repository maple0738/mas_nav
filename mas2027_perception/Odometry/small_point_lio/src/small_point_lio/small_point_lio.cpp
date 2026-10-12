/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio.h"

namespace small_point_lio {

    SmallPointLio::SmallPointLio(rclcpp::Node &node) {
        // init param
        parameters.read_parameters(node);
        preprocess.parameters = &parameters;
        estimator.parameters = &parameters;
        estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T.cast<state::value_type>();
        estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R.cast<state::value_type>();
        if (parameters.extrinsic_est_en) {
            estimator.kf.x.offset_T_L_I = parameters.extrinsic_T.cast<state::value_type>();
            estimator.kf.x.offset_R_L_I = parameters.extrinsic_R.cast<state::value_type>();
        }
        Q = estimator.process_noise_cov();
        estimator.imu_acceleration_scale = parameters.gravity.norm() / parameters.acc_norm;

        // init data
        reset();
    }

    void SmallPointLio::reset() {
        preprocess.reset();
        estimator.reset();
        pointcloud_odom_frame.clear();
        last_odom_publish_time = -std::numeric_limits<double>::infinity();
        is_init = false;
    }

    void SmallPointLio::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        preprocess.on_imu_callback(imu_msg);
    }

    void SmallPointLio::handle_once() {
        // we need to init small point lio
        if (!is_init) {
            if ((!preprocess.point_deque.empty() || !preprocess.imu_deque.empty()) &&
                preprocess.point_deque.size() >= parameters.init_map_size &&
                (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // init map
                for (const auto &point: preprocess.point_deque) {
                    estimator.ivox->add_point(point.position);
                }
                // fix gravity direction
                if (parameters.fix_gravity_direction) {
                    estimator.kf.x.gravity = Eigen::Matrix<state::value_type, 3, 1>::Zero();
                    for (const auto &imu_msg: preprocess.imu_deque) {
                        estimator.kf.x.gravity += imu_msg.linear_acceleration.cast<state::value_type>();
                    }
                    state::value_type scale = -static_cast<state::value_type>(parameters.gravity.norm()) / estimator.kf.x.gravity.norm();
                    estimator.kf.x.gravity *= scale;
                } else {
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                }
                estimator.kf.x.acceleration = -estimator.kf.x.gravity;
                // init time
                if (preprocess.point_deque.empty()) {
                    time_current = preprocess.imu_deque.back().timestamp;
                } else if (preprocess.imu_deque.empty()) {
                    time_current = preprocess.point_deque.back().timestamp;
                } else {
                    time_current = std::max(preprocess.point_deque.back().timestamp, preprocess.imu_deque.back().timestamp);
                }
                estimator.kf.init_timestamp(time_current);
                // clear data
                preprocess.point_deque.clear();
                preprocess.dense_point_deque.clear();
                preprocess.dense_frame_remaining.clear();
                preprocess.imu_deque.clear();
                is_init = true;
            }
            return;
        }

        // judge we should do point update or imu update
        bool is_publish_odometry = !preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty() &&
                                   preprocess.imu_deque.front().timestamp < preprocess.point_deque.back().timestamp;
        std::vector<std::pair<double, std::vector<Eigen::Vector3f>>> completed_clouds;
        std::vector<common::Point> point_batch;
        point_batch.reserve(parameters.point_update_batch_max_points);
        const auto flush_point_batch = [this, &point_batch]() {
            if (point_batch.empty()) {
                return;
            }
            // Move every observation to the batch end using the predicted state.
            // The EKF then linearizes the entire short window at one state.
            const auto &s = estimator.kf.x;
            const auto rotation_lidar_imu = parameters.extrinsic_est_en ? s.offset_R_L_I : estimator.Lidar_R_wrt_IMU;
            const auto translation_lidar_imu = parameters.extrinsic_est_en ? s.offset_T_L_I : estimator.Lidar_T_wrt_IMU;
            const Eigen::Vector3d world_acceleration = s.rotation * s.acceleration + s.gravity;
            std::vector<Eigen::Vector3f> compensated;
            compensated.reserve(point_batch.size());
            for (const auto &point : point_batch) {
                const double dt = point.timestamp - estimator.kf.time_state();
                const Eigen::Vector3d point_imu = rotation_lidar_imu * point.position.cast<double>() + translation_lidar_imu;
                const Eigen::Vector3d displacement = s.velocity * dt + 0.5 * world_acceleration * dt * dt;
                const Eigen::Vector3d point_imu_at_end =
                    exp<double>(s.omg * dt) * point_imu + s.rotation.transpose() * displacement;
                compensated.push_back((rotation_lidar_imu.transpose() *
                    (point_imu_at_end - translation_lidar_imu)).cast<float>());
            }
            estimator.kf.update_point_batch(compensated.size(), [this, &compensated](std::size_t i) {
                estimator.point_lidar_frame = compensated[i];
            });
            const auto &updated = estimator.kf.x;
            const Eigen::Matrix3d updated_rotation_lidar_imu = parameters.extrinsic_est_en ?
                updated.offset_R_L_I : estimator.Lidar_R_wrt_IMU;
            const Eigen::Vector3d updated_translation_lidar_imu = parameters.extrinsic_est_en ?
                updated.offset_T_L_I : estimator.Lidar_T_wrt_IMU;
            for (const auto &point : compensated) {
                const Eigen::Vector3d point_imu = updated_rotation_lidar_imu * point.cast<double>() + updated_translation_lidar_imu;
                estimator.ivox->add_point((updated.rotation * point_imu + updated.position).cast<float>());
            }
            if (parameters.publish_odometry_without_downsample &&
                time_current - last_odom_publish_time >= parameters.odom_publish_interval_s - 1e-9) {
                publish_odometry(time_current);
                last_odom_publish_time = time_current;
            }
            point_batch.clear();
        };
        while (!preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty()) {
            const common::Point &point_lidar_frame = preprocess.point_deque.front();
            const common::Point &dense_point_lidar_frame = preprocess.dense_point_deque.front();
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            if (dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp && dense_point_lidar_frame.timestamp < imu_msg.timestamp) {
                // collect odom frame pointcloud
                Eigen::Matrix<state::value_type, 3, 1> dense_point_imu_frame;
                if (parameters.extrinsic_est_en) {
                    dense_point_imu_frame = estimator.kf.x.offset_R_L_I * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                } else {
                    dense_point_imu_frame = estimator.Lidar_R_wrt_IMU * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                }
                // 去畸变：状态仅在 kf.time_state() 有效，稠密点直接套用会把运动烙进点云（拖影）。
                // 故只读快照前推到点采样时刻，不调 predict_state，避免多插积分步扰动 eskf 估计。
                const auto dt_deskew = static_cast<state::value_type>(dense_point_lidar_frame.timestamp - estimator.kf.time_state());
                if (dt_deskew > 0) {
                    const Eigen::Matrix<state::value_type, 3, 3> rotation_deskew =
                            estimator.kf.x.rotation * exp<state::value_type>(estimator.kf.x.omg * dt_deskew);
                    const Eigen::Matrix<state::value_type, 3, 1> position_deskew =
                            estimator.kf.x.position + estimator.kf.x.velocity * dt_deskew;
                    pointcloud_odom_frame.emplace_back((rotation_deskew * dense_point_imu_frame + position_deskew).cast<float>());
                } else {
                    // dt <= 0：点的时间戳不晚于状态时刻，无需前推（与 predict_state 的 dt>0 门限保持一致）
                    pointcloud_odom_frame.emplace_back((estimator.kf.x.rotation * dense_point_imu_frame + estimator.kf.x.position).cast<float>());
                }

                const double point_stamp = dense_point_lidar_frame.timestamp;
                if (preprocess.finish_dense_point()) {
                    completed_clouds.emplace_back(point_stamp, std::vector<Eigen::Vector3f>{});
                    completed_clouds.back().second.swap(pointcloud_odom_frame);
                }
            } else if (point_lidar_frame.timestamp < imu_msg.timestamp) {
                // point update
                if (point_lidar_frame.timestamp < time_current) {
                    preprocess.point_deque.pop_front();
                    continue;
                }
                time_current = point_lidar_frame.timestamp;

                // predict
                estimator.kf.predict_state(time_current);

                point_batch.push_back(point_lidar_frame);
                if (point_batch.size() >= parameters.point_update_batch_max_points ||
                    time_current - point_batch.front().timestamp >= parameters.point_update_batch_interval_s) {
                    flush_point_batch();
                }
                preprocess.point_deque.pop_front();
            } else {
                // imu update
                flush_point_batch();
                if (imu_msg.timestamp < time_current) {
                    preprocess.imu_deque.pop_front();
                    continue;
                }
                time_current = imu_msg.timestamp;

                // predict
                estimator.kf.predict_state(time_current);
                estimator.kf.predict_cov(time_current, Q);

                // update
                estimator.angular_velocity = imu_msg.angular_velocity.cast<state::value_type>();
                estimator.linear_acceleration = imu_msg.linear_acceleration.cast<state::value_type>();
                estimator.kf.update_imu();

                preprocess.imu_deque.pop_front();
            }
        }

        flush_point_batch();

        if (is_publish_odometry || !completed_clouds.empty()) {
            if (!parameters.publish_odometry_without_downsample) {
                publish_odometry(time_current);
            }
            if (pointcloud_callback) {
                for (const auto &[stamp, cloud] : completed_clouds) {
                    pointcloud_callback(cloud, stamp);
                }
            }
        }
    }

    void SmallPointLio::set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud, double stamp)> &pointcloud_callback) {
        this->pointcloud_callback = pointcloud_callback;
    }

    void SmallPointLio::set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback) {
        this->odometry_callback = odometry_callback;
    }

    void SmallPointLio::publish_odometry(double timestamp) {
        if (odometry_callback) {
            common::Odometry odometry;
            odometry.timestamp = timestamp;
            odometry.position = estimator.kf.x.position.cast<double>();
            odometry.velocity = estimator.kf.x.velocity.cast<double>();
            odometry.orientation = estimator.kf.x.rotation.cast<double>();
            odometry.angular_velocity = estimator.kf.x.omg.cast<double>();
            odometry.gravity = estimator.kf.x.gravity.cast<double>();
            if (parameters.extrinsic_est_en) {
                odometry.translation_lidar_imu_from_lidar_link = estimator.kf.x.offset_T_L_I.cast<double>();
                odometry.rotation_lidar_imu_from_lidar_link = estimator.kf.x.offset_R_L_I.cast<double>();
            } else {
                odometry.translation_lidar_imu_from_lidar_link = estimator.Lidar_T_wrt_IMU.cast<double>();
                odometry.rotation_lidar_imu_from_lidar_link = estimator.Lidar_R_wrt_IMU.cast<double>();
            }
            odometry_callback(odometry);
        }
    }

}// namespace small_point_lio
