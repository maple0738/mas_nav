#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace small_point_lio {

    struct AdaptiveTimingConfig {
        double min_interval = 0.001;
        double max_interval = 0.005;
        double normalized_error = 0.05;
        void validate() const {
            if (!std::isfinite(min_interval) || !std::isfinite(max_interval) ||
                !std::isfinite(normalized_error) || min_interval <= 0 ||
                max_interval < min_interval || max_interval > 0.02 || normalized_error <= 0) {
                throw std::invalid_argument("invalid adaptive timing bounds or residual error budget");
            }
        }
    };

    class AdaptiveTiming {
    public:
        using Information = Eigen::Matrix<double, 6, 6>;
        void reset() {
            interval_ = 0;
            last_stamp_ = -1;
            motion_ready_ = false;
            angular_acceleration_.setZero();
            last_gyro_.setZero();
        }
        double interval(double fallback, const AdaptiveTimingConfig &config) const {
            return interval_ > 0 ? interval_ : std::clamp(fallback, config.min_interval, config.max_interval);
        }
        void observeImu(double stamp, const Eigen::Vector3d &gyro, bool valid,
                        const AdaptiveTimingConfig &config) {
            const double dt = stamp - last_stamp_;
            if (!valid || !std::isfinite(stamp) || !gyro.allFinite()) {
                motion_ready_ = false;
                last_stamp_ = -1;
                interval_ = config.min_interval;
                return;
            }
            if (last_stamp_ >= 0 && dt > 0 && dt <= 0.1) {
                const auto derivative = ((gyro - last_gyro_) / dt).eval();
                const double alpha = 1 - std::exp(-dt / 0.02);
                angular_acceleration_ = motion_ready_ ? (angular_acceleration_ + alpha * (derivative - angular_acceleration_)).eval() : derivative;
                motion_ready_ = true;
            } else {
                motion_ready_ = false;
                interval_ = config.min_interval;
            }
            last_stamp_ = stamp;
            last_gyro_ = gyro;
        }
        void observeBatch(const Eigen::Vector3d &world_acceleration,
                          const Information &mean_information, std::size_t accepted,
                          double fallback, const AdaptiveTimingConfig &config) {
            if (!motion_ready_ || !world_acceleration.allFinite() || !mean_information.allFinite()) {
                interval_ = config.min_interval;
                return;
            }
            // Match h_point coordinates: world translation, IMU-local rotation.
            Eigen::Matrix<double, 6, 1> curvature;
            curvature << world_acceleration, angular_acceleration_;
            const double energy = curvature.dot(mean_information * curvature);
            if (!std::isfinite(energy) || energy < 0) {
                interval_ = config.min_interval;
                return;
            }
            double target = std::clamp(fallback, config.min_interval, config.max_interval);
            if (accepted > 0) {
                target = energy > 0 ? std::clamp(std::sqrt(std::sqrt(
                                                         4 * config.normalized_error * config.normalized_error / energy)),
                                                 config.min_interval, config.max_interval)
                                    : config.max_interval;
            }
            // Shorten immediately; grow slowly to avoid alternating short/long batches.
            interval_ = std::min(target, interval(fallback, config) * 1.2);
        }

    private:
        double interval_ = 0;
        double last_stamp_ = -1;
        bool motion_ready_ = false;
        Eigen::Vector3d last_gyro_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular_acceleration_ = Eigen::Vector3d::Zero();
    };
}// namespace small_point_lio
