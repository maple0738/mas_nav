#pragma once
#include "eskf.h"
#include "common/common.h"
#include <deque>

namespace small_point_lio {
// Independent output state: corrected from the estimator, never fed back to it.
class OutputPrediction {
public:
    void reset() { history_.clear(); ready_ = false; time_ = 0; }
    bool add(const common::ImuMsg &sample, double scale, bool check, double gyro_limit, double accel_limit) {
        if (!std::isfinite(sample.timestamp) || !sample.angular_velocity.allFinite() ||
            !sample.linear_acceleration.allFinite()) { reset(); return false; }
        if (!history_.empty() && sample.timestamp <= history_.back().timestamp) return false;
        if (!history_.empty() && sample.timestamp - history_.back().timestamp > .1) reset();
        history_.push_back(sample);
        while (history_.size() > 1000 || history_.back().timestamp - history_.front().timestamp > 2.)
            history_.pop_front();
        if (ready_ && sample.timestamp - posterior_time_ > .15) ready_ = false;
        if (ready_) advance(scale, check, gyro_limit, accel_limit);
        return ready_;
    }
    bool correct(const state &posterior, double stamp, double scale, bool check,
                 double gyro_limit, double accel_limit) {
        if (!std::isfinite(stamp) || !posterior.position.allFinite() || !posterior.rotation.allFinite() ||
            !posterior.velocity.allFinite() || !posterior.gravity.allFinite() || !posterior.bg.allFinite() ||
            !posterior.ba.allFinite() || !posterior.omg.allFinite() || !posterior.acceleration.allFinite() ||
            history_.empty() || stamp < history_.front().timestamp || stamp > history_.back().timestamp ||
            history_.back().timestamp - stamp > .15) {
            ready_ = false;
            return false;
        }
        predicted_ = posterior; time_ = stamp; posterior_time_ = stamp; ready_ = true;
        advance(scale, check, gyro_limit, accel_limit);
        return true;
    }
    const state &value() const { return predicted_; }
    double time() const { return time_; }
private:
    void advance(double scale, bool check, double gyro_limit, double accel_limit) {
        for (std::size_t i = 1; i < history_.size(); ++i) {
            const auto &left = history_[i - 1], &right = history_[i];
            if (right.timestamp <= time_) continue;
            const double start = std::max(time_, left.timestamp), dt = right.timestamp - start;
            if (dt <= 0) continue;
            const double fraction = (start - left.timestamp) / (right.timestamp - left.timestamp);
            const Eigen::Vector3d gyro = .5 * ((1 - fraction) * left.angular_velocity + (1 + fraction) * right.angular_velocity);
            const Eigen::Vector3d accel = .5 * ((1 - fraction) * left.linear_acceleration + (1 + fraction) * right.linear_acceleration);
            Eigen::Vector3d omega, specific;
            for (int j = 0; j < 3; ++j) {
                omega[j] = check && (std::abs(left.angular_velocity[j]) >= gyro_limit || std::abs(right.angular_velocity[j]) >= gyro_limit) ?
                    predicted_.omg[j] : gyro[j] - predicted_.bg[j];
                specific[j] = check && (std::abs(left.linear_acceleration[j]) >= accel_limit || std::abs(right.linear_acceleration[j]) >= accel_limit) ?
                    predicted_.acceleration[j] : scale * accel[j] - predicted_.ba[j];
            }
            const Eigen::Matrix3d midpoint = predicted_.rotation * exp<double>(omega * (dt * .5));
            const Eigen::Vector3d acceleration = midpoint * specific + predicted_.gravity;
            predicted_.position += predicted_.velocity * dt + .5 * acceleration * dt * dt;
            predicted_.velocity += acceleration * dt;
            predicted_.rotation *= exp<double>(omega * dt);
            predicted_.omg = omega; predicted_.acceleration = specific;
            time_ = right.timestamp;
        }
    }
    std::deque<common::ImuMsg> history_;
    state predicted_;
    bool ready_ = false;
    double time_ = 0, posterior_time_ = 0;
};
}
