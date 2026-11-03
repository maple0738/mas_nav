#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace small_point_lio {

struct AdaptiveSamplingConfig {
    double min_leaf = 0.25;
    double max_leaf = 1.0;
    double min_points_per_second = 16000.0;
    double max_points_per_second = 40000.0;
    double reference_range = 15.0;

    void validate() const {
        if (!std::isfinite(min_leaf) || !std::isfinite(max_leaf) ||
            !std::isfinite(min_points_per_second) || !std::isfinite(max_points_per_second) ||
            !std::isfinite(reference_range) || min_leaf <= 0 || max_leaf < min_leaf ||
            min_points_per_second <= 0 || max_points_per_second < min_points_per_second ||
            reference_range <= 0) {
            throw std::invalid_argument("invalid adaptive sampling bounds, point rates or reference range");
        }
    }
};

// One sampling pass per input. Feedback changes the NEXT input's leaf size.
class AdaptiveSampling {
public:
    void reset() { leaf_ = 0; scene_range_ = 0; target_ = 0; }

    double leaf(double initial, const AdaptiveSamplingConfig &config) {
        if (leaf_ == 0) {
            leaf_ = std::clamp(initial, config.min_leaf, config.max_leaf);
        }
        return leaf_;
    }

    double target() const { return target_; }

    // Ranges come from voxel representatives, avoiding raw-return density bias.
    void observe(std::vector<double> &ranges, double duration,
                 const AdaptiveSamplingConfig &config) {
        // Empty, degenerate or interrupted inputs must not train the controller.
        if (ranges.empty() || !std::isfinite(duration) || duration < 0.001 || duration > 0.2) {
            return;
        }
        auto middle = ranges.begin() + ranges.size() / 2;
        std::nth_element(ranges.begin(), middle, ranges.end());
        const double median = *middle;
        if (!std::isfinite(median) || median <= 0) {
            return;
        }
        const double alpha = 1.0 - std::exp(-duration / 0.5);
        scene_range_ = scene_range_ == 0 ? median : scene_range_ + alpha * (median - scene_range_);
        const double scale = std::clamp(scene_range_ / config.reference_range, 0.0, 1.0);
        target_ = duration * (config.min_points_per_second +
            (config.max_points_per_second - config.min_points_per_second) * scale * scale);
        const double ratio = static_cast<double>(ranges.size()) / target_;
        if (ratio >= 0.9 && ratio <= 1.1) {
            return;
        }
        // Bounded proportional feedback; normalize its response to elapsed sensor time.
        const double change = std::clamp(0.15 * std::log(ratio), -0.1, 0.1) * (duration / 0.05);
        leaf_ = std::clamp(leaf_ * std::exp(change), config.min_leaf, config.max_leaf);
    }

private:
    double leaf_ = 0;
    double scene_range_ = 0;
    double target_ = 0;
};

}  // namespace small_point_lio
