#pragma once

#include <cmath>
#include <stdexcept>

namespace rog_map {

// Algorithm configuration shared by ROS loading, projection and profile tests.
// Runtime YAML: mas2027_nav_executor/config/terrain_recognition.yaml.
struct SlopeRecognitionConfig {
  bool enable{true};
  int fit_radius_cells{5};
  int min_samples{30};
  double min_gradient{0.14};
  double max_gradient{0.70};
  double min_height_change_m{0.06};
  double max_plane_rms_m{0.04};
  double max_plane_residual_m{0.05};
};

struct UndulatingRecognitionConfig {
  bool enable{true};
  bool entry_window_enable{true};
  double min_peak_to_trough_m{0.035};
  double max_peak_to_trough_m{0.12};
  double min_wavelength_m{0.18};
  double max_wavelength_m{0.34};
  double min_delta_m{0.01};
  double max_trend_gradient{0.25};
  double min_neighbor_correlation{0.80};
  int min_common_samples{9};
};

struct TunnelRecognitionConfig {
  bool enable{true};
  double min_free_gap_ratio{0.5};
  int min_neighbor_support{5};
};

struct TerrainRecognitionConfig {
  SlopeRecognitionConfig slope;
  TunnelRecognitionConfig tunnel;
  UndulatingRecognitionConfig undulating;

  void validate() const {
    const auto positive = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
    if (!std::isfinite(tunnel.min_free_gap_ratio) ||
        tunnel.min_free_gap_ratio < 0.0 || tunnel.min_free_gap_ratio > 1.0 ||
        tunnel.min_neighbor_support < 1 || tunnel.min_neighbor_support > 9) {
      throw std::invalid_argument("projection.terrain.tunnel: free gap ratio "
                                  "must be in [0,1], support in [1,9]");
    }
    const auto &s = slope;
    if (s.fit_radius_cells < 2 || s.fit_radius_cells > 20 ||
        s.min_samples < 3 ||
        s.min_samples >
            (2 * s.fit_radius_cells + 1) * (2 * s.fit_radius_cells + 1) ||
        !positive(s.min_gradient) || !positive(s.max_gradient) ||
        s.min_gradient > s.max_gradient || !positive(s.min_height_change_m) ||
        !positive(s.max_plane_rms_m) || !positive(s.max_plane_residual_m)) {
      throw std::invalid_argument("projection.terrain.slope: invalid radius, "
                                  "sample count or metric thresholds");
    }
    const auto &u = undulating;
    if (!positive(u.min_peak_to_trough_m) ||
        !positive(u.max_peak_to_trough_m) ||
        u.min_peak_to_trough_m > u.max_peak_to_trough_m ||
        !positive(u.min_wavelength_m) || !positive(u.max_wavelength_m) ||
        u.min_wavelength_m > u.max_wavelength_m || !positive(u.min_delta_m) ||
        u.min_delta_m >= u.min_peak_to_trough_m ||
        !positive(u.max_trend_gradient) ||
        !positive(u.min_neighbor_correlation) ||
        u.min_neighbor_correlation > 1.0 || u.min_common_samples < 3 ||
        u.min_common_samples > 17) {
      throw std::invalid_argument(
          "projection.terrain.undulating: invalid amplitude, wavelength, noise "
          "or support thresholds");
    }
  }
};

} // namespace rog_map
