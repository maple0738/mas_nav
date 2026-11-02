#pragma once

#include <vector>

#include "mas2027_nav_executor/path_planner/search/directed_terrain.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/runup_gate.hpp"

namespace mas2027_nav_executor {

struct RunupConfig {
  bool enable{false};
  double distance{0.2};
  double transition{0.1};
  double saturation_length{0.05};
  double alignment_weight{100.0};
  double curvature_weight{100.0};
  double speed_regularization{0.1};
  // A soft cone inside the exact field used by the final direction gate.
  // Zero preserves the original exposure-only regularizer for other callers.
  double minimum_alignment_cosine{0.0};
};
struct RunupSample {
  Eigen::Vector2d position, velocity, acceleration;
  double time_measure{};
};
struct RunupGradient {
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
  Eigen::Vector2d acceleration{Eigen::Vector2d::Zero()};
  double time_measure{};
};

// Forward arc exposure, including its nonlocal adjoint. Unlike a position
// penalty this discourages turning along the approach AND through the ramp.
// Both directions share the same axis cost; no speed cap or clearance changes.
inline double runupCost(const std::vector<RunupSample>& samples, const TerrainSampler& sampler,
                        const RunupConfig& cfg, std::vector<RunupGradient>& gradient) {
  const size_t n = samples.size();
  gradient.assign(n, {});
  if (!cfg.enable || !sampler || cfg.saturation_length <= 0) return 0.0;
  struct State {
    rog_map::TerrainDirectionSample terrain;
    Eigen::Vector2d tangent, g_tangent{Eigen::Vector2d::Zero()}, g_axis{Eigen::Vector2d::Zero()};
    double speed{}, arc{}, curvature{}, g_arc{}, g_body{}, g_curvature{};
  };
  std::vector<State> state(n);
  for (size_t i = 0; i < n; ++i) {
    const auto& s = samples[i];
    auto& v = state[i];
    v.terrain = sampler(s.position.x(), s.position.y());
    v.speed =
        std::sqrt(s.velocity.squaredNorm() + cfg.speed_regularization * cfg.speed_regularization);
    v.arc = s.time_measure * v.speed;
    v.tangent = s.velocity / v.speed;
    v.curvature = (s.velocity.x() * s.acceleration.y() - s.velocity.y() * s.acceleration.x()) /
                  (v.speed * v.speed * v.speed);
  }
  const auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() * b.y() - a.y() * b.x();
  };
  const auto add_cross = [](State& current, State& source, double scale) {
    current.g_tangent +=
        scale * Eigen::Vector2d(source.terrain.uphill.y(), -source.terrain.uphill.x());
    source.g_axis += scale * Eigen::Vector2d(-current.tangent.y(), current.tangent.x());
  };
  // Range differences accumulate the derivative of the arc between i and j.
  std::vector<double> range(n + 1, 0.0);
  double cost = 0.0;
  for (size_t i = 0; i < n; ++i) {
    auto& current = state[i];
    double distance = 0.0, exposure = 0.0, crossed_exposure = 0.0;
    size_t end = i;
    for (size_t j = i; j < n; ++j) {
      const auto& source = state[j];
      if (distance >= cfg.distance + cfg.transition && j != i) break;
      const double gate = runup_distance_gate(distance, cfg.distance, cfg.transition).first;
      const double c = cross(current.tangent, source.terrain.uphill);
      const double w = source.terrain.body * gate * source.arc;
      exposure += w;
      crossed_exposure += w * c * c;
      distance += source.arc;
      end = j + 1;
    }
    const double saturation = cfg.saturation_length;
    const double exponential = std::exp(-exposure / saturation);
    const double gate = -std::expm1(-exposure / saturation);
    // Stable G(E)/E at the removable singularity; its derivative is essential
    // to propagate future terrain changes back to the current approach.
    const double h = exposure > 1e-6 * saturation
                         ? gate / exposure
                         : 1 / saturation - exposure / (2 * saturation * saturation);
    const double dh = exposure > 1e-6 * saturation
                          ? (exponential * exposure / saturation - gate) / (exposure * exposure)
                          : -1 / (2 * saturation * saturation);
    const double body = current.terrain.body;
    const double c = cross(current.tangent, current.terrain.uphill);
    const double curve = 0.5 * cfg.curvature_weight * current.curvature * current.curvature;
    const double align = 0.5 * cfg.alignment_weight;
    const double cosine2 = cfg.minimum_alignment_cosine * cfg.minimum_alignment_cosine;
    const double projection = current.tangent.dot(current.terrain.uphill);
    const double violation = cfg.minimum_alignment_cosine > 0 ?
      std::max(0.0, cosine2 * current.tangent.squaredNorm() - projection * projection) : 0.0;
    const double cone_weight = 100.0 * cfg.alignment_weight;
    const double cone = 0.5 * cone_weight * violation * violation;
    const double density = curve * (body + (1 - body) * gate) +
                           align * (body * c * c + (1 - body) * h * crossed_exposure) + body * cone;
    cost += current.arc * density;
    current.g_arc += density;
    current.g_curvature +=
        current.arc * cfg.curvature_weight * current.curvature * (body + (1 - body) * gate);
    current.g_body += current.arc * (curve * (1 - gate) + align * (c * c - h * crossed_exposure) + cone);
    const double cone_scale = current.arc * body * cone_weight * violation;
    current.g_tangent += 2 * cone_scale *
      (cosine2 * current.tangent - projection * current.terrain.uphill);
    current.g_axis -= 2 * cone_scale * projection * current.tangent;
    add_cross(current, current, current.arc * cfg.alignment_weight * body * c);
    const double g_exposure = current.arc * (curve * (1 - body) * exponential / saturation +
                                             align * (1 - body) * dh * crossed_exposure);
    const double g_crossed = current.arc * align * (1 - body) * h;
    distance = 0.0;
    for (size_t j = i; j < end; ++j) {
      auto& source = state[j];
      const auto gate_pair = runup_distance_gate(distance, cfg.distance, cfg.transition);
      const double c_future = cross(current.tangent, source.terrain.uphill);
      const double g_weight = g_exposure + g_crossed * c_future * c_future;
      source.g_body += g_weight * gate_pair.first * source.arc;
      source.g_arc += g_weight * source.terrain.body * gate_pair.first;
      const double g_distance = g_weight * source.terrain.body * source.arc * gate_pair.second;
      range[i] += g_distance;
      range[j] -= g_distance;
      add_cross(current, source,
                2 * g_crossed * c_future * source.terrain.body * gate_pair.first * source.arc);
      distance += source.arc;
    }
  }
  double g_distance = 0.0;
  for (size_t i = 0; i < n; ++i) {
    auto& v = state[i];
    const auto& s = samples[i];
    auto& g = gradient[i];
    g_distance += range[i];
    v.g_arc += g_distance;
    g.position =
        v.g_body * v.terrain.body_gradient + v.terrain.uphill_jacobian.transpose() * v.g_axis;
    const double q = v.speed * v.speed;
    const double q32 = q * v.speed;
    g.velocity =
        (Eigen::Matrix2d::Identity() - v.tangent * v.tangent.transpose()) * v.g_tangent / v.speed;
    g.velocity += v.g_curvature * (Eigen::Vector2d(s.acceleration.y(), -s.acceleration.x()) / q32 -
                                   3 * v.curvature * s.velocity / q);
    g.acceleration = v.g_curvature * Eigen::Vector2d(-s.velocity.y(), s.velocity.x()) / q32;
    g.velocity += v.g_arc * s.time_measure * s.velocity / v.speed;
    g.time_measure = v.g_arc * v.speed;
  }
  return cost;
}

}  // namespace mas2027_nav_executor
