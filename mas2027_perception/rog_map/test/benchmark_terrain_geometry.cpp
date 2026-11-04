#include <rog_map/projection_layer.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Standalone deterministic microbenchmark; not a wall-clock assertion in CTest.
int main()
{
  constexpr int size = 201;
  constexpr double resolution = .05;
  std::cout << "scene,median_ms,p95_ms,geometry_hash\n";
  for (int scene = 0; scene < 7; ++scene) {
    rog_map::ProjectionLayer layer;
    rog_map::ProjectionLayerConfig config;
    config.hysteresis_en = false;
    config.mask_filter_en = false;
    layer.updateFull(size, size, resolution, Eigen::Vector2d::Zero(), 1., config,
      [scene](int x, int y) {
        rog_map::ColumnStats stats;
        if (scene == 4 && (x * 17 + y * 13) % 7 < 3) return stats;
        double z = .003 * std::sin(x * 1.7 + y * 3.1);
        if (scene == 1) z += .25 * x * resolution;
        if (scene == 2) z += .035 * std::sin(2 * M_PI * x * resolution / .24);
        if (scene == 3 && x > size / 2) z += .2;
        if (scene == 5) {
          if (x > 60 && x < 100) z += .25 * (x - 60) * resolution;
          if (x >= 100) z += .5;
          if (y > 120) z += .035 * std::sin(2 * M_PI * x * resolution / .24);
        }
        stats.observed_count = 10;
        stats.occupied_count = 1;
        stats.occupied_z_index_min = stats.occupied_z_index_max = std::lround(z / resolution);
        stats.occupied_z_min_abs = stats.occupied_z_max_abs = z;
        if (scene == 6) {
          stats.occupied_count = 2;
          stats.occupied_z_index_max += 7;
          stats.occupied_z_max_abs += .35;
          stats.free_between_occupied_count = 6;
        }
        stats.surface_valid = true;
        stats.surface_z = stats.surface_low = stats.surface_high = z;
        return stats;
      });
    std::vector<double> times;
    uint64_t expected_hash = 0;
    for (int iteration = 0; iteration < 23; ++iteration) {
      const auto start = std::chrono::steady_clock::now();
      const auto geometry = layer.terrainGeometry();
      const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      uint64_t hash = 14695981039346656037ULL;
      const auto bytes = [&](const auto & values) {
        const auto * data = reinterpret_cast<const unsigned char *>(values.data());
        for (size_t i = 0; i < values.size() * sizeof(values[0]); ++i)
          hash = (hash ^ data[i]) * 1099511628211ULL;
      };
      bytes(geometry.labels); bytes(geometry.uphill_x); bytes(geometry.uphill_y);
      if (iteration == 0) expected_hash = hash;
      if (hash != expected_hash) return 1;
      if (iteration >= 3) times.push_back(ms);
    }
    std::sort(times.begin(), times.end());
    const char * names[] = {"flat", "slope", "waves", "step", "sparse", "mixed", "tunnel"};
    std::cout << names[scene] << ',' << std::fixed << std::setprecision(3)
              << (times[9] + times[10]) / 2 << ',' << times[18] << ',' << expected_hash << '\n';
  }
}
