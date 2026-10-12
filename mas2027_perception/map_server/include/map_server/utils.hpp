#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace map_server::map_utils {

enum class TerrainType : uint8_t {
    FLAT = 0,
    OBSTACLE = 1,
    SLOPE = 5,
    TUNNEL = 6,
    UNDULATING = 7
};

struct TerrainMapData {
    int width;
    int height;
    double resolution;
    std::vector<uint8_t> terrain;
    // Optional [uphill_x, uphill_y] for each cell, row major.
    std::vector<float> uphill;
};

struct MapInflationParams {
    double full_cost_radius_m; // 满代价半径内障碍代价保持255
    double cutoff_radius_m;
    double decay_rate_per_m;
    double resolution = 0.0; // map resolution (m/px), must be set before calling inflation functions
};

int containing_cell(double coordinate_m, double resolution);
int nearest_cell(double coordinate_m, double resolution);
double cell_center_coordinate(double cell_coordinate, double resolution);
int enclosing_radius_cells(double radius_m, double resolution);
int nearest_radius_cells(double radius_m, double resolution);
int morphology_kernel_size(double radius_m, double resolution);
int minimum_area_cells(double area_m2, double resolution);
int centered_extent_cells(double extent_m, double resolution);
int minimum_density_count(double density_per_m2, double resolution);

struct NavigationMapData {
    int width;
    int height;
    double resolution;
    cv::Mat cost_map;
    cv::Mat terrain_label_map;
    cv::Mat uphill_map;
};

TerrainMapData load_terrain_msgpack(const std::string& path);

NavigationMapData load_navigation_maps(
    const std::string& path,
    MapInflationParams inflation_params
);

cv::Mat inflate_cost_map(
    const cv::Mat& source,
    const MapInflationParams& params
);

/// Inflate only connected-component neighborhoods whose support can reach a
/// non-zero output. Falls back to the full-map transform when the aggregate
/// neighborhood area is not smaller than the source map.
cv::Mat inflate_cost_map_bounded(
    const cv::Mat& source,
    const MapInflationParams& params
);

} // namespace map_server::map_utils
