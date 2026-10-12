#include <map_server/utils.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

#include <msgpack.hpp>

namespace map_server::map_utils {

namespace {

constexpr uint8_t TERRAIN_LABEL_COUNT = static_cast<uint8_t>(TerrainType::UNDULATING) + 1;
// 每个膨胀区域的固定调用开销（阈值化 / 距离变换的启动成本）折算为等价的整图格数；
// 经实测标定，使分区/整图路径的回退阈值与两者实际耗时的交叉点吻合。
constexpr size_t PER_REGION_INFLATION_OVERHEAD_CELLS = 48;

void validate_resolution(const double resolution) {
    if (!std::isfinite(resolution) || resolution <= 0.0) {
        throw std::invalid_argument("map resolution must be finite and positive");
    }
}

int checked_integer(const double value, const char* quantity) {
    if (!std::isfinite(value)
        || value < static_cast<double>(std::numeric_limits<int>::min())
        || value > static_cast<double>(std::numeric_limits<int>::max())) {
        throw std::overflow_error(std::string(quantity) + " exceeds the supported cell range");
    }
    return static_cast<int>(value);
}

int ceil_nonnegative(const double value, const char* quantity) {
    if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string(quantity) + " must be finite and non-negative");
    }
    // Avoid promoting values that are only infinitesimally above an integer due to FP division.
    const double tolerance = 1e-9 * std::max(1.0, value);
    return checked_integer(std::ceil(value - tolerance), quantity);
}

void validate_common_inflation_params(const MapInflationParams& params) {
    if (!std::isfinite(params.resolution) || params.resolution <= 0.0) {
        throw std::invalid_argument("map inflation resolution must be finite and positive");
    }
    if (!std::isfinite(params.full_cost_radius_m) || params.full_cost_radius_m < 0.0
        || !std::isfinite(params.cutoff_radius_m)
        || params.cutoff_radius_m < params.full_cost_radius_m) {
        throw std::invalid_argument(
            "map inflation radii require 0 <= full_cost_radius_m <= cutoff_radius_m"
        );
    }
    if (!std::isfinite(params.decay_rate_per_m) || params.decay_rate_per_m < 0.0) {
        throw std::invalid_argument(
            "map inflation decay_rate_per_m must be finite and non-negative"
        );
    }
}

} // anonymous namespace

int containing_cell(const double coordinate_m, const double resolution) {
    validate_resolution(resolution);
    if (!std::isfinite(coordinate_m)) {
        throw std::invalid_argument("map coordinate must be finite");
    }
    return checked_integer(std::floor(coordinate_m / resolution), "map coordinate");
}

int nearest_cell(const double coordinate_m, const double resolution) {
    validate_resolution(resolution);
    if (!std::isfinite(coordinate_m)) {
        throw std::invalid_argument("map coordinate must be finite");
    }
    return checked_integer(
        std::round(coordinate_m / resolution - 0.5),
        "map coordinate"
    );
}

double cell_center_coordinate(const double cell_coordinate, const double resolution) {
    validate_resolution(resolution);
    if (!std::isfinite(cell_coordinate)) {
        throw std::invalid_argument("cell coordinate must be finite");
    }
    return (cell_coordinate + 0.5) * resolution;
}

int enclosing_radius_cells(const double radius_m, const double resolution) {
    validate_resolution(resolution);
    return ceil_nonnegative(radius_m / resolution, "metric radius");
}

int nearest_radius_cells(const double radius_m, const double resolution) {
    validate_resolution(resolution);
    if (!std::isfinite(radius_m) || radius_m < 0.0) {
        throw std::invalid_argument("metric radius must be finite and non-negative");
    }
    return checked_integer(std::round(radius_m / resolution), "metric radius");
}

int morphology_kernel_size(const double radius_m, const double resolution) {
    const int radius_cells = nearest_radius_cells(radius_m, resolution);
    if (radius_cells == 0) return 0;
    if (radius_cells > (std::numeric_limits<int>::max() - 1) / 2) {
        throw std::overflow_error("morphology radius exceeds the supported cell range");
    }
    return 2 * radius_cells + 1;
}

int minimum_area_cells(const double area_m2, const double resolution) {
    validate_resolution(resolution);
    return ceil_nonnegative(area_m2 / (resolution * resolution), "metric area");
}

int centered_extent_cells(const double extent_m, const double resolution) {
    validate_resolution(resolution);
    if (!std::isfinite(extent_m) || extent_m <= 0.0) {
        throw std::invalid_argument("metric extent must be finite and positive");
    }
    int extent_cells = ceil_nonnegative(extent_m / resolution, "metric extent");
    if (extent_cells % 2 == 0) {
        if (extent_cells == std::numeric_limits<int>::max()) {
            throw std::overflow_error("metric extent exceeds the supported cell range");
        }
        ++extent_cells;
    }
    if (extent_cells <= 0) {
        throw std::overflow_error("metric extent exceeds the supported cell range");
    }
    return extent_cells;
}

int minimum_density_count(const double density_per_m2, const double resolution) {
    validate_resolution(resolution);
    const int count = ceil_nonnegative(
        density_per_m2 * resolution * resolution,
        "projected point density"
    );
    return std::max(1, count);
}

TerrainMapData load_terrain_msgpack(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {
        throw std::runtime_error("Failed to open " + path);
    }
    std::vector<char> buffer((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    if (buffer.empty()) {
        throw std::runtime_error("Empty msgpack file: " + path);
    }

    auto handle = msgpack::unpack(buffer.data(), buffer.size());
    auto obj = handle.get();
    if (obj.type != msgpack::type::MAP) {
        throw std::runtime_error("Msgpack root must be a MAP");
    }

    TerrainMapData data{};
    auto m = obj.via.map;
    for (uint32_t i = 0; i < m.size; i++) {
        auto key = m.ptr[i].key.as<std::string>();
        if (key == "width") {
            data.width = m.ptr[i].val.as<int>();
        } else if (key == "height") {
            data.height = m.ptr[i].val.as<int>();
        } else if (key == "resolution") {
            data.resolution = m.ptr[i].val.as<double>();
        } else if (key == "terrain") {
            data.terrain.clear(); for (uint32_t j = 0; j < m.ptr[i].val.via.array.size; ++j) data.terrain.push_back(m.ptr[i].val.via.array.ptr[j].as<uint8_t>());
        } else if (key == "uphill") {
            const auto& values = m.ptr[i].val;
            if (values.type != msgpack::type::ARRAY) {
                throw std::runtime_error("uphill must be a flat array of XY pairs");
            }
            data.uphill.clear();
            data.uphill.reserve(values.via.array.size);
            for (uint32_t j = 0; j < values.via.array.size; ++j) {
                data.uphill.push_back(values.via.array.ptr[j].as<float>());
            }
        }
    }

    if (data.width <= 0 || data.height <= 0) {
        throw std::runtime_error("terrain map dimensions must be positive");
    }
    if (!std::isfinite(data.resolution) || data.resolution <= 0.0) {
        throw std::runtime_error("terrain map resolution must be finite and positive");
    }
    const size_t expected = static_cast<size_t>(data.width) * static_cast<size_t>(data.height);
    if (data.terrain.size() != expected) {
        throw std::runtime_error(
            "terrain data size mismatch: expected " + std::to_string(expected) +
            ", got " + std::to_string(data.terrain.size())
        );
    }
    for (size_t i = 0; i < data.terrain.size(); ++i) {
        if (data.terrain[i] >= TERRAIN_LABEL_COUNT ||
            (data.terrain[i] > static_cast<uint8_t>(TerrainType::OBSTACLE) &&
             data.terrain[i] < static_cast<uint8_t>(TerrainType::SLOPE))) {
            throw std::runtime_error(
                "invalid terrain label " + std::to_string(data.terrain[i])
                + " at flat index " + std::to_string(i)
            );
        }
    }
    if (!data.uphill.empty() && data.uphill.size() != 2 * expected) {
        throw std::runtime_error("uphill data size must equal twice the terrain cell count");
    }
    for (size_t i = 0; i < expected && !data.uphill.empty(); ++i) {
        const float x = data.uphill[2 * i], y = data.uphill[2 * i + 1];
        const float length = std::hypot(x, y);
        if (!std::isfinite(length) ||
            (length > 0.01f && std::abs(length - 1.0f) > 0.05f) ||
            (data.terrain[i] != static_cast<uint8_t>(TerrainType::SLOPE) && length > 0.01f)) {
            throw std::runtime_error("invalid uphill vector at flat index " + std::to_string(i));
        }
    }

    return data;
}

NavigationMapData load_navigation_maps(
    const std::string& path,
    MapInflationParams inflation_params
) {
    const TerrainMapData terrain_data = load_terrain_msgpack(path);
    inflation_params.resolution = terrain_data.resolution;

    cv::Mat obstacle_mask = cv::Mat::zeros(terrain_data.height, terrain_data.width, CV_8UC1);
    for (int y = 0; y < terrain_data.height; ++y) {
        const size_t row_offset = static_cast<size_t>(y) * static_cast<size_t>(terrain_data.width);
        uint8_t* row = obstacle_mask.ptr<uint8_t>(y);
        for (int x = 0; x < terrain_data.width; ++x) {
            if (terrain_data.terrain[row_offset + static_cast<size_t>(x)]
                == static_cast<uint8_t>(TerrainType::OBSTACLE)) {
                row[x] = 255;
            }
        }
    }
    cv::Mat cost_map = inflate_cost_map(obstacle_mask, inflation_params);

    cv::Mat terrain_label_map(terrain_data.height, terrain_data.width, CV_8UC1);
    std::copy(terrain_data.terrain.begin(), terrain_data.terrain.end(), terrain_label_map.ptr<uint8_t>(0));
    cv::Mat uphill_map = cv::Mat::zeros(terrain_data.height, terrain_data.width, CV_32FC2);
    if (!terrain_data.uphill.empty()) {
        std::copy(terrain_data.uphill.begin(), terrain_data.uphill.end(),
            uphill_map.ptr<float>(0));
    } else {
        // A label-only prior map cannot tell which end is higher. Its long
        // connected-component axis can still prevent entry from the sides.
        cv::Mat slope_mask = terrain_label_map == static_cast<uint8_t>(TerrainType::SLOPE);
        cv::Mat components;
        const int component_count = cv::connectedComponents(slope_mask, components, 8, CV_32S);
        struct Moments {
            double x{0.0}, y{0.0}, xx{0.0}, xy{0.0}, yy{0.0};
            int count{0};
        };
        std::vector<Moments> moments(static_cast<size_t>(component_count));
        for (int y = 0; y < terrain_data.height; ++y) {
            for (int x = 0; x < terrain_data.width; ++x) {
                const int component = components.at<int>(y, x);
                if (component == 0) continue;
                auto& m = moments[component];
                m.x += x; m.y += y; m.xx += static_cast<double>(x) * x;
                m.xy += static_cast<double>(x) * y; m.yy += static_cast<double>(y) * y;
                ++m.count;
            }
        }
        std::vector<cv::Vec2f> axes(moments.size(), cv::Vec2f(0.0f, 0.0f));
        for (size_t i = 1; i < moments.size(); ++i) {
            const auto& m = moments[i];
            if (m.count < 16) continue;
            const double mx = m.x / m.count, my = m.y / m.count;
            const double xx = m.xx / m.count - mx * mx;
            const double xy = m.xy / m.count - mx * my;
            const double yy = m.yy / m.count - my * my;
            const double discriminant = std::hypot(xx - yy, 2.0 * xy);
            const double major = (xx + yy + discriminant) * 0.5;
            const double minor = (xx + yy - discriminant) * 0.5;
            if (major < 1.5 * std::max(1.0, minor)) continue;
            const double angle = 0.5 * std::atan2(2.0 * xy, xx - yy);
            axes[i] = cv::Vec2f(static_cast<float>(std::cos(angle)),
                                static_cast<float>(std::sin(angle)));
        }
        for (int y = 0; y < terrain_data.height; ++y) {
            for (int x = 0; x < terrain_data.width; ++x) {
                uphill_map.at<cv::Vec2f>(y, x) = axes[components.at<int>(y, x)];
            }
        }
    }

    return {
        .width = terrain_data.width,
        .height = terrain_data.height,
        .resolution = terrain_data.resolution,
        .cost_map = std::move(cost_map),
        .terrain_label_map = std::move(terrain_label_map),
        .uphill_map = std::move(uphill_map),
    };
}

cv::Mat inflate_cost_map(
    const cv::Mat& source,
    const MapInflationParams& params
) {
    CV_Assert(source.type() == CV_8UC1);
    validate_common_inflation_params(params);
    const int h = source.rows;
    const int w = source.cols;

    // 二值化: 非零值视为障碍物源
    cv::Mat bin_mask;
    cv::threshold(source, bin_mask, 0, 1, cv::THRESH_BINARY);

    // 距离变换（像素单位）
    cv::Mat dist_px;
    cv::distanceTransform(1 - bin_mask, dist_px, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    cv::Mat out = source.clone();
    for (int y = 0; y < h; y++) {
        const float* dist_row = dist_px.ptr<float>(y);
        uint8_t* out_row = out.ptr<uint8_t>(y);
        for (int x = 0; x < w; x++) {
            if (bin_mask.at<uint8_t>(y, x)) continue;

            const double distance_m = static_cast<double>(dist_row[x]) * params.resolution;
            if (distance_m <= params.full_cost_radius_m) {
                out_row[x] = 255;
            } else if (distance_m <= params.cutoff_radius_m) {
                const float v = 255.0f * static_cast<float>(std::exp(
                    -params.decay_rate_per_m * (distance_m - params.full_cost_radius_m)
                ));
                out_row[x] = static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
            }
        }
    }
    return out;
}

cv::Mat inflate_cost_map_bounded(
    const cv::Mat& source,
    const MapInflationParams& params
) {
    CV_Assert(source.type() == CV_8UC1);
    validate_common_inflation_params(params);

    const int cutoff_radius_px = std::min(
        enclosing_radius_cells(params.cutoff_radius_m, params.resolution),
        std::max(source.rows, source.cols)
    );
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int label_count = cv::connectedComponentsWithStats(
        source, labels, stats, centroids, 8, CV_32S
    );

    std::vector<cv::Rect> inflation_regions;
    inflation_regions.reserve(static_cast<size_t>(std::max(0, label_count - 1)));
    size_t estimated_cell_cost = 0;
    const size_t map_cell_count = static_cast<size_t>(source.rows)
        * static_cast<size_t>(source.cols);

    for (int label = 1; label < label_count; ++label) {
        const cv::Rect component_bounds(
            stats.at<int>(label, cv::CC_STAT_LEFT),
            stats.at<int>(label, cv::CC_STAT_TOP),
            stats.at<int>(label, cv::CC_STAT_WIDTH),
            stats.at<int>(label, cv::CC_STAT_HEIGHT)
        );
        const int left = std::max(0, component_bounds.x - cutoff_radius_px);
        const int top = std::max(0, component_bounds.y - cutoff_radius_px);
        const int right = static_cast<int>(std::min<int64_t>(
            source.cols,
            static_cast<int64_t>(component_bounds.x) + component_bounds.width
                + cutoff_radius_px
        ));
        const int bottom = static_cast<int>(std::min<int64_t>(
            source.rows,
            static_cast<int64_t>(component_bounds.y) + component_bounds.height
                + cutoff_radius_px
        ));
        const cv::Rect clipped_region(left, top, right - left, bottom - top);
        if (clipped_region.empty()) continue;

        // 面积之和会重复计入重叠区域，因此本身已是保守上界；再加上每个区域固有的
        // 调用开销（阈值化 / 距离变换的启动成本），否则连通域很多时估计会偏低。
        estimated_cell_cost += static_cast<size_t>(clipped_region.area())
            + PER_REGION_INFLATION_OVERHEAD_CELLS;
        if (estimated_cell_cost >= map_cell_count) {
            return inflate_cost_map(source, params);
        }
        inflation_regions.push_back(clipped_region);
    }

    cv::Mat result = source.clone();
    for (const cv::Rect& region : inflation_regions) {
        const cv::Mat inflated_region = inflate_cost_map(source(region), params);
        cv::Mat result_region = result(region);
        cv::max(result_region, inflated_region, result_region);
    }
    return result;
}

} // namespace map_server::map_utils
