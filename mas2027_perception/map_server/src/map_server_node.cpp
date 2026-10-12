#include <map_server/map_server_node.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

namespace map_server {

namespace {

struct OccupancyMapMetadata {
    double resolution;
    double origin_x;
    double origin_y;
    int width;
    int height;
};

OccupancyMapMetadata load_occupancy_map_metadata(const std::string& yaml_path) {
    if (yaml_path.empty()) {
        throw std::invalid_argument("map_yaml_path must point to a Nav2 occupancy-map YAML");
    }

    const YAML::Node document = YAML::LoadFile(yaml_path);
    const YAML::Node origin = document["origin"];
    if (!origin || !origin.IsSequence() || origin.size() != 3) {
        throw std::runtime_error(yaml_path + ": origin must be [x, y, yaw]");
    }

    OccupancyMapMetadata metadata{};
    metadata.resolution = document["resolution"].as<double>();
    metadata.origin_x = origin[0].as<double>();
    metadata.origin_y = origin[1].as<double>();
    const double origin_yaw = origin[2].as<double>();
    if (!std::isfinite(metadata.resolution) || metadata.resolution <= 0.0) {
        throw std::runtime_error(yaml_path + ": resolution must be finite and positive");
    }
    if (!std::isfinite(metadata.origin_x) || !std::isfinite(metadata.origin_y)
        || !std::isfinite(origin_yaw)) {
        throw std::runtime_error(yaml_path + ": origin values must be finite");
    }
    if (std::abs(origin_yaw) > 1e-9) {
        throw std::runtime_error(
            yaml_path + ": non-zero origin yaw is unsupported; rotate all exported map products first");
    }

    const YAML::Node image_node = document["image"];
    if (!image_node || !image_node.IsScalar()) {
        throw std::runtime_error(yaml_path + ": image must name the matching PGM file");
    }
    std::filesystem::path image_path(image_node.as<std::string>());
    if (image_path.is_relative()) {
        image_path = std::filesystem::path(yaml_path).parent_path() / image_path;
    }
    const cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        throw std::runtime_error("failed to load occupancy image: " + image_path.string());
    }
    metadata.width = image.cols;
    metadata.height = image.rows;
    return metadata;
}

void validate_matching_maps(
    const OccupancyMapMetadata& metadata,
    const map_utils::NavigationMapData& terrain,
    const std::string& yaml_path,
    const std::string& terrain_path) {
    constexpr double resolution_tolerance = 1e-9;
    if (metadata.width != terrain.width || metadata.height != terrain.height) {
        std::ostringstream message;
        message << "map size mismatch: " << yaml_path << " references "
                << metadata.width << 'x' << metadata.height << ", but " << terrain_path
                << " is " << terrain.width << 'x' << terrain.height;
        throw std::runtime_error(message.str());
    }
    if (std::abs(metadata.resolution - terrain.resolution) > resolution_tolerance) {
        std::ostringstream message;
        message << "map resolution mismatch: " << yaml_path << " uses "
                << metadata.resolution << " m/px, but " << terrain_path << " uses "
                << terrain.resolution << " m/px";
        throw std::runtime_error(message.str());
    }
}

}  // namespace

MapServerNode::MapServerNode(const rclcpp::NodeOptions& options) : Node("map_server", options) {
    const auto terrain_path = declare_parameter<std::string>("terrain_map_path", "");
    const auto yaml_path = declare_parameter<std::string>("map_yaml_path", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    map_utils::MapInflationParams inflation;
    inflation.full_cost_radius_m = declare_parameter<double>("full_cost_radius_m", 0.0);
    inflation.cutoff_radius_m = declare_parameter<double>("cutoff_radius_m", 0.25);
    inflation.decay_rate_per_m = declare_parameter<double>("decay_rate_per_m", 8.0);
    if (terrain_path.empty()) {
        throw std::invalid_argument("terrain_map_path must point to a converted msgpack map");
    }
    maps_ = map_utils::load_navigation_maps(terrain_path, inflation);
    const auto metadata = load_occupancy_map_metadata(yaml_path);
    validate_matching_maps(metadata, maps_, yaml_path, terrain_path);

    cost_grid_ = to_occupancy_grid(maps_.cost_map, maps_.width, maps_.height,
        maps_.resolution, metadata.origin_x, metadata.origin_y, frame_id_);
    terrain_label_image_ = to_image(maps_.terrain_label_map, "mono8", frame_id_, now());
    uphill_image_ = to_image(maps_.uphill_map, "32FC2", frame_id_, now());
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    cost_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("cost_map", map_qos);
    terrain_label_pub_ = create_publisher<sensor_msgs::msg::Image>("terrain_label_map", map_qos);
    uphill_pub_ = create_publisher<sensor_msgs::msg::Image>("terrain_uphill_map", map_qos);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { publish_maps(); });
    RCLCPP_INFO(get_logger(),
        "loaded %dx%d static terrain map at %.3f m/px, origin=(%.6f, %.6f) from %s",
        maps_.width, maps_.height, maps_.resolution,
        metadata.origin_x, metadata.origin_y, yaml_path.c_str());
}

void MapServerNode::publish_maps() {
    const auto stamp = now();
    cost_grid_.header.stamp = stamp;
    terrain_label_image_.header.stamp = stamp;
    uphill_image_.header.stamp = stamp;
    cost_pub_->publish(cost_grid_);
    terrain_label_pub_->publish(terrain_label_image_);
    uphill_pub_->publish(uphill_image_);
}

nav_msgs::msg::OccupancyGrid MapServerNode::to_occupancy_grid(
    const cv::Mat& map, const int width, const int height, const double resolution,
    const double origin_x, const double origin_y, const std::string& frame_id) {
    nav_msgs::msg::OccupancyGrid result;
    result.header.frame_id = frame_id;
    result.info.width = static_cast<uint32_t>(width);
    result.info.height = static_cast<uint32_t>(height);
    result.info.resolution = static_cast<float>(resolution);
    result.info.origin.position.x = origin_x;
    result.info.origin.position.y = origin_y;
    result.info.origin.orientation.w = 1.0;
    result.data.resize(static_cast<size_t>(width) * static_cast<size_t>(height));
    for (size_t i = 0; i < result.data.size(); ++i) {
        result.data[i] = static_cast<int8_t>(
            std::min(100, (static_cast<int>(map.data[i]) * 100) / 255));
    }
    return result;
}

sensor_msgs::msg::Image MapServerNode::to_image(
    const cv::Mat& map, const std::string& encoding,
    const std::string& frame_id, const rclcpp::Time& stamp) {
    sensor_msgs::msg::Image result;
    result.header.stamp = stamp;
    result.header.frame_id = frame_id;
    result.height = static_cast<uint32_t>(map.rows);
    result.width = static_cast<uint32_t>(map.cols);
    result.encoding = encoding;
    result.is_bigendian = false;
    result.step = static_cast<uint32_t>(map.step);
    result.data.assign(map.datastart, map.dataend);
    return result;
}

}  // namespace map_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_server::MapServerNode)
