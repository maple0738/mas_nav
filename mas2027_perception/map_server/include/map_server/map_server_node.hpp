#pragma once

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <map_server/utils.hpp>

namespace map_server {

// Static terrain publisher. Online obstacles are intentionally owned by ROGMap in nav_executor.
class MapServerNode final : public rclcpp::Node {
public:
    explicit MapServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void publish_maps();
    static nav_msgs::msg::OccupancyGrid to_occupancy_grid(
        const cv::Mat& map, int width, int height, double resolution,
        double origin_x, double origin_y, const std::string& frame_id);
    static sensor_msgs::msg::Image to_image(
        const cv::Mat& map, const std::string& encoding,
        const std::string& frame_id, const rclcpp::Time& stamp);

    map_utils::NavigationMapData maps_;
    std::string frame_id_;
    nav_msgs::msg::OccupancyGrid cost_grid_;
    sensor_msgs::msg::Image terrain_label_image_;
    sensor_msgs::msg::Image uphill_image_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr terrain_label_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr uphill_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace map_server
