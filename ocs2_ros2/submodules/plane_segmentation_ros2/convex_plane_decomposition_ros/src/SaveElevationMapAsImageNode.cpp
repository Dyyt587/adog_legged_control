//
// Created by rgrandia on 11.06.20.
//

#include "rclcpp/rclcpp.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <opencv2/imgcodecs.hpp>

int count = 0;
double frequency;
std::string elevationMapTopic;
std::string elevationLayer;
std::string imageName;

void callback(const grid_map_msgs::msg::GridMap::ConstSharedPtr& message) {
  grid_map::GridMap messageMap;
  grid_map::GridMapRosConverter::fromMessage(*message, messageMap);

  const auto& data = messageMap[elevationLayer];
  float maxHeight = std::numeric_limits<float>::lowest();
  float minHeight = std::numeric_limits<float>::max();
  for (int i = 0; i < data.rows(); i++) {
    for (int j = 0; j < data.cols(); j++) {
      const auto value = data(i, j);
      if (!std::isnan(value)) {
        maxHeight = std::max(maxHeight, value);
        minHeight = std::min(minHeight, value);
      }
    }
  }

  cv::Mat image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
      messageMap, elevationLayer, CV_8UC1, minHeight, maxHeight, image);

  int range = 100 * (maxHeight - minHeight);
  cv::imwrite(imageName + "_" + std::to_string(count++) + "_" +
                  std::to_string(range) + "cm.png",
              image);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
      "save_elevation_map_to_image",
      rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true));

  if (!node->get_parameter("frequency", frequency)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneExtractionROS] Could not read parameter `frequency`.");
    return 1;
  }
  if (!node->get_parameter("elevation_topic", elevationMapTopic)) {
    RCLCPP_ERROR(node->get_logger(),
                 "[ConvexPlaneExtractionROS] Could not read parameter "
                 "`elevation_topic`.");
    return 1;
  }
  if (!node->get_parameter("height_layer", elevationLayer)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneExtractionROS] Could not read parameter `height_layer`.");
    return 1;
  }
  if (!node->get_parameter("imageName", imageName)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneExtractionROS] Could not read parameter `imageName`.");
    return 1;
  }

  auto elevationMapSubscriber_ =
      node->create_subscription<grid_map_msgs::msg::GridMap>(
          elevationMapTopic, 1, std::bind(&callback, std::placeholders::_1));

  rclcpp::Rate rate(frequency);
  while (rclcpp::ok()) {
    rclcpp::spin(node);
    rate.sleep();
  }

  return 0;
}