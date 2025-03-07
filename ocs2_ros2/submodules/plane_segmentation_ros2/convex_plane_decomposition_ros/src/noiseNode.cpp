//
// Created by rgrandia on 25.10.21.
//

#include "rclcpp/rclcpp.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <grid_map_filters_rsl/inpainting.hpp>
#include <grid_map_filters_rsl/smoothing.hpp>

double noiseUniform;
double noiseGauss;
double outlierPercentage;
bool blur;
double frequency;
std::string elevationMapTopicIn;
std::string elevationMapTopicOut;
std::string elevationLayer;
rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher;
grid_map::GridMap::Matrix noiseLayer;

void createNoise(size_t row, size_t col) {
  // Box-Muller Transform
  grid_map::GridMap::Matrix u1 =
      0.5 * grid_map::GridMap::Matrix::Random(row, col).array() + 0.5;
  grid_map::GridMap::Matrix u2 =
      0.5 * grid_map::GridMap::Matrix::Random(row, col).array() + 0.5;
  grid_map::GridMap::Matrix gauss01 =
      u1.binaryExpr(u2, [&](float v1, float v2) {
        return static_cast<float>(std::sqrt(-2.0f * log(v1)) *
                                  cos(2.0f * M_PIf32 * v2));
      });

  noiseLayer = noiseUniform * grid_map::GridMap::Matrix::Random(row, col) +
               noiseGauss * gauss01;
}

void callback(const grid_map_msgs::msg::GridMap::ConstSharedPtr& message) {
  grid_map::GridMap messageMap;
  grid_map::GridMapRosConverter::fromMessage(*message, messageMap);

  if (blur) {
    // Copy!
    auto originalMap = messageMap.get(elevationLayer);

    // Blur (remove nan -> filter -> put back nan
    grid_map::inpainting::minValues(messageMap, elevationLayer, "i");
    grid_map::smoothing::boxBlur(messageMap, "i", elevationLayer, 3, 1);
    messageMap.get(elevationLayer) =
        (originalMap.array().isFinite())
            .select(messageMap.get(elevationLayer), originalMap);
  }

  auto& elevation = messageMap.get(elevationLayer);
  if (noiseLayer.size() != elevation.size()) {
    createNoise(elevation.rows(), elevation.cols());
  }

  elevation += noiseLayer;

  grid_map_msgs::msg::GridMap messageMapOut =
      *(grid_map::GridMapRosConverter::toMessage(messageMap));
  publisher->publish(messageMapOut);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
      "noise_node", rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true));

  if (!node->get_parameter("frequency", frequency)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneExtractionROS::NoiseNode] Could not read parameter "
        "`frequency`.");
    return 1;
  }
  if (!node->get_parameter("noiseGauss", noiseGauss)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter "
        "`noiseGauss`.");
    return 1;
  }
  if (!node->get_parameter("noiseUniform", noiseUniform)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter "
        "`noiseUniform`.");
    return 1;
  }
  if (!node->get_parameter("blur", blur)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter "
        "`blur`.");
    return 1;
  }
  if (!node->get_parameter("outlier_percentage", outlierPercentage)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter "
        "`outlier_percentage`.");
    return 1;
  }
  if (!node->get_parameter("elevation_topic_in", elevationMapTopicIn)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter "
        "`elevation_topic_in`.");
    return 1;
  }
  if (!node->get_parameter("elevation_topic_out", elevationMapTopicOut)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter "
        "`elevation_topic_out`.");
    return 1;
  }
  if (!node->get_parameter("height_layer", elevationLayer)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter "
        "`height_layer`.");
    return 1;
  }

  publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
      elevationMapTopicOut, 1);
  auto elevationMapSubscriber_ =
      node->create_subscription<grid_map_msgs::msg::GridMap>(
          elevationMapTopicIn, 1, std::bind(&callback, std::placeholders::_1));

  rclcpp::Rate rate(frequency);
  while (rclcpp::ok()) {
    rclcpp::spin(node);
    rate.sleep();
  }

  return 0;
}