#include "convex_plane_decomposition_ros/ConvexPlaneDecompositionRos.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
      "convex_plane_decomposition_ros",
      rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true));

  double frequency;
  if (!node->get_parameter("frequency", frequency)) {
    RCLCPP_ERROR(
        node->get_logger(),
        "[ConvexPlaneDecompositionNode] Could not read parameter `frequency`.");
    return 1;
  }

  convex_plane_decomposition::ConvexPlaneExtractionROS
      convex_plane_decomposition_ros(node);

  rclcpp::Rate rate(frequency);
  while (rclcpp::ok()) {
    rclcpp::spin(node);
    rate.sleep();
  }
  return 0;
}
