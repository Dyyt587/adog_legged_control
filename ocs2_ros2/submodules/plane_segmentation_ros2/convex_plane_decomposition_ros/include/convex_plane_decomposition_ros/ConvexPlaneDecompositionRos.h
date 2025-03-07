#pragma once

#include <memory>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "rclcpp/rclcpp.hpp"

#include <Eigen/Geometry>

#include <grid_map_msgs/msg/grid_map.hpp>

#include <convex_plane_decomposition/Timer.h>

namespace convex_plane_decomposition {

// Forward declaration of the pipeline
class PlaneDecompositionPipeline;

class ConvexPlaneExtractionROS {
 public:
  ConvexPlaneExtractionROS(rclcpp::Node::SharedPtr node);

  ~ConvexPlaneExtractionROS();

 private:
  bool loadParameters();

  /**
   * Callback method for the incoming grid map message.
   * @param message the incoming message.
   */
  void callback(const grid_map_msgs::msg::GridMap& message);

  Eigen::Isometry3d getTransformToTargetFrame(const std::string& sourceFrame,
                                              const rclcpp::Time& time);

  // Parameters
  rclcpp::Node::SharedPtr node_;
  std::string elevationMapTopic_;
  std::string elevationLayer_;
  std::string targetFrameId_;
  double subMapWidth_;
  double subMapLength_;
  bool publishToController_;

  // ROS communication
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr
      elevationMapSubscriber_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr
      filteredmapPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      boundaryPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      insetPublisher_;
  rclcpp::Publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>::
      SharedPtr regionPublisher_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  // Pipeline
  std::unique_ptr<PlaneDecompositionPipeline> planeDecompositionPipeline_;

  // Timing
  Timer callbackTimer_;
};

}  // namespace convex_plane_decomposition
