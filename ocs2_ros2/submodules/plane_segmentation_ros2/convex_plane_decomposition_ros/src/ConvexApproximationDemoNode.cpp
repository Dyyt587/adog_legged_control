//
// Created by rgrandia on 24.06.20.
//

#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/GeometryUtils.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>

#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>

const std::string frameId = "odom";
std::mutex terrainMutex;
std::unique_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr;

void callback(
    const convex_plane_decomposition_msgs::msg::PlanarTerrain::ConstSharedPtr&
        msg) {
  std::unique_ptr<convex_plane_decomposition::PlanarTerrain> newTerrain(
      new convex_plane_decomposition::PlanarTerrain(
          convex_plane_decomposition::fromMessage(*msg)));

  std::lock_guard<std::mutex> lock(terrainMutex);
  planarTerrainPtr.swap(newTerrain);
}

geometry_msgs::msg::PointStamped toMarker(const Eigen::Vector3d& position,
                                          const std_msgs::msg::Header& header) {
  geometry_msgs::msg::PointStamped sphere;
  sphere.header = header;
  sphere.point.x = position.x();
  sphere.point.y = position.y();
  sphere.point.z = position.z();
  return sphere;
}

float randomFloat(float a, float b) {
  float random = ((float)rand()) / (float)RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("convex_approximation_demo_node");

  // Publishers for visualization
  auto positionPublisher =
      node->create_publisher<geometry_msgs::msg::PointStamped>("queryPosition",
                                                               1);
  auto projectionPublisher =
      node->create_publisher<geometry_msgs::msg::PointStamped>(
          "projectedQueryPosition", 1);
  auto convexTerrainPublisher =
      node->create_publisher<geometry_msgs::msg::PolygonStamped>(
          "convex_terrain", 1);
  auto terrainSubscriber = node->create_subscription<
      convex_plane_decomposition_msgs::msg::PlanarTerrain>(
      "/convex_plane_decomposition_ros/planar_terrain", 1,
      std::bind(&callback, std::placeholders::_1));

  // Node loop
  rclcpp::Rate rate(1.0);
  while (rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(terrainMutex);
      if (planarTerrainPtr) {
        const auto& map = planarTerrainPtr->gridMap;

        // Find edges.
        double maxX = map.getPosition().x() + map.getLength().x() * 0.5;
        double minX = map.getPosition().x() - map.getLength().x() * 0.5;
        double maxY = map.getPosition().y() + map.getLength().y() * 0.5;
        double minY = map.getPosition().y() - map.getLength().y() * 0.5;

        Eigen::Vector3d query{randomFloat(minX, maxX), randomFloat(minY, maxY),
                              randomFloat(0.0, 1.0)};
        auto penaltyFunction = [](const Eigen::Vector3d& projectedPoint) {
          return 0.0;
        };

        const auto projection = getBestPlanarRegionAtPositionInWorld(
            query, planarTerrainPtr->planarRegions, penaltyFunction);

        int numberOfVertices = 16;
        double growthFactor = 1.05;
        const auto convexRegion =
            convex_plane_decomposition::growConvexPolygonInsideShape(
                projection.regionPtr->boundaryWithInset.boundary,
                projection.positionInTerrainFrame, numberOfVertices,
                growthFactor);

        std_msgs::msg::Header header;
        header.stamp.nanosec = planarTerrainPtr->gridMap.getTimestamp();
        header.frame_id = frameId;

        auto convexRegionMsg = convex_plane_decomposition::to3dRosPolygon(
            convexRegion, projection.regionPtr->transformPlaneToWorld, header);

        convexTerrainPublisher->publish(convexRegionMsg);
        positionPublisher->publish(toMarker(query, header));
        projectionPublisher->publish(
            toMarker(projection.positionInWorld, header));
      }
    }

    rclcpp::spin(node);
    rate.sleep();
  }

  return 0;
}