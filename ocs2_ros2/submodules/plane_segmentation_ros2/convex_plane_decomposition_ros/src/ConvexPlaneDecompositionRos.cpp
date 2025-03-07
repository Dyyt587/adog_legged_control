#include "convex_plane_decomposition_ros/ConvexPlaneDecompositionRos.h"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvProcessing.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>

#include <opencv2/core/eigen.hpp>
#include "convex_plane_decomposition_ros/MessageConversion.h"
#include "convex_plane_decomposition_ros/ParameterLoading.h"
#include "convex_plane_decomposition_ros/RosVisualizations.h"

namespace grid_map {
GridMap getTransformedMap(GridMap&& gridMapSource,
                          const Eigen::Isometry3d& transform,
                          const std::string& heightLayerName,
                          const std::string& newFrameId) {
  // Check if height layer is valid.
  if (!gridMapSource.exists(heightLayerName)) {
    throw std::out_of_range("GridMap::getTransformedMap(...) : No map layer '" +
                            heightLayerName + "' available.");
  }

  // Check if transformation is z aligned.
  if (std::abs(transform(2, 2) - 1) >= 0.05) {
    throw std::invalid_argument("The given transform is not Z aligned!");
  }

  auto yawPitchRoll =
      transform.rotation().eulerAngles(2, 1, 0);  // Double check convention!
  double rotationAngle = yawPitchRoll.x() * 180 / CV_PI;
  if (std::abs(yawPitchRoll.y()) >= 3 &&
      std::abs(yawPitchRoll.z()) >=
          3) {  // Resolve yaw ambiguity in euler angles.
    rotationAngle += 180;
  }

  gridMapSource.convertToDefaultStartIndex();

  // Create the rotated gridMap.
  GridMap transformedMap(gridMapSource.getLayers());
  transformedMap.setBasicLayers(gridMapSource.getBasicLayers());
  transformedMap.setTimestamp(gridMapSource.getTimestamp());
  transformedMap.setFrameId(newFrameId);

  // openCV rotation parameters, initalized on first layer.
  cv::Mat imageRotationMatrix;
  cv::Rect2f boundingBox;

  bool firstLayer = true;
  for (const auto& layer : gridMapSource.getLayers()) {
    cv::Mat imageSource, imageResult;

    // From gridMap to openCV image. Assumes defaultStartIndex.
    cv::eigen2cv(gridMapSource[layer], imageSource);

    // Calculate transformation matrix and update geometry of the resulting grid
    // map.
    if (firstLayer) {
      // Get rotation matrix for rotating the image around its center in pixel
      // coordinates. See
      // https://answers.opencv.org/question/82708/rotation-center-confusion/
      cv::Point2f imageCenter = cv::Point2f((imageSource.cols - 1) / 2.0,
                                            (imageSource.rows - 1) / 2.0);

      imageRotationMatrix =
          cv::getRotationMatrix2D(imageCenter, rotationAngle, 1.0);
      boundingBox =
          cv::RotatedRect(cv::Point2f(0, 0), imageSource.size(), rotationAngle)
              .boundingRect2f();

      // Adjust transformation matrix. See
      // https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#gafbbc470ce83812914a70abfb604f4326
      // and
      // https://stackoverflow.com/questions/22041699/rotate-an-image-without-cropping-in-opencv-in-c
      imageRotationMatrix.at<double>(0, 2) +=
          boundingBox.width / 2.0 - imageSource.cols / 2.0;
      imageRotationMatrix.at<double>(1, 2) +=
          boundingBox.height / 2.0 - imageSource.rows / 2.0;

      // Calculate the new center of the gridMap.
      Position3 newCenter =
          transform * Position3{(gridMapSource.getPosition().x()),
                                (gridMapSource.getPosition().y()), 0.0};

      // Set the size of the rotated gridMap.
      transformedMap.setGeometry(
          {boundingBox.height * gridMapSource.getResolution(),
           boundingBox.width * gridMapSource.getResolution()},
          gridMapSource.getResolution(),
          Position(newCenter.x(), newCenter.y()));
      firstLayer = false;
    }

    // Rotate the layer.
    imageResult = cv::Mat(boundingBox.size(), CV_32F,
                          std::numeric_limits<double>::quiet_NaN());
    cv::warpAffine(imageSource, imageResult, imageRotationMatrix,
                   boundingBox.size(), cv::INTER_NEAREST,
                   cv::BORDER_TRANSPARENT);

    // Copy result into gridMapLayer. Assumes default start index.
    Matrix resultLayer;
    cv::cv2eigen(imageResult, resultLayer);
    transformedMap.add(layer, resultLayer);
  }

  // Add height translation.
  grid_map::Matrix heightLayer = transformedMap[heightLayerName];
  transformedMap[heightLayerName] =
      heightLayer.array() + transform.translation().z();

  return transformedMap;
}
}  // namespace grid_map

namespace convex_plane_decomposition {

ConvexPlaneExtractionROS::ConvexPlaneExtractionROS(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)),
      tfBuffer_(node_->get_clock()),
      tfListener_(tfBuffer_) {
  bool parametersLoaded = loadParameters();

  if (parametersLoaded) {
    elevationMapSubscriber_ =
        node_->create_subscription<grid_map_msgs::msg::GridMap>(
            elevationMapTopic_, 1,
            std::bind(&ConvexPlaneExtractionROS::callback, this,
                      std::placeholders::_1));
    filteredmapPublisher_ =
        node_->create_publisher<grid_map_msgs::msg::GridMap>("filtered_map", 1);
    boundaryPublisher_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "boundaries", 1);
    insetPublisher_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>("insets",
                                                                      1);
    regionPublisher_ = node_->create_publisher<
        convex_plane_decomposition_msgs::msg::PlanarTerrain>("planar_terrain",
                                                             1);
  }
}

ConvexPlaneExtractionROS::~ConvexPlaneExtractionROS() {
  if (callbackTimer_.getNumTimedIntervals() > 0 &&
      planeDecompositionPipeline_ != nullptr) {
    std::stringstream infoStream;
    infoStream << "\n##########################################################"
                  "##############\n";
    infoStream << "The benchmarking is computed over "
               << callbackTimer_.getNumTimedIntervals() << " iterations. \n";
    infoStream << "PlaneExtraction Benchmarking    : Average time [ms], Max "
                  "time [ms]\n";
    auto printLine = [](std::string name, const Timer& timer) {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2);
      ss << "\t" << name << "\t: " << std::setw(17)
         << timer.getAverageInMilliseconds() << ", " << std::setw(13)
         << timer.getMaxIntervalInMilliseconds() << "\n";
      return ss.str();
    };
    infoStream << printLine("Pre-process        ",
                            planeDecompositionPipeline_->getPrepocessTimer());
    infoStream << printLine(
        "Sliding window     ",
        planeDecompositionPipeline_->getSlidingWindowTimer());
    infoStream << printLine(
        "Contour extraction ",
        planeDecompositionPipeline_->getContourExtractionTimer());
    infoStream << printLine("Post-process       ",
                            planeDecompositionPipeline_->getPostprocessTimer());
    infoStream << printLine("Total callback     ", callbackTimer_);
    std::cerr << infoStream.str() << std::endl;
  }
}

bool ConvexPlaneExtractionROS::loadParameters() {
  if (!node_->get_parameter("elevation_topic", elevationMapTopic_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[ConvexPlaneExtractionROS] Could not read parameter "
                 "`elevation_topic`.");
    return false;
  }
  if (!node_->get_parameter("target_frame_id", targetFrameId_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[ConvexPlaneExtractionROS] Could not read parameter "
                 "`target_frame_id`.");
    return false;
  }
  if (!node_->get_parameter("height_layer", elevationLayer_)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[ConvexPlaneExtractionROS] Could not read parameter `height_layer`.");
    return false;
  }
  if (!node_->get_parameter("submap.width", subMapWidth_)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[ConvexPlaneExtractionROS] Could not read parameter `submap.width`.");
    return false;
  }
  if (!node_->get_parameter("submap.length", subMapLength_)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[ConvexPlaneExtractionROS] Could not read parameter `submap.length`.");
    return false;
  }
  if (!node_->get_parameter("publish_to_controller", publishToController_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[ConvexPlaneExtractionROS] Could not read parameter "
                 "`publish_to_controller`.");
    return false;
  }

  PlaneDecompositionPipeline::Config config;
  config.preprocessingParameters =
      loadPreprocessingParameters(node_.get(), "preprocessing.");
  config.contourExtractionParameters =
      loadContourExtractionParameters(node_.get(), "contour_extraction.");
  config.ransacPlaneExtractorParameters = loadRansacPlaneExtractorParameters(
      node_.get(), "ransac_plane_refinement.");
  config.slidingWindowPlaneExtractorParameters =
      loadSlidingWindowPlaneExtractorParameters(
          node_.get(), "sliding_window_plane_extractor.");
  config.postprocessingParameters =
      loadPostprocessingParameters(node_.get(), "postprocessing.");

  planeDecompositionPipeline_ =
      std::make_unique<PlaneDecompositionPipeline>(config);

  return true;
}

void ConvexPlaneExtractionROS::callback(
    const grid_map_msgs::msg::GridMap& message) {
  callbackTimer_.startTimer();

  // Convert message to map.
  grid_map::GridMap messageMap;
  std::vector<std::string> layers{elevationLayer_};
  grid_map::GridMapRosConverter::fromMessage(message, messageMap, layers, false,
                                             false);
  if (!containsFiniteValue(messageMap.get(elevationLayer_))) {
    RCLCPP_WARN(node_->get_logger(),
                "[ConvexPlaneExtractionROS] map does not contain any values");
    callbackTimer_.endTimer();
    return;
  }

  // Transform map if necessary
  if (targetFrameId_ != messageMap.getFrameId()) {
    std::string errorMsg;
    rclcpp::Time timeStamp =
        rclcpp::Time(0);  // Use Time(0) to get the latest transform.
    if (tfBuffer_.canTransform(targetFrameId_, messageMap.getFrameId(),
                               timeStamp, rclcpp::Duration::from_nanoseconds(0),
                               &errorMsg)) {
      const auto transform =
          getTransformToTargetFrame(messageMap.getFrameId(), timeStamp);

      messageMap = grid_map::getTransformedMap(std::move(messageMap), transform,
                                               elevationLayer_, targetFrameId_);
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "[ConvexPlaneExtractionROS] " << errorMsg);
      callbackTimer_.endTimer();
      return;
    }
  }

  // Extract submap
  bool success;
  const grid_map::Position submapPosition = [&]() {
    // The map center might be between cells. Taking the submap there can result
    // in changing submap dimensions. project map center to an index and index
    // to center s.t. we get the location of a cell.
    grid_map::Index centerIndex;
    grid_map::Position centerPosition;
    messageMap.getIndex(messageMap.getPosition(), centerIndex);
    messageMap.getPosition(centerIndex, centerPosition);
    return centerPosition;
  }();
  grid_map::GridMap elevationMap = messageMap.getSubmap(
      submapPosition, Eigen::Array2d(subMapLength_, subMapWidth_), success);
  if (!success) {
    RCLCPP_WARN(node_->get_logger(),
                "[ConvexPlaneExtractionROS] Could not extract submap");
    callbackTimer_.endTimer();
    return;
  }
  const grid_map::Matrix elevationRaw = elevationMap.get(elevationLayer_);

  // Run pipeline.
  planeDecompositionPipeline_->update(std::move(elevationMap), elevationLayer_);
  auto& planarTerrain = planeDecompositionPipeline_->getPlanarTerrain();

  // Publish terrain
  if (publishToController_) {
    regionPublisher_->publish(toMessage(planarTerrain));
  }

  // --- Visualize in Rviz --- Not published to the controller
  // Add raw map
  planarTerrain.gridMap.add("elevation_raw", elevationRaw);

  // Add segmentation
  planarTerrain.gridMap.add("segmentation");
  planeDecompositionPipeline_->getSegmentation(
      planarTerrain.gridMap.get("segmentation"));

  grid_map_msgs::msg::GridMap outputMessage =
      *(grid_map::GridMapRosConverter::toMessage(planarTerrain.gridMap));
  filteredmapPublisher_->publish(outputMessage);

  const double lineWidth = 0.005;  // [m] RViz marker size
  boundaryPublisher_->publish(convertBoundariesToRosMarkers(
      planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
      planarTerrain.gridMap.getTimestamp(), lineWidth));
  insetPublisher_->publish(convertInsetsToRosMarkers(
      planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
      planarTerrain.gridMap.getTimestamp(), lineWidth));

  callbackTimer_.endTimer();
}

Eigen::Isometry3d ConvexPlaneExtractionROS::getTransformToTargetFrame(
    const std::string& sourceFrame,
    const rclcpp::Time& time) {
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped =
        tfBuffer_.lookupTransform(targetFrameId_, sourceFrame, time);
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(), "[ConvexPlaneExtractionROS] %s",
                 ex.what());
    return Eigen::Isometry3d::Identity();
  }

  Eigen::Isometry3d transformation;

  // Extract translation.
  transformation.translation().x() = transformStamped.transform.translation.x;
  transformation.translation().y() = transformStamped.transform.translation.y;
  transformation.translation().z() = transformStamped.transform.translation.z;

  // Extract rotation.
  Eigen::Quaterniond rotationQuaternion(transformStamped.transform.rotation.w,
                                        transformStamped.transform.rotation.x,
                                        transformStamped.transform.rotation.y,
                                        transformStamped.transform.rotation.z);
  transformation.linear() = rotationQuaternion.toRotationMatrix();
  return transformation;
}

}  // namespace convex_plane_decomposition
