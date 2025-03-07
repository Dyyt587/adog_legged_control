# Plane Segmentation ROS2
## Overview
This is a C++ ROS2 package for extracting convex polygons from elevation maps.
In a first step, the terrain is segmented into planes, and their non-convex contour is extracted.
In a second step, a local convex approximation can be obtained.

fork from: https://github.com/leggedrobotics/elevation_mapping_cupy/tree/main/plane_segmentation

## Usage

### Install dependencies
```bash
sudo apt-get install libgmp-dev libmpfr-dev
```

### Build
```bash
colcon build --packages-select build convex_plane_decomposition_ros
```

### Run as node
```bash
ros2 launch convex_plane_decomposition_ros convex_plane_decomposition.launch.py
```

### Run demo
```bash
ros2 launch convex_plane_decomposition_ros demo.launch.py
```

### Convex approximation demo
A simple 2D demo the convex inner approximation is available:
```bash
ros2 run convex_plane_decomposition_ros convex_plane_decomposition_ros_TestShapeGrowing
```

### Parameters
You can select input map topics, pipeline parameters etc. in the respective yaml file in
```bash
convex_plane_decomposition_ros/config/
```
Some other parameters are set through the launch files.
