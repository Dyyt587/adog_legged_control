cmake_minimum_required(VERSION 3.14)
project(convex_plane_decomposition_msgs)

set(dependencies
        std_msgs
        geometry_msgs
        grid_map_msgs
)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/BoundingBox2d.msg"
        "msg/PlanarRegion.msg"
        "msg/PlanarTerrain.msg"
        "msg/Point2d.msg"
        "msg/Polygon2d.msg"
        "msg/PolygonWithHoles2d.msg"
        DEPENDENCIES std_msgs geometry_msgs grid_map_msgs
)

ament_package()