cmake_minimum_required(VERSION 3.8)
project(unitree_go2_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY meshes xacro launch config urdf world
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()