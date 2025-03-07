cmake_minimum_required(VERSION 3.5)
project(ocs2_robotic_assets)

find_package(ament_cmake REQUIRED)

###################################
## catkin specific configuration ##
###################################

include_directories(
  include
)
###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

#############
## Install ##
#############

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
