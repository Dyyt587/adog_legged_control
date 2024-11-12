cmake_minimum_required(VERSION 3.8)
project(adog_legged_controller_test1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(adog_legged_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ruckig REQUIRED)
find_package(nav_msgs REQUIRED)

# add_library(
#   adog_legged_controller_test1
#   src/main.cpp
# )

# install(TARGETS
#   DESTINATION lib/${PROJECT_NAME})


include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)
aux_source_directory(src SRC_LIST)

add_executable(adog_legged_controller_test1 ${SRC_LIST})

target_link_libraries(adog_legged_controller_test1 ruckig::ruckig)
ament_target_dependencies(adog_legged_controller_test1 rclcpp adog_legged_interfaces sensor_msgs nav_msgs)


install(TARGETS
  adog_legged_controller_test1
  DESTINATION lib/${PROJECT_NAME})
# install(
#   TARGETS
#   adog_legged_controller_test
#   RUNTIME DESTINATION bin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
