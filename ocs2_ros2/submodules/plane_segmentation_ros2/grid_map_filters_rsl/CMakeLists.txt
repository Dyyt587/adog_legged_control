# Project configuration
cmake_minimum_required(VERSION 3.14)
project(grid_map_filters_rsl)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        grid_map_cv
        grid_map_core
)

find_package(ament_cmake REQUIRED)

find_package(grid_map_cv REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########
add_library(${PROJECT_NAME}
        src/GridMapDerivative.cpp
        src/inpainting.cpp
        src/lookup.cpp
        src/smoothing.cpp
        src/processing.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

## GTest.
if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_${PROJECT_NAME}
            test/TestDerivativeFilter.cpp
            test/TestFilters.cpp
            test/TestLookup.cpp
    )
    ament_target_dependencies(test_${PROJECT_NAME} ${dependencies})
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

    # Generate test coverage report
    find_package(cmake_code_coverage QUIET)
    if (cmake_code_coverage_FOUND)
        add_gtest_coverage(
                TEST_BUILD_TARGETS test_${PROJECT_NAME}
        )
    endif (cmake_code_coverage_FOUND)
endif ()

#############
## Install ##
#############
install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

###########
## Clang ##
###########
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    add_default_clang_tooling()
endif (cmake_clang_tools_FOUND)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()