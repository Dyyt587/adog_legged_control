cmake_minimum_required(VERSION 3.14)
project(convex_plane_decomposition)

set(dependencies
        cgal5_colcon
        grid_map_filters_rsl
        OpenCV
        Boost
)

find_package(ament_cmake REQUIRED)
find_package(cgal5_colcon REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_filters_rsl REQUIRED)

# CGAL dependencies (QUIET because they cannot be found by Clion)
find_package(GMP 4.2 QUIET)
find_package(MPFR 2.2.1 QUIET)
include(cmake/ConfigExtras.cmake)

# OpenCv
find_package(OpenCV REQUIRED)

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
        src/contour_extraction/ContourExtraction.cpp
        src/contour_extraction/Upsampling.cpp
        src/ransac/RansacPlaneExtractor.cpp
        src/sliding_window_plane_extraction/SlidingWindowPlaneExtractor.cpp
        src/ConvexRegionGrowing.cpp
        src/Draw.cpp
        src/GridMapPreprocessing.cpp
        src/LoadGridmapFromImage.cpp
        src/PlanarRegion.cpp
        src/PlaneDecompositionPipeline.cpp
        src/Postprocessing.cpp
        src/SegmentedPlaneProjection.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
        gmp
        mpfr
)
target_compile_options(${PROJECT_NAME} PUBLIC -DCGAL_HAS_THREADS)

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

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_${PROJECT_NAME}
            test/testConvexApproximation.cpp
            test/testPipeline.cpp
            test/testPlanarRegion.cpp
            test/testRegionGrowing.cpp
            test/testUpsampling.cpp
    )
    ament_target_dependencies(test_${PROJECT_NAME}
            ${dependencies}
    )
    target_link_libraries(test_${PROJECT_NAME}
            ${PROJECT_NAME}
            gmp
            mpfr
    )
endif ()

ament_package(CONFIG_EXTRAS "cmake/ConfigExtras.cmake")