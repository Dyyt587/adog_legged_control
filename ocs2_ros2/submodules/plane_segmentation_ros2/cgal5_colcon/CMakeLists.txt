cmake_minimum_required(VERSION 3.14)
project(cgal5_colcon)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

set(VERSION 5.3)

file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/include)

set(CGAL_VERSION 5.3)
ExternalProject_Add(cgal
        URL https://github.com/CGAL/cgal/archive/refs/tags/v${CGAL_VERSION}.tar.gz
        UPDATE_COMMAND ""
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
        -DCMAKE_BUILD_TYPE:STRING=Release
        BUILD_COMMAND $(MAKE)
        INSTALL_COMMAND $(MAKE) install
)

# # Install the library where ament_cmake expects them
# set_target_properties(cgal PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)

# Propagate dependencies
# ament_export_dependencies(cgal)  
ament_export_include_directories(include)
ament_package(CONFIG_EXTRAS "cmake/cgal-extras.cmake.in")