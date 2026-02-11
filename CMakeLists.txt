cmake_minimum_required(VERSION 3.16)
project(MotionPlanning VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find SFML 3.x
find_package(SFML COMPONENTS Graphics Window System CONFIG REQUIRED)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Core library
file(GLOB_RECURSE CORE_SOURCES "src/core/*.cpp")
add_library(motion_planning_core ${CORE_SOURCES})
target_link_libraries(motion_planning_core SFML::Graphics SFML::Window SFML::System)

# Algorithms library
file(GLOB_RECURSE ALGO_SOURCES "src/algorithms/*.cpp")
add_library(motion_planning_algorithms ${ALGO_SOURCES})
target_link_libraries(motion_planning_algorithms motion_planning_core)

# Visualization library
file(GLOB_RECURSE VIZ_SOURCES "src/visualization/*.cpp")
add_library(motion_planning_viz ${VIZ_SOURCES})
target_link_libraries(motion_planning_viz motion_planning_core SFML::Graphics SFML::Window SFML::System)

# Main executable
add_executable(motion_planning src/main.cpp)
target_link_libraries(motion_planning
    motion_planning_core
    motion_planning_algorithms
    motion_planning_viz
)

# Examples
add_executable(example_grid examples/grid_search.cpp)
target_link_libraries(example_grid motion_planning_core motion_planning_algorithms motion_planning_viz)

add_executable(example_rrt examples/rrt_demo.cpp)
target_link_libraries(example_rrt motion_planning_core motion_planning_algorithms motion_planning_viz)

# Copy assets if they exist
if(EXISTS "${PROJECT_SOURCE_DIR}/assets")
    file(COPY ${PROJECT_SOURCE_DIR}/assets DESTINATION ${CMAKE_BINARY_DIR})
endif()
