# Live visualization using Rerun.
# Ensure LIDAR_SLAM_ENABLE_RERUN is set (in src/CMakeLists.txt) to use this.
add_library(slam_viz STATIC visualizer.cpp)

target_include_directories(slam_viz PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(slam_viz
    PUBLIC slam_core # Needed for map
    PRIVATE spdlog::spdlog
)

if(LIDAR_SLAM_ENABLE_RERUN)
    target_link_libraries(slam_viz PRIVATE rerun_sdk)
    target_compile_definitions(slam_viz PRIVATE LIDAR_SLAM_ENABLE_RERUN)
endif()

target_compile_options(slam_viz PRIVATE -Wall -Wextra -Wpedantic)
