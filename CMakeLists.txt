cmake_minimum_required(VERSION 3.25)
project(lidar_slam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Build options ─────────────────────────────────────────────────────────────
option(LIDAR_SLAM_BUILD_TESTS "Build the unit tests" ON)
option(LIDAR_SLAM_ENABLE_RERUN "Build with Rerun visualization support" OFF)

# ── Third-party deps ──────────────────────────────────────────────────────────
include(cmake/dependencies.cmake)

# ── Compile options ───────────────────────────────────────────────────────────
# Note: libc++ and the compiler are selected by cmake/toolchain-clang-libcxx.cmake
add_compile_options($<$<CONFIG:Debug>:-gdwarf-4>) # Debug symbols for every target

# ── Libraries ─────────────────────────────────────────────────────────────────
add_subdirectory(src/core)
add_subdirectory(src/io)
add_subdirectory(src/viz)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(lidar_slam src/main.cpp)
target_compile_options(lidar_slam PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(lidar_slam PRIVATE    
    # Internal    
    slam_io
    slam_viz
    
    # Third-party
    spdlog::spdlog
    CLI11::CLI11
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(LIDAR_SLAM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
