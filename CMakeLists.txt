cmake_minimum_required(VERSION 3.25)
project(lidar_slam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Note: deliberately NOT named BUILD_TESTING — that magic name would also enable
# the dependencies' own test suites (Eigen registers 900+ tests off it).
option(LIDAR_SLAM_BUILD_TESTS "Build the unit tests" ON)

# Off by default: pulls in the Rerun C++ SDK (and builds Arrow from source the
# first time), which normal and CI builds have no need for.
option(LIDAR_SLAM_ENABLE_RERUN "Build with Rerun visualization support" OFF)

# libc++ and the compiler are selected by cmake/toolchain-clang-libcxx.cmake
# (referenced from CMakePresets.json) so the stdlib choice reaches sub-builds
# such as Rerun's Arrow ExternalProject.

include(cmake/dependencies.cmake)

# ── Targets ───────────────────────────────────────────────────────────────────
add_executable(lidar_slam
    src/main.cpp
    src/io/bag_reader.cpp
    src/io/calibration.cpp
    src/io/ros_deserializer.cpp
    src/imu/buffer.cpp
    src/imu/propagator.cpp
    src/imu/initializer.cpp
    src/preprocess/deskew.cpp
    src/preprocess/voxel_grid.cpp
    src/map/ikd_tree.cpp
    src/map/association.cpp
    src/map/local_map.cpp
    src/estimator/measurement.cpp
    src/estimator/iekf.cpp
    src/estimator/outlier.cpp
    src/estimator/iterated_ekf.cpp
    src/viz/visualizer.cpp
)

target_include_directories(lidar_slam PRIVATE src)
target_include_directories(lidar_slam SYSTEM PRIVATE ${eigen_SOURCE_DIR} ${sophus_SOURCE_DIR})

# Rerun visualization is opt-in; the visualizer wrapper compiles to no-ops when
# the SDK is not linked.
if(LIDAR_SLAM_ENABLE_RERUN)
    target_link_libraries(lidar_slam PRIVATE rerun_sdk)
    target_compile_definitions(lidar_slam PRIVATE LIDAR_SLAM_ENABLE_RERUN)
endif()

target_compile_options(lidar_slam PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    $<$<CONFIG:Debug>:-gdwarf-4>
)

target_link_libraries(lidar_slam PRIVATE
    Eigen3::Eigen
    Sophus::Sophus
    yaml-cpp::yaml-cpp
    nanoflann::nanoflann
    spdlog::spdlog
    ${LZ4_LIBRARIES}
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(LIDAR_SLAM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
