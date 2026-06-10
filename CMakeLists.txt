cmake_minimum_required(VERSION 3.25)
project(lidar_slam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Note: deliberately NOT named BUILD_TESTING — that magic name would also enable
# the dependencies' own test suites (Eigen registers 900+ tests off it).
option(LIDAR_SLAM_BUILD_TESTS "Build the unit tests" ON)

# Use libc++ globally so all dependencies share the same ABI
add_compile_options(-stdlib=libc++)
add_link_options(-stdlib=libc++ -lc++abi)

include(cmake/dependencies.cmake)

# ── Targets ───────────────────────────────────────────────────────────────────
add_executable(lidar_slam
    src/main.cpp
    src/io/bag_reader.cpp
    src/io/calibration.cpp
    src/io/ros_deserializer.cpp
    src/imu/buffer.cpp
    src/imu/propagator.cpp
    src/preprocess/deskew.cpp
    src/preprocess/voxel_grid.cpp
)

target_include_directories(lidar_slam PRIVATE src)
target_include_directories(lidar_slam SYSTEM PRIVATE ${eigen_SOURCE_DIR} ${sophus_SOURCE_DIR})

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
