# Core SLAM algorithm: state, IMU propagation, estimator, map, preprocessing, etc.
add_library(slam_core STATIC
    imu/buffer.cpp
    imu/propagator.cpp
    imu/initializer.cpp
    preprocess/deskew.cpp
    preprocess/voxel_grid.cpp
    map/ikd_tree.cpp
    map/association.cpp
    map/local_map.cpp
    estimator/measurement.cpp
    estimator/iekf.cpp
    estimator/outlier.cpp
    estimator/iterated_ekf.cpp
)

target_include_directories(slam_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(slam_core SYSTEM PUBLIC
    ${eigen_SOURCE_DIR}
    ${sophus_SOURCE_DIR}
)
target_link_libraries(slam_core PUBLIC
    Eigen3::Eigen
    Sophus::Sophus
    nanoflann::nanoflann
)

target_compile_options(slam_core PRIVATE -Wall -Wextra -Wpedantic)
