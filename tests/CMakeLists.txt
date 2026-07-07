# Unit tests (GoogleTest). Add new *_test.cpp files (and any src/ units under
# test) to the sources below.
add_executable(unit_tests
    ikd_tree_test.cpp
    association_test.cpp
    s2_test.cpp
    state_test.cpp
    measurement_test.cpp
    iekf_test.cpp
    outlier_test.cpp
    iterated_ekf_test.cpp
    initializer_test.cpp
    map_crop_test.cpp
    local_map_test.cpp
    ros_deserializer_test.cpp
    ${CMAKE_SOURCE_DIR}/src/io/ros_deserializer.cpp
    ${CMAKE_SOURCE_DIR}/src/map/ikd_tree.cpp
    ${CMAKE_SOURCE_DIR}/src/map/association.cpp
    ${CMAKE_SOURCE_DIR}/src/map/local_map.cpp
    ${CMAKE_SOURCE_DIR}/src/imu/propagator.cpp
    ${CMAKE_SOURCE_DIR}/src/imu/initializer.cpp
    ${CMAKE_SOURCE_DIR}/src/estimator/measurement.cpp
    ${CMAKE_SOURCE_DIR}/src/estimator/iekf.cpp
    ${CMAKE_SOURCE_DIR}/src/estimator/outlier.cpp
    ${CMAKE_SOURCE_DIR}/src/estimator/iterated_ekf.cpp
)

target_include_directories(unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(unit_tests SYSTEM PRIVATE
    ${eigen_SOURCE_DIR}
    ${sophus_SOURCE_DIR}
)

target_compile_options(unit_tests PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(unit_tests PRIVATE
    GTest::gtest_main
    Eigen3::Eigen
    Sophus::Sophus
    nanoflann::nanoflann
)

# Register each TEST() with CTest so `ctest` discovers them individually.
include(GoogleTest)
gtest_discover_tests(unit_tests)
