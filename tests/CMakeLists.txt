# Unit tests (GoogleTest)
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
    params_test.cpp
    point_cloud_test.cpp
    result_writer_test.cpp
)

target_compile_options(unit_tests PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(unit_tests PRIVATE
    GTest::gtest_main
    slam_io
)

include(GoogleTest)
gtest_discover_tests(unit_tests)
