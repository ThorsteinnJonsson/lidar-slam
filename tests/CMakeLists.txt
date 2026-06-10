# Unit tests (GoogleTest). Add new *_test.cpp files to the sources below.
add_executable(unit_tests
    placeholder_test.cpp
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
    nanoflann::nanoflann
)

# Register each TEST() with CTest so `ctest` discovers them individually.
include(GoogleTest)
gtest_discover_tests(unit_tests)
