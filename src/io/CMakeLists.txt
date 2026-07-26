# Dataset I/O: ROS bag reader, message deserializers, calibration/params
# parsing, and the per-format dataset loaders. Depends on slam_core for the
# shared types (PointCloud, State, the config structs).
add_library(slam_io STATIC
    bag_reader.cpp
    calibration.cpp
    dataset_loader.cpp
    params.cpp
    result_writer.cpp
    ros_deserializer.cpp
)

target_include_directories(slam_io PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(slam_io
    PUBLIC slam_core yaml-cpp::yaml-cpp
    PRIVATE ${LZ4_LIBRARIES}
)

target_compile_options(slam_io PRIVATE -Wall -Wextra -Wpedantic)
