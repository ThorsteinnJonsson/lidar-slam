cmake_minimum_required(VERSION 3.25)
project(lidar_slam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(lidar_slam src/main.cpp)

target_compile_options(lidar_slam PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -stdlib=libc++
    $<$<CONFIG:Debug>:-gdwarf-4>
)

target_link_options(lidar_slam PRIVATE
    -stdlib=libc++
    -lc++abi
)
