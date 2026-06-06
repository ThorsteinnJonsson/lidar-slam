cmake_minimum_required(VERSION 3.25)
project(lidar_slam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Dependencies ──────────────────────────────────────────────────────────────
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY git@github.com:jbeder/yaml-cpp.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(
    nanoflann
    GIT_REPOSITORY git@github.com:jlblancoc/nanoflann.git
    GIT_TAG        v1.6.2
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY git@github.com:gabime/spdlog.git
    GIT_TAG        v1.15.3
    GIT_SHALLOW    TRUE
)

set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(NANOFLANN_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(NANOFLANN_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(eigen yaml-cpp spdlog)

# nanoflann is header-only — skip its CMake to avoid uninstall target conflict with Eigen
FetchContent_GetProperties(nanoflann)
if(NOT nanoflann_POPULATED)
    FetchContent_Populate(nanoflann)
endif()
add_library(nanoflann_iface INTERFACE)
add_library(nanoflann::nanoflann ALIAS nanoflann_iface)
target_include_directories(nanoflann_iface INTERFACE ${nanoflann_SOURCE_DIR}/include)

# ── Targets ───────────────────────────────────────────────────────────────────
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

target_link_libraries(lidar_slam PRIVATE
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    nanoflann::nanoflann
    spdlog::spdlog
)
