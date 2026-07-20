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

FetchContent_Declare(
    sophus
    GIT_REPOSITORY git@github.com:strasdat/Sophus.git
    GIT_TAG        main
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY git@github.com:google/googletest.git
    GIT_TAG        v1.15.2
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(
    cli11
    GIT_REPOSITORY git@github.com:CLIUtils/CLI11.git
    GIT_TAG        v2.4.2
    GIT_SHALLOW    TRUE
)

set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SOPHUS_USE_BASIC_LOGGING ON CACHE BOOL "" FORCE)
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(CLI11_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(CLI11_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CLI11_BUILD_DOCS OFF CACHE BOOL "" FORCE)
# Suppress the dependencies' own test suites (Eigen et al. key off BUILD_TESTING).
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(eigen yaml-cpp spdlog sophus cli11)

# GoogleTest only when building tests (keeps non-test builds lean).
if(LIDAR_SLAM_BUILD_TESTS)
    FetchContent_MakeAvailable(googletest)
endif()

# nanoflann is header-only — skip its CMake to avoid uninstall target conflict with Eigen
FetchContent_GetProperties(nanoflann)
if(NOT nanoflann_POPULATED)
    FetchContent_Populate(nanoflann)
endif()
add_library(nanoflann_iface INTERFACE)
add_library(nanoflann::nanoflann ALIAS nanoflann_iface)
target_include_directories(nanoflann_iface INTERFACE ${nanoflann_SOURCE_DIR}/include)

# lz4 for ROS bag decompression
find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)

# Rerun C++ SDK for visualization (opt-in). Builds Arrow from source; the libc++
# toolchain file (CMakePresets) propagates in so Arrow links against libc++.
if(LIDAR_SLAM_ENABLE_RERUN)
    FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/download/0.33.1/rerun_cpp_sdk.zip
    )
    FetchContent_MakeAvailable(rerun_sdk)
endif()
