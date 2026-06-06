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

# lz4 for ROS bag decompression
find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)
