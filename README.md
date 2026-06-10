# lidar-slam

LiDAR-SLAM based on [FAST-LIO2](https://github.com/hku-mars/FAST_LIO). C++23, built
with CMake + FetchContent.

## Prerequisites

- clang-18 / clang++-18 with libc++ (`libc++-18-dev`, `libc++abi-18-dev`)
- CMake ≥ 3.25 and Ninja
- `liblz4-dev` (ROS bag decompression)

Most dependencies (Eigen, Sophus, spdlog, yaml-cpp, nanoflann, GoogleTest) are
fetched automatically via CMake FetchContent.

## Build

Configure and build with a preset (`debug`, `release`, or `relwithdebinfo`):

```bash
cmake --preset debug
cmake --build --preset debug
```

The binary lands in `build/<preset>/lidar_slam`. Build outputs are per-preset, so
switching between them doesn't require a clean.

## Run

```bash
./build/debug/lidar_slam
```

Expects the NTU VIRAL dataset under `datasets/ntu_viral/eee_03` (calibration YAMLs
plus `eee_03.bag`).

## Tests

Tests use GoogleTest and are built by default (disable with
`-DLIDAR_SLAM_BUILD_TESTS=OFF` at configure time). Run the suite via CTest — scope
to the `tests` subdirectory so the fetched dependencies' own test suites don't get
pulled in:

```bash
ctest --test-dir build/debug/tests --output-on-failure
```

Run a single test by name:

```bash
ctest --test-dir build/debug/tests -R <test_name> --output-on-failure
```

Or run the test binary directly, with GoogleTest filters:

```bash
./build/debug/tests/unit_tests --gtest_filter='Placeholder.*'
```

## Formatting

`clang-format-18` (Google style). Run after every change:

```bash
find src tests -name '*.h' -o -name '*.cpp' | xargs clang-format-18 -i
```
