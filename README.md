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

## Evaluation

Each run writes three [TUM-format](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)
trajectories (`timestamp tx ty tz qx qy qz qw`) into `evaluation/`:

| File             | Contents                                                                       |
| ---------------- | ------------------------------------------------------------------------------ |
| `trajectory.tum` | estimated IMU pose in the SLAM world frame                                     |
| `prism.tum`      | estimate at the Leica prism (lever arm applied), for prism-to-prism comparison |
| `gt.tum`         | Leica prism ground truth from `/leica/pose/relative` (position only)           |

Metrics are computed offline with [evo](https://github.com/MichaelGrupp/evo).
The ground truth is position-only and in its own frame, so align before comparing
(`--align`, SE(3) Umeyama) and restrict the error to translation (`-r trans_part`).
Install once into a virtualenv:

```bash
python3 -m venv ~/.venvs/evo   # needs python3-venv for ensurepip
~/.venvs/evo/bin/pip install evo
source ~/.venvs/evo/bin/activate.fish # Or equivalent for bash
```

Absolute trajectory error (ATE) and relative pose error (RPE / drift):

```bash
evo_ape tum evaluation/gt.tum evaluation/prism.tum --align -r trans_part
evo_rpe tum evaluation/gt.tum evaluation/prism.tum --align -r trans_part \
        --delta 1 --delta_unit m
```

Add `--plot` for the trajectory overlay and error-over-time plots.

## Tests

Tests use GoogleTest and are built by default (disable with
`-DLIDAR_SLAM_BUILD_TESTS=OFF` at configure time). Run the suite via CTest:

```bash
ctest --test-dir build/debug --output-on-failure
```

Run a single test by name:

```bash
ctest --test-dir build/debug -R <test_name> --output-on-failure
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
