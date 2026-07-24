# lidar-slam

LiDAR-SLAM based on [FAST-LIO2](https://github.com/hku-mars/FAST_LIO). C++23, built
with CMake + FetchContent.

## Prerequisites

- clang-18 / clang++-18 with libc++ (`libc++-18-dev`, `libc++abi-18-dev`)
- CMake ≥ 3.25 and Ninja
- `liblz4-dev` (ROS bag decompression)

Most dependencies (Eigen, Sophus, spdlog, yaml-cpp, nanoflann, CLI11, GoogleTest)
are fetched automatically via CMake FetchContent.

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
./build/release/lidar_slam \
    --format NTU_VIRAL \
    --sequence datasets/ntu_viral/eee_03 \
    --params config/ntu_viral.yaml
```

| Option | | |
| --- | --- | --- |
| `--format` | required | Dataset format: `NTU_VIRAL`, `HILTI_22`, or `FAST_LIVO2`. |
| `--sequence` | required | Sequence directory holding the `.bag`. |
| `--params` | required | Tunable parameters, see [config/README.md](config/README.md). Use the matching per-format file. |
| `--output` | optional | Output directory. Defaults to `evaluation/<sequence>`. |

Run from the repository root: the paths above are relative.

| Format | LiDAR | Params | Ground truth |
| --- | --- | --- | --- |
| `NTU_VIRAL` | Ouster OS1 | `config/ntu_viral.yaml` | in-bag Leica prism |
| `HILTI_22` | Hesai PandarXT-32 | `config/hilti_22.yaml` | sparse control points (`.txt`) |
| `FAST_LIVO2` | Livox Avia | `config/fast_livo2.yaml` | none |

## Evaluation

Each run writes three [TUM-format](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)
trajectories (`timestamp tx ty tz qx qy qz qw`) into the output directory:

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

## Visualization

Live 3D visualization (map, registered scans, trajectory) uses the
[Rerun](https://rerun.io) C++ SDK. It is opt-in: configure with the
`release-rerun` preset, which sets `LIDAR_SLAM_ENABLE_RERUN=ON`.

```bash
cmake --preset release-rerun
cmake --build --preset release-rerun
```

The first build compiles Arrow from source (a few minutes); it is cached
afterward. Normal `debug`/`release` builds do not pull in Rerun.

CMake fetches the SDK (linked into the binary). The **viewer** is a separate GUI
app you install yourself, and it must be the **same version** as the SDK
(0.33.1). Grab the prebuilt binary and drop it somewhere on `PATH`:

```bash
curl -fL -o ~/.cargo/bin/rerun \
  https://github.com/rerun-io/rerun/releases/download/0.33.1/rerun-cli-0.33.1-x86_64-unknown-linux-gnu
chmod +x ~/.cargo/bin/rerun
rerun --version   # expect 0.33.1
```

Alternatives: `cargo install rerun-cli --version 0.33.1 --locked` (compiles from
source), or `pipx install rerun-sdk==0.33.1`.

At run time the app calls `spawn()`, which launches the viewer automatically as
long as `rerun` is on `PATH`:

```bash
./build/release-rerun/lidar_slam
```

If the viewer is not found, `spawn()` logs a warning and the run continues with
visualization disabled. When the pinned SDK version changes, update the viewer to
match or it will refuse to connect.

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
