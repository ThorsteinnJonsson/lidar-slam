# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

LiDAR-SLAM implementation based on [FAST-LIO2](https://github.com/hku-mars/FAST_LIO). A C++ project using CMake and vcpkg for dependency management.

## Build

Uses `CMakePresets.json` with clang-18 and Ninja. Presets: `release`, `debug`, `relwithdebinfo`.

```bash
cmake --preset debug
cmake --build --preset debug
```

The binary lands in `build/<preset>/lidar_slam`. Build outputs are per-preset so switching between them doesn't require a clean.

`compile_commands.json` is generated automatically into the preset's build dir. Symlink it to the root for LSP tools:
```bash
ln -sf build/debug/compile_commands.json compile_commands.json
```

## Testing

GoogleTest, fetched via FetchContent.

```bash
ctest --test-dir build/debug --output-on-failure
ctest --test-dir build/debug -R <test_name> --output-on-failure
```

Add new test files to `tests/CMakeLists.txt`. Our option is `LIDAR_SLAM_BUILD_TESTS`
(ON by default) — deliberately not named `BUILD_TESTING`, and we force `BUILD_TESTING
OFF` for the fetched dependencies so their test suites don't register into CTest
(Eigen otherwise adds 900+). If dependency tests ever show up in `ctest`, reconfigure
from a clean build dir — they come from stale CTest files.

## Toolchain

- Compiler: clang-18 (`clang-18` / `clang++-18`), stdlib: libc++-18 (set globally so all deps share the same ABI)
- Build system: Ninja
- C++ standard: C++23 (`-std=c++23`, extensions off)
- Formatting: `clang-format-18` (Google style, `Standard: Latest`). Run after every code change:
  ```bash
  find src tests -name "*.h" -o -name "*.cpp" | xargs clang-format-18 -i
  ```

## Key dependencies

Managed via CMake FetchContent (`cmake/dependencies.cmake`):

| Library | Purpose |
|---|---|
| Eigen 3.4 | Linear algebra |
| Sophus | SO3/SE3 Lie group math (chosen for Ceres/g2o compatibility) |
| spdlog | Logging |
| yaml-cpp | Calibration file parsing |
| nanoflann | KD-tree for nearest-neighbour search |

System deps: `liblz4-dev` (ROS bag decompression), `libc++-18-dev`, `libc++abi-18-dev`, `ninja-build`.
