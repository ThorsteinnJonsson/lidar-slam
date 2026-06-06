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

```bash
ctest --test-dir build/debug --output-on-failure
ctest --test-dir build/debug -R <test_name> --output-on-failure
```

## Toolchain

- Compiler: clang-18 (`clang-18` / `clang++-18`)
- Build system: Ninja
- C++ standard: C++23 (`-std=c++23`, extensions off)
