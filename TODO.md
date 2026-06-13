# SLAM Framework TODO

Roughly following FAST-LIO2. Dataset: NTU VIRAL (ROS1 bag, Ouster OS1-16 LiDAR + IMU).

---

## Phase 1: Foundation & I/O

- [x] Add Eigen, yaml-cpp, spdlog, nanoflann as dependencies (CMake FetchContent)
- [x] Core data types: `ImuMeasurement`, `PointCloud` (`src/types.h`)
- [x] ROS1 bag reader — minimal in-house parser (`src/io/bag_reader.h/cpp`), lz4 + uncompressed chunks
- [x] Parse IMU messages (`/imu/imu`) and LiDAR point clouds (`/os1_cloud_node1/points`) (`src/io/ros_deserializer.h/cpp`)
- [x] Smoke test: 70,475 IMU measurements and 1,814 point clouds from `eee_03.bag`
- [x] Load extrinsic calibration from YAML (`T_Body_Lidar`, `T_Body_Imu`) — strips `%YAML:1.0` directive and duplicate `gyro_std` key before yaml-cpp parsing (`src/io/calibration.h/cpp`)

---

## Phase 2: Math Utilities & IMU Propagation

- [x] SO3 / SE3 Lie group operations — using Sophus (chosen for Ceres/g2o compatibility)
- [x] Forward integration of IMU (gyro → rotation, accel → velocity/position) — midpoint rule (`src/imu/propagator.h/cpp`)
- [x] Covariance propagation for the error-state — F_c/G_c Jacobians, F_d ≈ I + F_c·dt, Q_d = G_c·Q_c·G_c^T·dt (`src/imu/propagator.cpp`)
- [x] IMU buffer with time-based lookup/interpolation — linear interp at boundaries (`src/imu/buffer.h/cpp`)

---

## Phase 3: Point Cloud Preprocessing

- [x] Capture per-point timestamps — parse the `t` field (uint32 ns offset from header stamp) in PointCloud2, store `t_offset_ns` in `PointCloud` (`src/types.h`, `src/io/ros_deserializer.cpp`)
- [x] Expose lidar→imu extrinsic `T_imu_lidar` as `Sophus::SE3d` — `imu_from_lidar()` (`src/io/calibration.h/cpp`)
- [x] Build per-scan IMU trajectory — `build_scan_trajectory()` integrates gyro across `[t_start, t_end]`, stores stamped poses (`src/preprocess/deskew.h/cpp`)
- [x] Motion undistortion — `deskew()` as a pure function over a supplied trajectory; transforms each point to the scan-end frame via `T_I_L⁻¹ · T_rel · T_I_L` (`src/preprocess/deskew.cpp`)
  - Rotation-only for now (velocity seed = 0). Full 6-DOF translation term deferred until the iEKF state feeds velocity in Phase 5/6 — marked `TODO(velocity)` in `deskew.h`
- [x] Voxel downsampling — `voxel_downsample()` centroid-per-voxel over a bit-packed int64 hash grid; keeps x/y/z + averaged intensity, drops ring/t_offset_ns, skips non-finite points (`src/preprocess/voxel_grid.h/cpp`). Demo: 16384 → 4144 points at 0.5 m leaf

---

## Phase 4: Map

Incremental k-d tree (ikd-Tree, Cai et al. 2021) as the map backend, built up in
testable milestones. Test harness added with 4.1 — first tests in the project.

- [x] 4.1 Static tree + k-NN — balanced `build()` (max-spread axis, median split),
      subtree AABB + treesize attributes, recursive `knn()` with AABB pruning and a
      bounded max-heap (`src/map/ikd_tree.h/cpp`). `validate()` checks structural
      invariants. Tested against brute-force and nanoflann oracles + edge cases
      (`tests/ikd_tree_test.cpp`, 7 tests). Per-node deleted/treedeleted/pushdown/
      invalid_num fields reserved but unused. Note: compiled into `unit_tests` only;
      wires into `lidar_slam` in 4.3 when the estimator consumes it.
- [ ] 4.2 Incremental ops. API decisions: `size()` reports the **live** count
      (`treesize − invalid_num`); delete is **box-delete only** (`delete(box)`, the op
      FAST-LIO2 uses to drop far map regions — exact-point delete deferred);
      `insert` takes a **batch** (`std::vector<Vec3>`) with single-point insert
      internally. Refactor recursion to operate on `unique_ptr<Node>&` slots so a
      rebuild can swap a subtree in place. Sub-steps:
  - [ ] Attribute plumbing — extend `pull_up` to maintain `invalid_num`
        (`deleted + L.invalid_num + R.invalid_num`) and `treedeleted`
        (`deleted && L.treedeleted && R.treedeleted`); add `push_down` (lazy
        propagation of `treedeleted`/`deleted` into children before descending).
  - [ ] `insert` — recursive descend-by-axis, attach leaf, `pull_up` on unwind,
        rebalance check on unwind.
  - [ ] `delete(box)` — AABB-vs-box: fully-inside → mark `treedeleted`/`pushdown`
        (`invalid_num = treesize`); fully-outside → skip; partial → `push_down`,
        test own point, recurse, `pull_up`, rebalance check.
  - [ ] Partial rebuild (single-threaded) — on unwind test balance
        (`max(size_L, size_R) > α_bal·(size−1)`) and garbage (`invalid_num >
        α_del·size`); rebuild the topmost violating node by flattening live points
        (skip `treedeleted` subtrees) and reusing `build_range`. α_bal ≈ 0.7,
        α_del ≈ 0.5 as tunable constants.
  - [ ] `knn` update — skip `deleted` points and `treedeleted` subtrees; AABB
        pruning stays valid (box still bounds physical points, conservative).
  - [ ] `validate` update — verify `invalid_num`/`treedeleted` consistency bottom-up
        and account for pending `pushdown` without mutating (it's `const`).
  - [ ] Tests — random insert / box-delete / interleaved sequences vs a brute-force
        live set (knn matches), `validate()` after each batch; adversarial cases:
        sorted inserts force rebalance, mass-delete forces garbage-collection rebuild
        (assert the tree actually shrinks / stays balanced).
- [ ] 4.3 Point-to-plane association — kNN(5) → plane fit (solve `A·n = −1`,
      normalize; reject on neighbor-distance / residual thresholds) → correspondences
      for the iEKF.

Deferred refinements (revisit after the iEKF works):
- [ ] Parallel two-thread rebuild for large subtrees — the paper's `N_max` scheme: a
      worker rebuilds a copy while the main thread logs ops on the old subtree and
      replays them on swap. Single-threaded rebuild stalls briefly on big rebuilds;
      parallelize only if profiling shows it matters.
- [ ] Voxel-downsample-on-insert — FAST-LIO2 keeps the map at fixed resolution by
      snapping each insert to a voxel and keeping only the point nearest the voxel
      center. For now map resolution is managed by box-delete + the per-scan
      `voxel_downsample()`.

---

## Phase 5: State Estimation (iEKF)

- [ ] Error-state representation (position, velocity, rotation, IMU biases, gravity)
- [ ] Measurement model — point-to-plane residuals and Jacobians
- [ ] Iterated EKF update loop
- [ ] Outlier rejection (chi-squared test on residuals)
- [ ] Online LiDAR↔IMU extrinsic estimation (stretch) — add `[δθ_ext, δp_ext]` to the error state (18 → 24, or 23 with gravity on S²) as in FAST-LIO2 (`offset_R_L_I` / `offset_T_L_I`); random-constant dynamics, driven by the point-to-plane Jacobian. Currently `T_imu_lidar` is fixed from YAML (`imu_from_lidar()`). Watch observability — translation needs sufficient motion excitation, can drift/hurt accuracy otherwise.

---

## Phase 6: Pipeline

- [ ] Main SLAM loop tying all phases together
- [ ] Initialization (static IMU init for gravity/bias estimation)

---

## Phase 7: Evaluation

- [ ] Pose trajectory output (TUM format)
- [ ] Evaluate against NTU VIRAL ground truth (ATE / RTE metrics)
- [ ] Optional: ROS2 publisher for RViz visualization
