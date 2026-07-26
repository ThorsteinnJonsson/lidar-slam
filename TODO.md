# SLAM Framework TODO

FAST-LIO2-style LiDAR-inertial SLAM. ROS1 bags; three dataset formats
(NTU VIRAL, HILTI 2022, FAST-LIVO2) behind a `DatasetLoader` interface.

## Status

Runs end-to-end on NTU VIRAL (Ouster), HILTI 2022 (Hesai), and FAST-LIVO2 (Livox Avia):
bag I/O → per-scan deskew → voxel downsample → iterated error-state EKF against an
incremental k-d tree (ikd-Tree) map → TUM trajectory + evo metrics.

State is 23-DOF `[R, p, v, b_g, b_a, g, R_ext, p_ext]`, error
`δx = [δθ, δp, δv, δb_g, δb_a, δg, δθ_ext, δp_ext] ∈ R²³` (right perturbation on rotations,
gravity on S² so it tilts but keeps `|g|` fixed). The extrinsic blocks are only free when
`kEnableExtrinsicEstimation` is on — default **off**, pinned via its prior covariance.

- [x] Phase 1 — I/O: in-house ROS1 bag reader, IMU/PointCloud2 deserializers, YAML calibration
- [x] Phase 2 — IMU propagation: Sophus SO3/SE3, midpoint integration, error-state covariance, time-indexed buffer
- [x] Phase 3 — Preprocessing: per-point deskew (full IMU kinematics), voxel downsampling
- [x] Phase 4 — Map: ikd-Tree (balanced build, incremental insert/box-delete, partial rebuild, pruned k-NN)
- [x] Phase 5 — Estimator: iterated ES-EKF, point-to-plane association, Mahalanobis outlier gate, pipeline glue
- [x] Phase 6 — Pipeline: main loop, static initialization, sliding-window map crop, voxel-on-insert
- [x] Phase 7 — Evaluation: TUM output + evo ATE/RTE vs Leica prism ground truth

## Open

- [ ] FAST-LIVO2 has no ground truth for `HKU_Cultural_Center_01`, so no ATE. Pick a GT sequence to
      quantify accuracy.
- [ ] General refactoring pass. `main.cpp` is ~300 lines doing CLI, bag streaming, the per-scan
      pipeline, and output; `propagator.cpp` still indexes error-state blocks with literals instead
      of the `kIdx*` constants; trim comments that restate the code.

## Done, with findings worth keeping

- [x] Split into libraries: `slam_core` (algorithm, in `src/core/`), `slam_io` (dataset I/O, depends
      on core), `slam_viz` (Rerun, behind the flag). Each has its own `src/<dir>/CMakeLists.txt`; top
      level uses `add_subdirectory`. `tests/CMakeLists.txt` now links `slam_io` (brings core) instead
      of re-listing `src/*.cpp`. Core's include root is `src/core` so includes stayed unchanged;
      `types.h` moved to `src/core/`. STATIC libs.

- [x] Multi-format datasets behind a `DatasetLoader` interface (`--format NTU_VIRAL|HILTI_22|FAST_LIVO2`).
      NTU eee_03 ATE 0.112 (unchanged), HILTI exp21 ATE 0.063 (with the real extrinsic from the
      FAST-LIVO2 repo), FAST-LIVO2 runs but ships no GT. Per-format quirks handled: Hesai per-point
      time is absolute f64 seconds (not ns offset); Livox CustomMsg is a separate decoder and its IMU
      reports accel in g (x9.80665); Livox `timebase` was 160 s off the header stamp so the header is
      the scan reference; Livox sweeps can overlap, so non-advancing scans are dropped. Extrinsic in
      params takes a quaternion or a 3x3 matrix.
- [x] CLI11 arguments: `--format`, `--sequence`, `--params` (all required), `--output`.
- [x] Tunables in `config/ntu_viral.yaml` (see `config/README.md`).
- [x] Online LiDAR↔IMU extrinsic estimation (17→23 DOF), behind `enable_extrinsic_estimation`,
      default off. Costs ATE on `eee_03` (0.1119 → 0.1137 even at a 0.2°/2 mm seed) and the extrinsic
      walks to ~2-4σ of whatever seed it gets instead of converging: it absorbs registration misfit
      rather than being observed. Needs real motion excitation. 23-DOF costs ~22% wall-clock even off.
- [x] Background (parallel) ikd-Tree rebuild for subtrees ≥1500. No benefit: rebuild was <1% of
      wall-clock and the worker finishes within one scan. Kept for completeness.
- [x] Multi-dataset support. `rtp_03`/`tnp_01` ship no `imu_v100.yaml` and use `T_Body2Lidar`/
      `T_Body2Imu` key aliases; both handled. Outputs go to `evaluation/<sequence>/`.

## Known issues

- Map tilts ~3-4° at takeoff on `eee_03`. FAST-LIO2 tilts comparably (~3°) on the same bag, so we are
  at parity and the cause looks intrinsic to the data/method. See `experiments/fastlio2_ntuviral/`.
