# SLAM Framework TODO

FAST-LIO2-style LiDAR-inertial SLAM. Dataset: NTU VIRAL (ROS1 bag, Ouster OS1-16 + IMU).

## Status

Runs end-to-end on all four NTU VIRAL sequences (`eee_02`, `eee_03`, `rtp_03`, `tnp_01`):
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

- [ ] Dataset loader interface, when a second format actually lands. `--format` validates
      today but selects nothing.

## Done, with findings worth keeping

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
