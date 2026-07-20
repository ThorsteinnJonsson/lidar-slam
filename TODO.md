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

- [ ] Read parameters from file — pull the hard-coded constants out of `main.cpp` (noise, time offset,
      voxel leaf, iEKF sigma, `kSequence`, `kEnableExtrinsicEstimation`).
- [ ] Promote `kSequence` to a command-line argument (pairs naturally with the config-file work).

## Done, with findings worth keeping

- [x] Online LiDAR↔IMU extrinsic estimation — `[δθ_ext, δp_ext]` added to the error state (17→23 DOF),
      behind `kEnableExtrinsicEstimation` (default off, matching FAST-LIO2's `extrinsic_est_en: false`
      for NTU VIRAL). Measured on `eee_03`: enabling it costs ATE (0.1119 → 0.1205 with a 1°/1 cm seed,
      0.1137 even at 0.2°/2 mm). The extrinsic walks out to ~2-4σ of *whatever* seed it is given rather
      than converging on a fixed value — it absorbs registration misfit instead of being observed
      (δp_ext and δp shift the world point identically within a frame; only motion separates them).
      Revisit on a sequence with real excitation. Note the 23-DOF filter costs ~22% wall-clock even
      when the flag is off.
- [x] Background (parallel) ikd-Tree rebuild — subtrees ≥1500 rebuild on a worker thread, frozen
      subtree stays live, swap on finalize. Measured no benefit (rebuild was <1% of wall-clock;
      worker finishes within one scan) and a slight overhead; kept for completeness. ATE unchanged
      to float32 epsilon (deferred swap shifts k-NN tie-breaking, not the neighbor set).
- [x] Multi-dataset support — `kSequence` selects the sequence; missing `imu_v100.yaml` falls back to
      the NTU VIRAL-wide defaults, and the loader accepts the `T_Body2Lidar`/`T_Body2Imu` key aliases
      that `rtp_03`/`tnp_01` use. Outputs go to `evaluation/<sequence>/`.

## Known issues

- Map tilts ~3-4° at takeoff on `eee_03`. FAST-LIO2 tilts comparably (~3°) on the same bag, so we are
  at parity and the cause looks intrinsic to the data/method. See `experiments/fastlio2_ntuviral/`.
