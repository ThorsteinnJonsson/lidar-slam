# SLAM Framework TODO

FAST-LIO2-style LiDAR-inertial SLAM. Dataset: NTU VIRAL (ROS1 bag, Ouster OS1-16 + IMU).

## Status

Runs end-to-end on `eee_03`: bag I/O → per-scan deskew → voxel downsample →
iterated error-state EKF against an incremental k-d tree (ikd-Tree) map → TUM
trajectory + evo metrics.

State is 17-DOF `[R, p, v, b_g, b_a, g]`, error `δx = [δθ, δp, δv, δb_g, δb_a, δg] ∈ R¹⁷`
(right perturbation on R, gravity on S² so it tilts but keeps `|g|` fixed).

- [x] Phase 1 — I/O: in-house ROS1 bag reader, IMU/PointCloud2 deserializers, YAML calibration
- [x] Phase 2 — IMU propagation: Sophus SO3/SE3, midpoint integration, error-state covariance, time-indexed buffer
- [x] Phase 3 — Preprocessing: per-point deskew (full IMU kinematics), voxel downsampling
- [x] Phase 4 — Map: ikd-Tree (balanced build, incremental insert/box-delete, partial rebuild, pruned k-NN)
- [x] Phase 5 — Estimator: iterated ES-EKF, point-to-plane association, Mahalanobis outlier gate, pipeline glue
- [x] Phase 6 — Pipeline: main loop, static initialization, sliding-window map crop, voxel-on-insert
- [x] Phase 7 — Evaluation: TUM output + evo ATE/RTE vs Leica prism ground truth

## Open / deferred

- [ ] Online LiDAR↔IMU extrinsic estimation (stretch) — add `[δθ_ext, δp_ext]` to the error state;
      needs motion excitation, watch observability. `T_imu_lidar` is currently fixed from YAML.
- [x] Background (parallel) ikd-Tree rebuild — subtrees ≥1500 rebuild on a worker thread, frozen
      subtree stays live, swap on finalize. Measured no benefit (rebuild was <1% of wall-clock;
      worker finishes within one scan) and a slight overhead; kept for completeness. ATE unchanged
      to float32 epsilon (deferred swap shifts k-NN tie-breaking, not the neighbor set).
- [ ] Read parameters from file
- [ ] Add support for more datasets (via command line)

## Known issues

- Map tilts ~3-4° at takeoff on `eee_03`. FAST-LIO2 tilts comparably (~3°) on the same bag, so we are
  at parity and the cause looks intrinsic to the data/method. See `experiments/fastlio2_ntuviral/`.
