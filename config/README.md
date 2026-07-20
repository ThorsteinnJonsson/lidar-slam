# Configuration

`params.yaml` holds the tunable runtime parameters. It is read at startup from
the hard-coded path `config/params.yaml`, relative to the working directory, so
run the binary from the repository root.

Every key is optional: the loader starts from the defaults compiled into the
parameter structs and overwrites only the keys present in the file.

Not configured here:

- **Dataset selection** (root directory and sequence) is hard-coded in
  `main.cpp` for now and becomes a command-line argument later.
- **Sensor calibration** (`imu_v100.yaml`, `lidar_horz.yaml`,
  `leica_prism.yaml`) ships with each dataset and is loaded from there.
- **ikd-Tree internals** (rebuild balance/garbage thresholds) are structural
  constants of the data structure, not experiment knobs, and stay in
  `src/map/ikd_tree.cpp`.

---

## Top level

| Key                     | Default | Meaning                                                                                                                                                                                                                                                                                                                                                                                 |
| ----------------------- | ------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `lidar_time_offset_sec` | `-0.1`  | Lidar-to-IMU clock offset. The OS1 stamps points ahead of the IMU clock on NTU VIRAL; the corrected stamp is `t_lidar + offset`. Applied to the whole scan, so the scan window, deskew trajectory, and IMU window all shift together. Arguably per-dataset: `-0.05` measured better on `eee_03` (ATE 0.095 vs 0.112) but `-0.1` is kept for parity with the FAST-LIO2 NTU VIRAL config. |

## `preprocess`

| Key               | Default | Meaning                                                                                                                                                                                                                     |
| ----------------- | ------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `scan_voxel_leaf` | `0.5`   | Voxel leaf size (m) for downsampling each incoming scan before registration. Larger means fewer points and a faster update, at the cost of detail. **Distinct from `map.voxel_leaf`** below, which controls map resolution. |

## `init`

Static initialization: the pipeline slides a window of IMU samples until a
stationary stretch passes the variance gate, then seeds the filter from it.

| Key                 | Default | Meaning                                                                                                                                                |
| ------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `imu_count`         | `200`   | IMU samples in the initialization window.                                                                                                              |
| `gravity_magnitude` | `9.78`  | Local gravity magnitude (m/s²); 9.78 is Singapore, where NTU VIRAL was recorded. Pinning `\|g\|` keeps gravity and accel bias distinguishable at rest. |
| `max_accel_var`     | `0.05`  | Per-axis accelerometer variance gate. Above this the window is judged to be moving and is slid forward.                                                |
| `max_gyro_var`      | `0.01`  | Per-axis gyroscope variance gate, same role.                                                                                                           |

Initial covariance seed, given as per-block standard deviations:

| Key              | Default  | Meaning                                                                                                                                                                                  |
| ---------------- | -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `theta_std`      | `0.017`  | Attitude (rad, ~1°). Roll/pitch come from gravity; yaw is a gauge freedom fixed to zero.                                                                                                 |
| `pos_std`        | `0.001`  | Position (m). A gauge freedom fixed at the origin, hence tight.                                                                                                                          |
| `vel_std`        | `0.01`   | Velocity (m/s), known to be ~0 at rest.                                                                                                                                                  |
| `bias_gyro_std`  | `0.001`  | Gyro bias (rad/s), well observed while stationary.                                                                                                                                       |
| `bias_accel_std` | `0.01`   | Accel bias (m/s²). Deliberately tight: accel bias is unobservable at rest, and a loose seed lets the filter dump the motion-onset registration misfit into it, which then tilts gravity. |
| `gravity_std`    | `0.01`   | Gravity tilt (rad). Gravity lives on S², so this is a tilt allowance, not a magnitude one.                                                                                               |
| `ext_rot_std`    | `0.0035` | LiDAR-IMU extrinsic rotation (rad, ~0.2°). Only meaningful when `estimator.enable_extrinsic_estimation` is true.                                                                         |
| `ext_trans_std`  | `0.002`  | LiDAR-IMU extrinsic translation (m, 2 mm), same condition.                                                                                                                               |

## `imu_noise`

Continuous-time IMU process noise driving the covariance prediction, as standard
deviations (the FAST-LIO2 config states these as variances; these are their
square roots).

| Key               | Default  | Meaning                                 |
| ----------------- | -------- | --------------------------------------- |
| `gyro_noise_std`  | `0.3162` | Gyro white noise, `sqrt(0.1)`.          |
| `accel_noise_std` | `0.3162` | Accelerometer white noise, `sqrt(0.1)`. |
| `gyro_rw_std`     | `0.01`   | Gyro bias random walk, `sqrt(1e-4)`.    |
| `accel_rw_std`    | `0.01`   | Accel bias random walk, `sqrt(1e-4)`.   |

These are deliberately looser than the datasheet values in `imu_v100.yaml`.
A tight `Q` makes the filter overconfident: the takeoff registration misfit gets
absorbed as attitude/gravity error and locks in as a permanent map tilt. Loose
`Q` keeps the filter plastic so the transient heals (measured on `eee_03`:
final gravity tilt 5.2° → 3.6°, ATE 0.32 → 0.11). A 2×2 bisect showed the
gyro/accel white noise carries the entire improvement and the bias random walk
is irrelevant.

## `iekf`

Iterated error-state EKF measurement update.

| Key               | Default  | Meaning                                                                                                                                                                                                                               |
| ----------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sigma`           | `0.0316` | Point-to-plane measurement standard deviation (m); `R = sigma² I`. Matches FAST-LIO2's `LASER_POINT_COV = 1e-3`, i.e. ~10× less information per point than a 0.01 default.                                                            |
| `max_iterations`  | `5`      | Cap on the re-linearization loop.                                                                                                                                                                                                     |
| `convergence_tol` | `0.001`  | Stop once `\|\|dx\|\|` falls below this.                                                                                                                                                                                              |
| `reject_outliers` | `true`   | Enable the Mahalanobis chi-squared correspondence gate.                                                                                                                                                                               |
| `outlier_chi2`    | `3.841`  | Gate threshold: χ²(1) at the 95th percentile. Rows with `z²/(H P Hᵀ + σ²)` above this are dropped. Normalizing by the full innovation variance loosens the gate when the prior is uncertain and tightens it as the estimate firms up. |

## `association`

Point-to-plane correspondence search against the map.

| Key                  | Default | Meaning                                                                                                                                                                                |
| -------------------- | ------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `num_neighbors`      | `5`     | Neighbors fetched from the map to fit each plane.                                                                                                                                      |
| `max_neighbor_dist2` | `5.0`   | Reject the correspondence if the farthest neighbor is beyond this **squared** distance (m²).                                                                                           |
| `max_plane_dist`     | `0.1`   | Planarity gate: reject if any neighbor sits farther than this (m) from the fitted plane.                                                                                               |
| `min_quality`        | `0.9`   | FAST-LIO2 range-normalized gate. Keep a point when `s = 1 - 0.9·\|dist\|/sqrt(range)` exceeds this, tolerating proportionally larger residuals at long range where the fit is noisier. |

## `map`

Local map (ikd-Tree) growth and bounding.

| Key               | Default | Meaning                                                                                                                                      |
| ----------------- | ------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| `voxel_on_insert` | `true`  | Keep a registered point only when no map point already lies within `voxel_leaf`, so re-observing a surface does not pile up near-duplicates. |
| `voxel_leaf`      | `0.1`   | Map resolution (m): minimum spacing between map points. **Distinct from `preprocess.scan_voxel_leaf`.**                                      |

### `map.crop`

Sliding-window map bound (FAST-LIO2 "map sliding"). The map is held to a cube
around the sensor; when the sensor comes within a margin of a face, the cube
recenters and the outer slabs are box-deleted.

| Key             | Default | Meaning                                                                        |
| --------------- | ------- | ------------------------------------------------------------------------------ |
| `enabled`       | `true`  | Enable cropping. A no-op on short sequences that never approach the cube edge. |
| `sensor_range`  | `300.0` | Lidar detection range (m); the other two are factors of it.                    |
| `margin_factor` | `1.5`   | Slide once within `margin_factor × sensor_range` of a face.                    |
| `keep_factor`   | `2.0`   | Cube half-side is `keep_factor × sensor_range`.                                |

## `estimator`

| Key                           | Default | Meaning                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ----------------------------- | ------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `enable_extrinsic_estimation` | `false` | Estimate the LiDAR-IMU extrinsic online (adds `[δθ_ext, δp_ext]` to the filtered state). Off by default, matching FAST-LIO2's `extrinsic_est_en: false` for NTU VIRAL. Measured on `eee_03`, enabling it costs ATE (0.1119 → 0.1137 even with a tight seed) and the extrinsic walks out to ~2-4σ of whatever seed it is given rather than converging, meaning it absorbs registration misfit rather than being observed. Within a single frame `δp_ext` and `δp` shift the world point identically, so only real motion excitation separates them. When off, the extrinsic is pinned via its prior covariance and stays exactly at the calibration value. |
