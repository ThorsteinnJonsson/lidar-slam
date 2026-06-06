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
- [ ] Forward integration of IMU (gyro → rotation, accel → velocity/position)
- [ ] Covariance propagation for the error-state
- [ ] IMU buffer with time-based lookup/interpolation

---

## Phase 3: Point Cloud Preprocessing

- [ ] Motion undistortion — deskew each point using IMU-propagated poses
- [ ] Voxel downsampling

---

## Phase 4: Map

- [ ] Incremental KD-Tree (ikd-Tree) for nearest-neighbor search and dynamic updates
- [ ] Point-to-plane association (find k-nearest neighbors, fit local plane)

---

## Phase 5: State Estimation (iEKF)

- [ ] Error-state representation (position, velocity, rotation, IMU biases, gravity)
- [ ] Measurement model — point-to-plane residuals and Jacobians
- [ ] Iterated EKF update loop
- [ ] Outlier rejection (chi-squared test on residuals)

---

## Phase 6: Pipeline

- [ ] Main SLAM loop tying all phases together
- [ ] Initialization (static IMU init for gravity/bias estimation)

---

## Phase 7: Evaluation

- [ ] Pose trajectory output (TUM format)
- [ ] Evaluate against NTU VIRAL ground truth (ATE / RTE metrics)
- [ ] Optional: ROS2 publisher for RViz visualization
