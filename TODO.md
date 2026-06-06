# SLAM Framework TODO

Roughly following FAST-LIO2. Dataset: NTU VIRAL (ROS1 bag, Ouster OS1-16 LiDAR + IMU).

---

## Phase 1: Foundation & I/O

- [x] Add Eigen, yaml-cpp, spdlog, nanoflann as dependencies (CMake FetchContent)
- [ ] Core data types: `ImuMeasurement`, `PointCloud`, `State`
- [ ] ROS1 bag reader — write minimal parser in-house (no standalone library available; ROS bag V2.0 format)
- [ ] Parse IMU messages (`/imu/imu`) and LiDAR point clouds (`/os1_cloud_node1/points`)
- [ ] Load extrinsic calibration from YAML (`T_Body_Lidar`, `T_Body_Imu`) — note: files use `!!opencv-matrix` tag, needs custom handling
- [ ] Smoke test: print IMU/point cloud counts from `eee_03.bag`

---

## Phase 2: Math Utilities

- [ ] SO3 / SE3 Lie group operations (exp, log, hat, vee, adjoint)

---

## Phase 3: IMU Propagation

- [ ] Forward integration of IMU (gyro → rotation, accel → velocity/position)
- [ ] Covariance propagation for the error-state
- [ ] IMU buffer with time-based lookup/interpolation

---

## Phase 4: Point Cloud Preprocessing

- [ ] Motion undistortion — deskew each point using IMU-propagated poses
- [ ] Voxel downsampling

---

## Phase 5: Map

- [ ] Incremental KD-Tree (ikd-Tree) for nearest-neighbor search and dynamic updates
- [ ] Point-to-plane association (find k-nearest neighbors, fit local plane)

---

## Phase 6: State Estimation (iEKF)

- [ ] Error-state representation (position, velocity, rotation, IMU biases, gravity)
- [ ] Measurement model — point-to-plane residuals and Jacobians
- [ ] Iterated EKF update loop
- [ ] Outlier rejection (chi-squared test on residuals)

---

## Phase 7: Pipeline

- [ ] Main SLAM loop tying all phases together
- [ ] Initialization (static IMU init for gravity/bias estimation)

---

## Phase 8: Evaluation

- [ ] Pose trajectory output (TUM format)
- [ ] Evaluate against NTU VIRAL ground truth (ATE / RTE metrics)
- [ ] Optional: ROS2 publisher for RViz visualization
