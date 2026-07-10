# Handoff: run FAST-LIO2 on NTU VIRAL eee_03 and measure its map tilt

## Why (context, read first)

Our own LiDAR-SLAM (this repo) produces a map that tilts ~4.5 deg off horizontal
when the drone takes off in `eee_03`, and we have exhausted every identifiable
implementation/tuning difference vs FAST-LIO2. The open question this experiment
answers: **does FAST-LIO2 itself keep the map level on this exact bag, or does
it tilt too?** Published ATE numbers cannot answer this because aligned
evaluation (`evo --align`) absorbs a global tilt.

The deliverable is ONE number plus a screenshot-level visual check:

1. The **trajectory tilt vs ground truth**: run evo with `-va`, read the
   printed "Rotation of alignment" 3x3 matrix R, and report
   `tilt_deg = acos(R[2][2]) * 180 / pi`. This is the angle between the
   estimated trajectory's vertical and the ground truth's vertical.
   - tilt < ~1 deg: FAST-LIO2 stays level (our bug is genuinely ours).
   - tilt ~3-5 deg: FAST-LIO2 tilts the same way (the tilt is intrinsic to the
     data/method and our implementation is already at parity).
2. ATE RMSE (secondary, for sanity: expect roughly 0.1-0.5 m).
3. Visual: in RViz, watch the map around takeoff (t ~ 30-40 s into the bag).
   Does the ground plane stay parallel to the RViz grid afterward?

## Hard constraints

- **Everything lives in `experiments/fastlio2_ntuviral/`** (this folder).
  Do NOT touch, build, or modify anything else in the repo. Do not modify the
  dataset yamls.
- **Docker only** (host has no ROS). Use `osrf/ros:noetic-desktop-full` as the
  base image - it includes RViz. (Note: the official `ros` Docker Hub repo has
  no `desktop-full` tag, only `osrf/ros` does - confirmed 2026-07-10.)
- **Use this fork, not the upstream repo**:
  `https://github.com/brytsknguyen/FAST_LIO` - it is the NTU VIRAL authors'
  fork and ships a ready-made NTU VIRAL config/launch (look under `config/` and
  `launch/` for the ntuviral files; use them as-is, do not retune).
  FAST-LIO also requires `livox_ros_driver` as a build dependency even for
  Ouster data: clone `https://github.com/Livox-SDK/livox_ros_driver` into the
  same catkin workspace.

## Dataset facts (verified, do not rediscover)

- Bag: `datasets/ntu_viral/eee_03/eee_03.bag` (repo root relative), 4.0 GB,
  ~182 s. Mount the dataset directory read-only into the container.
- Topics in the bag:
  - horizontal lidar (the one to use): `/os1_cloud_node1/points`
    (Ouster OS1-16, 10 Hz, per-point time field "t" in ns from scan start)
  - IMU: `/imu/imu` (VectorNav VN100, ~385 Hz)
  - ground truth: `/leica/pose/relative` (geometry_msgs/PoseStamped,
    position-only prism tracking, ~20 Hz)
- The fork's ntuviral config should already have the right topics; verify they
  match the above before running.
- The platform sits still for ~30 s, then takes off around t=35 s. The takeoff
  is where the tilt (if any) gets injected.

## Suggested structure

```
experiments/fastlio2_ntuviral/
  HANDOFF.md        (this file)
  Dockerfile        (noetic-desktop-full + catkin ws + FAST_LIO fork + livox driver)
  run.sh            (build image, then: roscore + fastlio launch + rosbag play + rosbag record /Odometry)
  eval.sh           (convert recorded odometry to TUM, run evo, print tilt)
  output/           (recorded bag, TUM files, evo output - gitignore this)
```

RViz: run the container with `--net=host`, `-e DISPLAY`, and
`-v /tmp/.X11-unix:/tmp/.X11-unix`; tell the user to run `xhost +local:docker`
on the host first. FAST-LIO's launch files already open RViz with a preset
view when `rviz:=true`.

## Recording and evaluating the trajectory

- FAST-LIO publishes its pose on `/Odometry` (nav_msgs/Odometry). Record it:
  `rosbag record -O output/fastlio_odom.bag /Odometry`, and play the dataset
  with `--clock` and real-time rate (FAST-LIO is comfortably real-time).
- Convert to TUM: the host has evo in a venv at `~/.venvs/evo/bin/`:
  `~/.venvs/evo/bin/evo_traj bag output/fastlio_odom.bag /Odometry --save_as_tum`
  (this can run on the HOST, no ROS needed - evo reads bags directly).
- Ground truth in TUM format ALREADY EXISTS at `evaluation/gt.tum` (repo root),
  written by our pipeline from the leica topic. Reuse it.
- Evaluate (host):
  ```
  ~/.venvs/evo/bin/evo_ape tum evaluation/gt.tum <fastlio>.tum \
      --align -r trans_part --t_max_diff 0.03 -va
  ```
  - `--t_max_diff 0.03` is required (GT is 20 Hz; the default 0.01 drops pairs).
  - From the verbose output take "Rotation of alignment" and report
    `acos(R[2][2])` in degrees, plus the rmse line.
- Known caveat, mention in the report but do not fix: GT tracks the prism, the
  estimate tracks the IMU/body origin; the constant ~0.2 m lever arm slightly
  inflates ATE but has negligible effect on the alignment-rotation tilt (the
  number we care about).

## Report back

- The tilt angle (deg), the ATE RMSE, and whether RViz visually showed the
  ground plane staying level through takeoff.
- Any config values the ntuviral launch used that differ from FAST-LIO
  defaults (especially `time_offset_lidar_to_imu` or equivalent, extrinsics,
  and noise/cov settings) - list them verbatim, they are valuable to us.
- If the numbers reproduce across two runs (FAST-LIO has some nondeterminism),
  say so; a single run is acceptable if time is short.
