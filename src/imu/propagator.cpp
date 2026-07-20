#include "imu/propagator.h"

#include <sophus/so3.hpp>

ImuPropagator::ImuPropagator(const NoiseParams& noise) : noise_(noise) {}

// ── midpoint integration
// ──────────────────────────────────────────────────────

State ImuPropagator::propagate(const State& s0, const ImuMeasurement& m0,
                               const ImuMeasurement& m1) const {
  const double dt =
      static_cast<double>(m1.stamp.to_nsec() - m0.stamp.to_nsec()) * 1e-9;

  // Bias-corrected measurements.
  const Eigen::Vector3d w0 = m0.angular_velocity - s0.b_g;
  const Eigen::Vector3d w1 = m1.angular_velocity - s0.b_g;
  const Eigen::Vector3d a0 = m0.linear_acceleration - s0.b_a;
  const Eigen::Vector3d a1 = m1.linear_acceleration - s0.b_a;

  // Midpoint quantities.
  const Eigen::Vector3d w_mid = 0.5 * (w0 + w1);
  const Eigen::Vector3d a_mid = 0.5 * (a0 + a1);

  // Rotation at midpoint (for transforming accel to world frame).
  const Sophus::SO3d R_mid = s0.R * Sophus::SO3d::exp(0.5 * w_mid * dt);

  // Integrate.
  State s1;
  s1.R = s0.R * Sophus::SO3d::exp(w_mid * dt);
  s1.gravity = s0.gravity;
  s1.b_g = s0.b_g;
  s1.b_a = s0.b_a;
  // Extrinsic is a random constant: carried through prediction unchanged.
  s1.R_imu_lidar = s0.R_imu_lidar;
  s1.p_imu_lidar = s0.p_imu_lidar;

  const Eigen::Vector3d a_world = R_mid * a_mid + s0.gravity;
  s1.v = s0.v + a_world * dt;
  s1.p = s0.p + s0.v * dt + 0.5 * a_world * dt * dt;

  return s1;
}

// ── covariance propagation
// ────────────────────────────────────────────────────

// Error-state ordering: [δθ (0-2), δp (3-5), δv (6-8), δb_g (9-11), δb_a
// (12-14), δg (15-16), δθ_ext (17-19), δp_ext (20-22)]. δg is 2-DOF: gravity
// tilts on S² (see state/s2.h). The extrinsic blocks are random constants:
// zero dynamics and zero process noise, so they move only through the
// point-to-plane measurement.
//
// Continuous-time Jacobian F_c (23×23):
//   δθ̇  = -[ω̂]× δθ  - δb_g
//   δṗ  =  δv
//   δv̇  = -R[â]× δθ  - R δb_a - [g]× B δg   (B = S² tangent basis at g)
//   δḃ_g = 0
//   δḃ_a = 0
//   δġ   = 0
//   δθ̇_ext = 0
//   δṗ_ext = 0
//
// Noise input n = [n_g, n_a, n_bg, n_ba] ∈ R¹²:
//   G_c row δθ:   [-I,  0, 0, 0]
//   G_c row δp:   [ 0,  0, 0, 0]
//   G_c row δv:   [ 0, -R, 0, 0]
//   G_c row δb_g: [ 0,  0, I, 0]
//   G_c row δb_a: [ 0,  0, 0, I]
//   G_c row δg:   [ 0,  0, 0, 0]

std::pair<State, ErrorMatrix> ImuPropagator::propagate_with_covariance(
    const State& s0, const ErrorMatrix& P, const ImuMeasurement& m0,
    const ImuMeasurement& m1) const {
  const double dt =
      static_cast<double>(m1.stamp.to_nsec() - m0.stamp.to_nsec()) * 1e-9;

  // Bias-corrected midpoint quantities (same as propagate()).
  const Eigen::Vector3d w0 = m0.angular_velocity - s0.b_g;
  const Eigen::Vector3d w1 = m1.angular_velocity - s0.b_g;
  const Eigen::Vector3d a0 = m0.linear_acceleration - s0.b_a;
  const Eigen::Vector3d a1 = m1.linear_acceleration - s0.b_a;
  const Eigen::Vector3d w_mid = 0.5 * (w0 + w1);
  const Eigen::Vector3d a_mid = 0.5 * (a0 + a1);
  const Sophus::SO3d R_mid = s0.R * Sophus::SO3d::exp(0.5 * w_mid * dt);

  // Propagate nominal state.
  State s1;
  s1.R = s0.R * Sophus::SO3d::exp(w_mid * dt);
  s1.gravity = s0.gravity;
  s1.b_g = s0.b_g;
  s1.b_a = s0.b_a;
  // Extrinsic is a random constant: carried through prediction unchanged.
  s1.R_imu_lidar = s0.R_imu_lidar;
  s1.p_imu_lidar = s0.p_imu_lidar;
  const Eigen::Vector3d a_world = R_mid * a_mid + s0.gravity;
  s1.v = s0.v + a_world * dt;
  s1.p = s0.p + s0.v * dt + 0.5 * a_world * dt * dt;

  // ── Build F_c ──────────────────────────────────────────────────────────────
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d R_mat = s0.R.matrix();

  ErrorMatrix Fc = ErrorMatrix::Zero();

  // δθ row
  Fc.block<3, 3>(0, 0) = -skew(w_mid);  // -[ω̂]×
  Fc.block<3, 3>(0, 9) = -I3;           // -δb_g

  // δp row
  Fc.block<3, 3>(3, 6) = I3;  // δv

  // δv row
  Fc.block<3, 3>(6, 0) = -R_mat * skew(a_mid);  // -R[â]×
  Fc.block<3, 3>(6, 12) = -R_mat;               // -R δb_a
  // ∂(a_world)/∂δg for the 2-DOF S² tilt: a rotation Exp(B·δg) moves g by
  // -[g]× B·δg, so the δv column is -[g]× B (3×2).
  Fc.block<3, 2>(6, 15) = -skew(s0.gravity) * s2_tangent_basis(s0.gravity);

  // ── Discrete Jacobian: F_d ≈ I + F_c·dt (first-order) ────────────────────
  const ErrorMatrix Fd = ErrorMatrix::Identity() + Fc * dt;

  // ── Build additive noise covariance Q_d = G_c·Q_c·G_c^T·dt ──────────────
  // G_c·Q_c·G_c^T is block-diagonal with blocks:
  //   σ_g²·I   (δθ)
  //   0        (δp)
  //   σ_a²·I   (δv,  since R*I*R^T = I)
  //   σ_bg²·I  (δb_g)
  //   σ_ba²·I  (δb_a)
  //   0        (δg)
  const double sg2 = noise_.gyro_noise_std * noise_.gyro_noise_std;
  const double sa2 = noise_.accel_noise_std * noise_.accel_noise_std;
  const double sbg2 = noise_.gyro_rw_std * noise_.gyro_rw_std;
  const double sba2 = noise_.accel_rw_std * noise_.accel_rw_std;

  ErrorMatrix Qd = ErrorMatrix::Zero();
  Qd.block<3, 3>(0, 0) = sg2 * dt * I3;
  Qd.block<3, 3>(6, 6) = sa2 * dt * I3;
  Qd.block<3, 3>(9, 9) = sbg2 * dt * I3;
  Qd.block<3, 3>(12, 12) = sba2 * dt * I3;

  // ── Propagate covariance ───────────────────────────────────────────────────
  const ErrorMatrix P1 = Fd * P * Fd.transpose() + Qd;

  return {s1, P1};
}
