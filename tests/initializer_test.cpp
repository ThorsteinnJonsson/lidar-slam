#include "imu/initializer.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <vector>

namespace {

constexpr double kG = 9.78;

ImuMeasurement sample(const Eigen::Vector3d& accel,
                      const Eigen::Vector3d& gyro) {
  ImuMeasurement m;
  m.linear_acceleration = accel;
  m.angular_velocity = gyro;
  return m;
}

// A window of identical static samples for a platform whose body-to-world
// rotation is R_wb, with the given constant gyro bias. At rest the
// accelerometer senses the specific force f = R_wb^T * (0,0,g).
std::vector<ImuMeasurement> static_window(const Sophus::SO3d& R_wb,
                                          const Eigen::Vector3d& gyro_bias,
                                          size_t n = 200) {
  const Eigen::Vector3d up_world(0.0, 0.0, kG);
  const Eigen::Vector3d f = R_wb.inverse() * up_world;
  return std::vector<ImuMeasurement>(n, sample(f, gyro_bias));
}

double rot_err(const Sophus::SO3d& a, const Sophus::SO3d& b) {
  return (a.inverse() * b).log().norm();
}

}  // namespace

TEST(Initializer, LevelStaticRecoversIdentityAndGravity) {
  const Eigen::Vector3d bias(0.01, -0.02, 0.005);
  const auto imu = static_window(Sophus::SO3d{}, bias);

  const InitResult r = initialize_static(imu, InitParams{});

  ASSERT_TRUE(r.ok);
  EXPECT_LT(rot_err(r.state.R, Sophus::SO3d{}), 1e-9);
  EXPECT_NEAR(r.state.gravity.z(), -kG, 1e-9);
  EXPECT_LT((r.state.gravity - Eigen::Vector3d(0, 0, -kG)).norm(), 1e-9);
  EXPECT_LT((r.state.b_g - bias).norm(), 1e-9);
  EXPECT_LT(r.state.b_a.norm(), 1e-12);
  EXPECT_LT(r.state.v.norm(), 1e-12);
  EXPECT_LT(r.state.p.norm(), 1e-12);
}

TEST(Initializer, TiltedStaticAlignsToGravity) {
  // Pure tilt about the x-axis (a horizontal axis), so the minimal-arc
  // alignment the initializer computes should recover it exactly.
  const Sophus::SO3d R_true =
      Sophus::SO3d::exp(Eigen::Vector3d(10.0 * M_PI / 180.0, 0.0, 0.0));
  const auto imu = static_window(R_true, Eigen::Vector3d::Zero());

  const InitResult r = initialize_static(imu, InitParams{});

  ASSERT_TRUE(r.ok);
  // Measured "up" rotated into the world must land on world up.
  const Eigen::Vector3d f_hat = imu.front().linear_acceleration.normalized();
  const Eigen::Vector3d up_recovered = r.state.R * f_hat;
  EXPECT_LT((up_recovered - Eigen::Vector3d::UnitZ()).norm(), 1e-9);
  EXPECT_LT(rot_err(r.state.R, R_true), 1e-9);
}

TEST(Initializer, MovingWindowRejected) {
  // Accel alternates by +-1 m/s^2 on z: variance ~1, well over the gate.
  std::vector<ImuMeasurement> imu;
  for (size_t i = 0; i < 200; ++i) {
    const double jitter = (i % 2 == 0) ? 1.0 : -1.0;
    imu.push_back(
        sample(Eigen::Vector3d(0, 0, kG + jitter), Eigen::Vector3d::Zero()));
  }

  const InitResult r = initialize_static(imu, InitParams{});

  EXPECT_FALSE(r.ok);
  EXPECT_GT(r.max_accel_var, InitParams{}.max_accel_var);
}

TEST(Initializer, GyroBiasRecovered) {
  const Eigen::Vector3d bias(0.03, 0.01, -0.02);
  const auto imu = static_window(Sophus::SO3d{}, bias);

  const InitResult r = initialize_static(imu, InitParams{});

  ASSERT_TRUE(r.ok);
  EXPECT_LT((r.state.b_g - bias).norm(), 1e-9);
}

TEST(Initializer, CovarianceSeedStructure) {
  const auto imu = static_window(Sophus::SO3d{}, Eigen::Vector3d::Zero());
  const InitResult r = initialize_static(imu, InitParams{});
  ASSERT_TRUE(r.ok);

  // Symmetric, PSD diagonal, every entry positive.
  EXPECT_LT((r.cov - r.cov.transpose()).norm(), 1e-12);
  for (int i = 0; i < 17; ++i) EXPECT_GT(r.cov(i, i), 0.0);

  // Accel bias is the one loose block: its variance dwarfs attitude/position.
  const double ba_var = r.cov(12, 12);
  EXPECT_GT(ba_var, r.cov(0, 0));    // vs attitude
  EXPECT_GT(ba_var, r.cov(3, 3));    // vs position
  EXPECT_GT(ba_var, r.cov(15, 15));  // vs gravity
}

TEST(Initializer, EmptyWindowNotOk) {
  const InitResult r = initialize_static({}, InitParams{});
  EXPECT_FALSE(r.ok);
}
