#include "estimator/outlier.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <utility>
#include <vector>

#include "estimator/measurement.h"
#include "state/state.h"

namespace {

// Assemble a LinearizedMeasurement from explicit (H row, residual) pairs.
LinearizedMeasurement make_meas(
    const std::vector<std::pair<ErrorState, double>>& rows) {
  const Eigen::Index m = static_cast<Eigen::Index>(rows.size());
  LinearizedMeasurement out;
  out.H = MeasurementJacobian(m, kErrorDim);
  out.z = Eigen::VectorXd(m);
  for (Eigen::Index i = 0; i < m; ++i) {
    out.H.row(i) = rows[static_cast<size_t>(i)].first.transpose();
    out.z(i) = rows[static_cast<size_t>(i)].second;
  }
  return out;
}

// A row sensitive to one position component (unit normal along +x), with the
// given residual.
std::pair<ErrorState, double> pos_row(double residual) {
  ErrorState h = ErrorState::Zero();
  h(3) = 1.0;  // delta-p_x
  return {h, residual};
}

}  // namespace

TEST(Outlier, KeepsCleanInliers) {
  const auto m = make_meas({pos_row(0.0), pos_row(0.001), pos_row(-0.002)});
  const ErrorMatrix P = ErrorMatrix::Identity() * 0.25;

  const auto gated = gate_measurement(m, P, /*sigma=*/0.01, /*chi2=*/3.841);
  EXPECT_EQ(gated.H.rows(), 3);
  EXPECT_EQ(gated.z.size(), 3);
}

TEST(Outlier, DropsGrossOutliers) {
  // Three inliers (zero residual) interleaved with two gross outliers.
  const auto m = make_meas(
      {pos_row(0.0), pos_row(5.0), pos_row(0.0), pos_row(-5.0), pos_row(0.0)});
  const ErrorMatrix P = ErrorMatrix::Identity() * 0.25;

  const auto gated = gate_measurement(m, P, /*sigma=*/0.01, /*chi2=*/3.841);

  ASSERT_EQ(gated.H.rows(), 3);
  // Survivors are the inliers, so every kept residual is near zero.
  for (Eigen::Index i = 0; i < gated.z.size(); ++i)
    EXPECT_LT(std::abs(gated.z(i)), 1e-6);
}

TEST(Outlier, TighterThresholdDropsSuperset) {
  std::vector<std::pair<ErrorState, double>> rows;
  for (double z : {0.0, 0.2, 0.4, 0.6, 0.8, 1.0}) rows.push_back(pos_row(z));
  const auto m = make_meas(rows);
  const ErrorMatrix P = ErrorMatrix::Identity() * 0.25;

  const auto loose = gate_measurement(m, P, 0.1, /*chi2=*/9.0);
  const auto tight = gate_measurement(m, P, 0.1, /*chi2=*/3.841);

  // A tighter gate can only keep fewer rows.
  EXPECT_LE(tight.H.rows(), loose.H.rows());
  EXPECT_LT(tight.H.rows(), m.H.rows());
}

TEST(Outlier, LargerPriorKeepsBorderlineRow) {
  // Same residual, two priors. The innovation variance H P H^T + sigma^2 grows
  // with P, so the bigger prior tolerates the residual the smaller one rejects.
  const auto m = make_meas({pos_row(0.5)});
  const double sigma = 0.01;
  const double chi2 = 3.841;

  const auto small =
      gate_measurement(m, ErrorMatrix::Identity() * 0.01, sigma, chi2);
  const auto large =
      gate_measurement(m, ErrorMatrix::Identity() * 1.0, sigma, chi2);

  EXPECT_EQ(small.H.rows(), 0);
  EXPECT_EQ(large.H.rows(), 1);
}

TEST(Outlier, EmptyInputEmptyOutput) {
  LinearizedMeasurement m;
  m.H = MeasurementJacobian(0, kErrorDim);
  m.z = Eigen::VectorXd(0);

  const auto gated = gate_measurement(m, ErrorMatrix::Identity(), 0.01, 3.841);
  EXPECT_EQ(gated.H.rows(), 0);
  EXPECT_EQ(gated.z.size(), 0);
}
