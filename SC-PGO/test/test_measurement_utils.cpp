#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "pgo/measurement_utils.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(MeasurementUtils, WrapAngleUsesHalfOpenRange) {
  EXPECT_NEAR(pgo::wrapAngle(kPi), -kPi, 1e-12);
  EXPECT_NEAR(pgo::wrapAngle(-kPi), -kPi, 1e-12);
  EXPECT_NEAR(pgo::wrapAngle(3.0 * kPi), -kPi, 1e-12);
}

TEST(MeasurementUtils, ConvertsNorthClockwiseHeadingToEnuYaw) {
  EXPECT_NEAR(pgo::northClockwiseHeadingToEnuYaw(0.0, 0.0, 0.0), kPi / 2.0, 1e-12);
  EXPECT_NEAR(pgo::northClockwiseHeadingToEnuYaw(kPi / 2.0, 0.0, 0.0), 0.0, 1e-12);
  EXPECT_NEAR(
      pgo::northClockwiseHeadingToEnuYaw(0.0, 0.1, -0.2),
      kPi / 2.0 - 0.1,
      1e-12);
}

TEST(MeasurementUtils, RejectsInvalidPositionCovariance) {
  EXPECT_FALSE(pgo::validPositionCovariance(0.0, 0.0, 0.0));
  EXPECT_FALSE(pgo::validPositionCovariance(1.0, 2.0, 1.0));
  EXPECT_FALSE(pgo::validPositionCovariance(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0));
  EXPECT_TRUE(pgo::validPositionCovariance(4.0, 0.5, 9.0));
}

TEST(MeasurementUtils, ComputesCorrelatedPositionMahalanobisDistance) {
  EXPECT_NEAR(pgo::positionMahalanobisSquared(2.0, 3.0, 4.0, 0.0, 9.0), 2.0, 1e-12);
  EXPECT_TRUE(std::isinf(
      pgo::positionMahalanobisSquared(1.0, 1.0, 1.0, 1.0, 1.0)));
}

TEST(MeasurementUtils, GatesYawAcrossPiBoundary) {
  const double estimate = kPi - 0.01;
  const double measurement = -kPi + 0.01;
  EXPECT_TRUE(pgo::acceptYawMeasurement(estimate, measurement, 0.01, 9.0));
  EXPECT_FALSE(pgo::acceptYawMeasurement(estimate, measurement, 1e-6, 9.0));
}

}  // namespace
