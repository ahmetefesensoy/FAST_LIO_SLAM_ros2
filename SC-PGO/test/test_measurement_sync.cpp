#include <gtest/gtest.h>

#include <vector>

#include "pgo/measurement_sync.hpp"

TEST(MeasurementSync, SelectsClosestCandidateInsideAsymmetricWindow) {
  const std::vector<double> stamps{9.4, 9.82, 10.01, 10.08};

  const auto result = pgo::selectMeasurement(stamps, 10.0, 0.25, 0.03);

  ASSERT_TRUE(result.index.has_value());
  EXPECT_EQ(*result.index, 2U);
  EXPECT_EQ(result.stale_prefix, 1U);
  EXPECT_NEAR(result.skew_s, 0.01, 1e-12);
  EXPECT_FALSE(result.future_only);
}

TEST(MeasurementSync, PrefersPastMeasurementWhenAbsoluteSkewTies) {
  const std::vector<double> stamps{9.98, 10.02};

  const auto result = pgo::selectMeasurement(stamps, 10.0, 0.05, 0.05);

  ASSERT_TRUE(result.index.has_value());
  EXPECT_EQ(*result.index, 0U);
  EXPECT_NEAR(result.skew_s, -0.02, 1e-12);
}

TEST(MeasurementSync, ReportsStalePrefixWithoutSelectingExpiredData) {
  const std::vector<double> stamps{8.0, 9.0};

  const auto result = pgo::selectMeasurement(stamps, 10.0, 0.2, 0.02);

  EXPECT_FALSE(result.index.has_value());
  EXPECT_EQ(result.stale_prefix, 2U);
  EXPECT_FALSE(result.future_only);
}

TEST(MeasurementSync, LeavesTooNewMeasurementForNextKeyframe) {
  const std::vector<double> stamps{10.08, 10.12};

  const auto result = pgo::selectMeasurement(stamps, 10.0, 0.2, 0.02);

  EXPECT_FALSE(result.index.has_value());
  EXPECT_EQ(result.stale_prefix, 0U);
  EXPECT_TRUE(result.future_only);
}

TEST(MeasurementSync, InvalidLimitsFailClosed) {
  const std::vector<double> stamps{10.0};

  EXPECT_FALSE(pgo::selectMeasurement(stamps, 10.0, -0.1, 0.1).index.has_value());
  EXPECT_FALSE(pgo::selectMeasurement(stamps, 10.0, 0.1, -0.1).index.has_value());
}
