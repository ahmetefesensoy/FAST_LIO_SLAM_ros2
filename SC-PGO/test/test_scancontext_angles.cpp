#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "scancontext/angle_utils.hpp"

TEST(ScanContextAngles, CoversAxesAndQuadrants) {
  EXPECT_NEAR(scancontext::azimuthDegrees(1.0F, 0.0F), 0.0F, 1e-5F);
  EXPECT_NEAR(scancontext::azimuthDegrees(0.0F, 1.0F), 90.0F, 1e-5F);
  EXPECT_NEAR(scancontext::azimuthDegrees(-1.0F, 0.0F), 180.0F, 1e-5F);
  EXPECT_NEAR(scancontext::azimuthDegrees(0.0F, -1.0F), 270.0F, 1e-5F);
  EXPECT_NEAR(scancontext::azimuthDegrees(1.0F, -1.0F), 315.0F, 1e-5F);
}

TEST(ScanContextAngles, OriginIsDeterministic) {
  EXPECT_FLOAT_EQ(scancontext::azimuthDegrees(0.0F, 0.0F), 0.0F);
}

TEST(ScanContextAngles, NonFiniteInputFailsClosed) {
  EXPECT_TRUE(std::isnan(scancontext::azimuthDegrees(
      std::numeric_limits<float>::quiet_NaN(), 0.0F)));
}
