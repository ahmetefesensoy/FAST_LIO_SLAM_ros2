#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>

#include "pgo/transforms.hpp"

namespace {

TEST(PgoTransforms, MapToOdomPreservesOptimizedKeyframePose) {
  const gtsam::Pose3 odom_T_base(
      gtsam::Rot3::RzRyRx(0.03, -0.02, 0.4),
      gtsam::Point3(8.0, -3.0, 0.5));
  const gtsam::Pose3 map_T_base(
      gtsam::Rot3::RzRyRx(0.04, -0.01, 0.7),
      gtsam::Point3(12.0, 4.0, 0.8));

  const gtsam::Pose3 map_T_odom =
      pgo::mapToOdom(map_T_base, odom_T_base);
  const gtsam::Pose3 reconstructed = map_T_odom.compose(odom_T_base);

  EXPECT_TRUE(reconstructed.equals(map_T_base, 1e-9));
}

TEST(PgoTransforms, IdenticalGlobalAndLocalPoseGivesIdentity) {
  const gtsam::Pose3 pose(
      gtsam::Rot3::RzRyRx(-0.1, 0.2, -1.0),
      gtsam::Point3(-2.0, 6.0, 1.5));

  EXPECT_TRUE(pgo::mapToOdom(pose, pose).equals(gtsam::Pose3(), 1e-9));
}

TEST(PgoTransforms, InitializesBaseFromAntennaPositionAndAbsoluteYaw) {
  const gtsam::Pose3 local_pose(
      gtsam::Rot3::RzRyRx(0.1, -0.2, -0.4),
      gtsam::Point3(2.0, 3.0, 4.0));

  const gtsam::Pose3 initialized = pgo::initializeGlobalPose(
      local_pose, gtsam::Point2(101.0, 202.0), 3.14159265358979323846 / 2.0,
      gtsam::Point3(1.0, 0.0, 0.0));

  const gtsam::Point3 reconstructed_antenna =
      initialized.transformFrom(gtsam::Point3(1.0, 0.0, 0.0));
  EXPECT_NEAR(reconstructed_antenna.x(), 101.0, 1e-9);
  EXPECT_NEAR(reconstructed_antenna.y(), 202.0, 1e-9);
  EXPECT_NEAR(initialized.translation().z(), 4.0, 1e-9);
  EXPECT_NEAR(initialized.rotation().roll(), 0.1, 1e-9);
  EXPECT_NEAR(initialized.rotation().pitch(), -0.2, 1e-9);
  EXPECT_NEAR(initialized.rotation().yaw(), 3.14159265358979323846 / 2.0, 1e-9);
}

TEST(PgoTransforms, KeepsLocalPoseWithoutAbsoluteMeasurements) {
  const gtsam::Pose3 local_pose(
      gtsam::Rot3::RzRyRx(0.1, -0.2, 0.4),
      gtsam::Point3(2.0, 3.0, 4.0));

  EXPECT_TRUE(
      pgo::initializeGlobalPose(local_pose, std::nullopt, std::nullopt)
          .equals(local_pose, 1e-9));
}

}  // namespace
