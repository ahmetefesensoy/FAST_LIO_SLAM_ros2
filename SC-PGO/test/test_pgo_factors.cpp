#include <cmath>

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "pgo/factors.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(PgoFactors, PositionFactorConstrainsOnlyEastAndNorth) {
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(2) << 1.0, 2.0).finished());
  const pgo::PositionXYFactor factor(gtsam::Symbol('x', 0), 8.0, 17.0, noise);
  const gtsam::Pose3 pose(
      gtsam::Rot3::RzRyRx(0.2, -0.1, 0.3), gtsam::Point3(10.0, 20.0, 5.0));

  gtsam::Matrix jacobian;
  const gtsam::Vector error = factor.evaluateError(pose, jacobian);

  ASSERT_EQ(error.size(), 2);
  EXPECT_NEAR(error(0), 2.0, 1e-12);
  EXPECT_NEAR(error(1), 3.0, 1e-12);
  ASSERT_EQ(jacobian.rows(), 2);
  ASSERT_EQ(jacobian.cols(), 6);
}

TEST(PgoFactors, YawFactorWrapsAcrossPiBoundary) {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  const pgo::YawFactor factor(gtsam::Symbol('x', 0), -kPi + 0.01, noise);
  const gtsam::Pose3 pose(
      gtsam::Rot3::RzRyRx(0.0, 0.0, kPi - 0.01), gtsam::Point3(4.0, 5.0, 6.0));

  gtsam::Matrix jacobian;
  const gtsam::Vector error = factor.evaluateError(pose, jacobian);

  ASSERT_EQ(error.size(), 1);
  EXPECT_NEAR(error(0), -0.02, 1e-9);
  ASSERT_EQ(jacobian.rows(), 1);
  ASSERT_EQ(jacobian.cols(), 6);
  EXPECT_NEAR((jacobian.block<1, 3>(0, 3).norm()), 0.0, 1e-12);
}

TEST(PgoFactors, GnssAndHeadingCorrectADriftedPose) {
  const gtsam::Key key = gtsam::Symbol('x', 0);
  gtsam::NonlinearFactorGraph graph;
  const auto loose_prior = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.1, 0.1, 10.0, 10.0, 10.0, 0.1).finished());
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      key, gtsam::Pose3(), loose_prior));
  graph.add(pgo::PositionXYFactor(
      key, 5.0, -2.0,
      gtsam::noiseModel::Isotropic::Sigma(2, 0.05)));
  graph.add(pgo::YawFactor(
      key, 0.7,
      gtsam::noiseModel::Isotropic::Sigma(1, 0.01)));

  gtsam::Values initial;
  initial.insert(
      key,
      gtsam::Pose3(
          gtsam::Rot3::RzRyRx(0.0, 0.0, -0.5),
          gtsam::Point3(1.0, 1.0, 0.0)));

  const gtsam::Values result =
      gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  const auto optimized = result.at<gtsam::Pose3>(key);

  EXPECT_NEAR(optimized.translation().x(), 5.0, 0.01);
  EXPECT_NEAR(optimized.translation().y(), -2.0, 0.01);
  EXPECT_NEAR(optimized.translation().z(), 0.0, 1e-6);
  EXPECT_NEAR(optimized.rotation().yaw(), 0.7, 0.01);
}

}  // namespace
