#pragma once

#include <boost/none.hpp>
#include <boost/optional.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "pgo/measurement_utils.hpp"

namespace pgo {

class PositionXYFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  PositionXYFactor(
      gtsam::Key key,
      double east,
      double north,
      const gtsam::SharedNoiseModel& noise_model)
      : Base(noise_model, key), east_(east), north_(north) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    gtsam::Matrix36 translation_jacobian;
    const gtsam::Point3& translation = pose.translation(translation_jacobian);
    if (jacobian) {
      *jacobian = translation_jacobian.topRows<2>();
    }
    return (gtsam::Vector(2) << translation.x() - east_,
            translation.y() - north_)
        .finished();
  }

 private:
  double east_;
  double north_;
};

class YawFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  YawFactor(
      gtsam::Key key,
      double measured_yaw,
      const gtsam::SharedNoiseModel& noise_model)
      : Base(noise_model, key), measured_yaw_(wrapAngle(measured_yaw)) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    gtsam::Matrix36 rotation_jacobian;
    const gtsam::Rot3& rotation = pose.rotation(rotation_jacobian);
    gtsam::Matrix13 yaw_jacobian;
    const double predicted_yaw = rotation.yaw(yaw_jacobian);
    if (jacobian) {
      *jacobian = yaw_jacobian * rotation_jacobian;
    }
    return gtsam::Vector1(wrapAngle(predicted_yaw - measured_yaw_));
  }

 private:
  double measured_yaw_;
};

}  // namespace pgo
