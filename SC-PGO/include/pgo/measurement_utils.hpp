#pragma once

#include <cmath>
#include <limits>

namespace pgo {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

inline double wrapAngle(double angle) {
  if (!std::isfinite(angle)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double wrapped = std::fmod(angle + kPi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

inline double northClockwiseHeadingToEnuYaw(
    double heading,
    double antenna_yaw_offset,
    double meridian_convergence) {
  return wrapAngle(
      kPi / 2.0 - heading + antenna_yaw_offset + meridian_convergence);
}

inline bool validPositionCovariance(
    double covariance_xx,
    double covariance_xy,
    double covariance_yy) {
  if (!std::isfinite(covariance_xx) || !std::isfinite(covariance_xy) ||
      !std::isfinite(covariance_yy)) {
    return false;
  }
  constexpr double kMinVariance = 1e-12;
  if (covariance_xx <= kMinVariance || covariance_yy <= kMinVariance) {
    return false;
  }
  return covariance_xx * covariance_yy - covariance_xy * covariance_xy >
         kMinVariance;
}

inline double positionMahalanobisSquared(
    double residual_x,
    double residual_y,
    double covariance_xx,
    double covariance_xy,
    double covariance_yy) {
  if (!std::isfinite(residual_x) || !std::isfinite(residual_y) ||
      !validPositionCovariance(covariance_xx, covariance_xy, covariance_yy)) {
    return std::numeric_limits<double>::infinity();
  }
  const double determinant =
      covariance_xx * covariance_yy - covariance_xy * covariance_xy;
  return (covariance_yy * residual_x * residual_x -
          2.0 * covariance_xy * residual_x * residual_y +
          covariance_xx * residual_y * residual_y) /
         determinant;
}

inline bool acceptYawMeasurement(
    double estimated_yaw,
    double measured_yaw,
    double yaw_variance,
    double mahalanobis_threshold) {
  if (!std::isfinite(yaw_variance) || yaw_variance <= 0.0 ||
      !std::isfinite(mahalanobis_threshold) || mahalanobis_threshold < 0.0) {
    return false;
  }
  const double residual = wrapAngle(measured_yaw - estimated_yaw);
  return std::isfinite(residual) &&
         residual * residual / yaw_variance <= mahalanobis_threshold;
}

}  // namespace pgo
