#pragma once

#include <optional>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>

namespace pgo {

// map_T_base = map_T_odom * odom_T_base
inline gtsam::Pose3 mapToOdom(
    const gtsam::Pose3& map_T_base,
    const gtsam::Pose3& odom_T_base) {
  return map_T_base.compose(odom_T_base.inverse());
}

inline gtsam::Pose3 initializeGlobalPose(
    const gtsam::Pose3& local_pose,
    const std::optional<gtsam::Point2>& antenna_xy,
    const std::optional<double>& global_yaw,
    const gtsam::Point3& antenna_in_base = gtsam::Point3(0.0, 0.0, 0.0)) {
  const double yaw = global_yaw.value_or(local_pose.rotation().yaw());
  const gtsam::Rot3 rotation = gtsam::Rot3::RzRyRx(
      local_pose.rotation().roll(), local_pose.rotation().pitch(), yaw);
  gtsam::Point3 base_position = local_pose.translation();
  if (antenna_xy) {
    const gtsam::Point3 rotated_lever_arm = rotation.rotate(antenna_in_base);
    base_position = gtsam::Point3(
        antenna_xy->x() - rotated_lever_arm.x(),
        antenna_xy->y() - rotated_lever_arm.y(),
        local_pose.translation().z());
  }
  return gtsam::Pose3(rotation, base_position);
}

}  // namespace pgo
