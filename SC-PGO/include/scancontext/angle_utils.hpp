#pragma once

#include <cmath>
#include <limits>

namespace scancontext {

inline float azimuthDegrees(float x, float y) {
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  if (x == 0.0F && y == 0.0F) {
    return 0.0F;
  }
  constexpr float kRadToDeg = 57.295779513082320876F;
  float angle = std::atan2(y, x) * kRadToDeg;
  if (angle < 0.0F) {
    angle += 360.0F;
  }
  return angle;
}

}  // namespace scancontext
