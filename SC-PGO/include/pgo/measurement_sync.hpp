#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace pgo {

struct MeasurementSelection {
  std::optional<std::size_t> index;
  std::size_t stale_prefix{0};
  bool future_only{false};
  double skew_s{std::numeric_limits<double>::quiet_NaN()};
};

inline MeasurementSelection selectMeasurement(
    const std::vector<double>& stamps,
    double target_s,
    double max_past_age_s,
    double max_future_lead_s) {
  MeasurementSelection result;
  if (!std::isfinite(target_s) || !std::isfinite(max_past_age_s) ||
      !std::isfinite(max_future_lead_s) || max_past_age_s < 0.0 ||
      max_future_lead_s < 0.0) {
    return result;
  }

  while (result.stale_prefix < stamps.size() &&
         stamps[result.stale_prefix] < target_s - max_past_age_s) {
    ++result.stale_prefix;
  }
  if (result.stale_prefix == stamps.size()) {
    return result;
  }
  if (stamps[result.stale_prefix] > target_s + max_future_lead_s) {
    result.future_only = true;
    return result;
  }

  std::size_t best = result.stale_prefix;
  double best_abs_skew = std::abs(stamps[best] - target_s);
  for (std::size_t index = best + 1; index < stamps.size(); ++index) {
    if (stamps[index] > target_s + max_future_lead_s) {
      break;
    }
    const double abs_skew = std::abs(stamps[index] - target_s);
    if (abs_skew < best_abs_skew) {
      best = index;
      best_abs_skew = abs_skew;
    }
  }
  result.index = best;
  result.skew_s = stamps[best] - target_s;
  return result;
}

}  // namespace pgo
