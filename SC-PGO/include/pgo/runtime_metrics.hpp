#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace pgo {

struct RuntimeMetrics {
  std::uint64_t keyframes;
  std::uint64_t graph_updates;
  std::uint64_t gnss_accepted;
  std::uint64_t gnss_rejected;
  std::uint64_t heading_accepted;
  std::uint64_t heading_rejected;
  std::uint64_t loops_accepted;
  std::uint64_t gnss_stale_dropped;
  std::uint64_t heading_stale_dropped;
  std::uint64_t gnss_future_deferred;
  std::uint64_t heading_future_deferred;
  double gnss_skew_s_last;
  double heading_skew_s_last;
  double graph_update_ms_last;
};

inline std::string renderRuntimeMetricsJson(const RuntimeMetrics& metrics) {
  std::ostringstream output;
  output << "{\n"
         << "  \"keyframes\": " << metrics.keyframes << ",\n"
         << "  \"graph_updates\": " << metrics.graph_updates << ",\n"
         << "  \"gnss_accepted\": " << metrics.gnss_accepted << ",\n"
         << "  \"gnss_rejected\": " << metrics.gnss_rejected << ",\n"
         << "  \"heading_accepted\": " << metrics.heading_accepted << ",\n"
         << "  \"heading_rejected\": " << metrics.heading_rejected << ",\n"
         << "  \"loops_accepted\": " << metrics.loops_accepted << ",\n"
         << "  \"gnss_stale_dropped\": " << metrics.gnss_stale_dropped << ",\n"
         << "  \"heading_stale_dropped\": " << metrics.heading_stale_dropped << ",\n"
         << "  \"gnss_future_deferred\": " << metrics.gnss_future_deferred << ",\n"
         << "  \"heading_future_deferred\": " << metrics.heading_future_deferred << ",\n"
         << "  \"gnss_skew_s_last\": " << metrics.gnss_skew_s_last << ",\n"
         << "  \"heading_skew_s_last\": " << metrics.heading_skew_s_last << ",\n"
         << "  \"graph_update_ms_last\": " << metrics.graph_update_ms_last << "\n"
         << "}\n";
  return output.str();
}

}  // namespace pgo
