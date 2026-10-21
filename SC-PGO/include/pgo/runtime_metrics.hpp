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
         << "  \"graph_update_ms_last\": " << metrics.graph_update_ms_last << "\n"
         << "}\n";
  return output.str();
}

}  // namespace pgo
