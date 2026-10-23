#include <gtest/gtest.h>

#include "pgo/runtime_metrics.hpp"

TEST(PgoRuntimeMetrics, RendersMachineReadableFinalSnapshot) {
  const pgo::RuntimeMetrics metrics{
      42, 7, 5, 2, 4, 1, 3, 6, 8, 9, 10, -0.012, 0.018, 12.5};

  EXPECT_EQ(
      pgo::renderRuntimeMetricsJson(metrics),
      "{\n"
      "  \"keyframes\": 42,\n"
      "  \"graph_updates\": 7,\n"
      "  \"gnss_accepted\": 5,\n"
      "  \"gnss_rejected\": 2,\n"
      "  \"heading_accepted\": 4,\n"
      "  \"heading_rejected\": 1,\n"
      "  \"loops_accepted\": 3,\n"
      "  \"gnss_stale_dropped\": 6,\n"
      "  \"heading_stale_dropped\": 8,\n"
      "  \"gnss_future_deferred\": 9,\n"
      "  \"heading_future_deferred\": 10,\n"
      "  \"gnss_skew_s_last\": -0.012,\n"
      "  \"heading_skew_s_last\": 0.018,\n"
      "  \"graph_update_ms_last\": 12.5\n"
      "}\n");
}
