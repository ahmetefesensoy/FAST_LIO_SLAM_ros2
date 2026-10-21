#!/usr/bin/env bash
set -euo pipefail

binary="$1"
config="$2"
metrics_file="$(mktemp)"
node_log="$(mktemp)"
rm -f "${metrics_file}"
node_pid=""

cleanup() {
  if [[ -n "${node_pid}" ]] && kill -0 "${node_pid}" 2>/dev/null; then
    kill -TERM "${node_pid}" 2>/dev/null || true
    wait "${node_pid}" 2>/dev/null || true
  fi
  rm -f "${metrics_file}" "${node_log}"
}
trap cleanup EXIT

"${binary}" --ros-args --params-file "${config}" \
  -p "metrics_output_path:=${metrics_file}" >"${node_log}" 2>&1 &
node_pid=$!
sleep 2
kill -INT "${node_pid}"
wait "${node_pid}"
node_pid=""

python3 - "${metrics_file}" <<'PY'
import json
from pathlib import Path
import sys

metrics = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
required = {
    "keyframes", "graph_updates", "gnss_accepted", "gnss_rejected",
    "heading_accepted", "heading_rejected", "loops_accepted",
    "graph_update_ms_last",
}
missing = required.difference(metrics)
if missing:
    raise SystemExit(f"missing final metrics: {sorted(missing)}")
PY
