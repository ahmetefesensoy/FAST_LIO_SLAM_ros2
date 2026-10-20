#!/usr/bin/env bash
set -euo pipefail

binary="$1"
config="$2"
node_log="$(mktemp)"
topic_output="$(mktemp)"
node_pid=""

cleanup() {
  if [[ -n "${node_pid}" ]] && kill -0 "${node_pid}" 2>/dev/null; then
    kill -INT "${node_pid}" 2>/dev/null || true
    for _ in $(seq 1 40); do
      kill -0 "${node_pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM "${node_pid}" 2>/dev/null || true
    wait "${node_pid}" 2>/dev/null || true
  fi
  rm -f "${node_log}" "${topic_output}"
}
trap cleanup EXIT

"${binary}" --ros-args --params-file "${config}" >"${node_log}" 2>&1 &
node_pid=$!

timeout 15 ros2 topic echo /pgo/diagnostics --once >"${topic_output}"
grep -q 'key: gnss_accepted' "${topic_output}"
grep -q 'key: graph_update_ms_last' "${topic_output}"

kill -INT "${node_pid}"
for _ in $(seq 1 80); do
  kill -0 "${node_pid}" 2>/dev/null || break
  sleep 0.1
done
if kill -0 "${node_pid}" 2>/dev/null; then
  echo "PGO node did not stop after diagnostics smoke test" >&2
  exit 1
fi
wait "${node_pid}"
node_pid=""
