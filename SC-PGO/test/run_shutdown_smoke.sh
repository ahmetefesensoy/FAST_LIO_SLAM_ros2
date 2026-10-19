#!/usr/bin/env bash
set -euo pipefail

binary="${1:?usage: run_shutdown_smoke.sh <alaserPGO> <params.yaml>}"
params="${2:?usage: run_shutdown_smoke.sh <alaserPGO> <params.yaml>}"

"${binary}" --ros-args --params-file "${params}" &
pid=$!

sleep 3
kill -INT "${pid}"

for _ in $(seq 1 100); do
  if ! kill -0 "${pid}" 2>/dev/null; then
    wait "${pid}"
    exit $?
  fi
  sleep 0.1
done

kill -KILL "${pid}" 2>/dev/null || true
wait "${pid}" 2>/dev/null || true
echo "alaserPGO did not stop within 10 seconds" >&2
exit 1
