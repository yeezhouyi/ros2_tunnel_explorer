#!/usr/bin/env bash
# run_stage4b1_mini_eval.sh — 3-run mini eval for wall_risk_tiebreaker
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_DIR}/install/setup.bash"
set -u

WORLD="$(ros2 pkg prefix tunnel_worlds)/share/tunnel_worlds/worlds/branching_tunnel_y.sdf"
EXPLORER_PARAMS="$(ros2 pkg prefix tunnel_frontier_explorer)/share/tunnel_frontier_explorer/config/stage4b1/wall_risk_tiebreaker.yaml"
OUT_ROOT="${HOME}/stage4b1_mini_eval"

cleanup_all() {
  pkill -f "centerline_extractor" 2>/dev/null || true
  pkill -f "tunnel_frontier_explorer" 2>/dev/null || true
  pkill -f "stage0_simulation" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ruby" 2>/dev/null || true
  sleep 3
}

for run_id in 01 02 03; do
  echo ""
  echo "==== $(date -u +%H:%M:%S) wall_risk_tiebreaker run ${run_id} ===="
  cleanup_all

  "${REPO_DIR}/scripts/run_stage2a_benchmark.sh" \
    --explorer-params-file "${EXPLORER_PARAMS}" \
    --world "${WORLD}" \
    --output-dir "${OUT_ROOT}/wall_risk_tiebreaker" \
    --run-id "${run_id}" \
    --stop-on-completed true \
    --stall-timeout-seconds 240 \
    --wait-time 120 \
    --duration 900 &

  BENCH_PID=$!

  echo "[mini_eval] Waiting for Nav2 (polling /bt_navigator)..."
  NAV2_READY=false
  for i in $(seq 1 30); do
    if timeout 3 ros2 node list 2>/dev/null | grep -q "/bt_navigator"; then
      NAV2_READY=true
      break
    fi
    sleep 10
  done
  if ! ${NAV2_READY}; then
    echo "[mini_eval] FAILED: Nav2 not ready after 300s"
    kill "${BENCH_PID}" 2>/dev/null || true
    cleanup_all
    exit 1
  fi
  echo "[mini_eval] Nav2 ready, starting centerline..."
  ros2 launch tunnel_centerline_extractor centerline_extractor.launch.py &
  CENTERLINE_PID=$!
  echo "[mini_eval] Centerline PID=${CENTERLINE_PID}"

  echo "[mini_eval] Waiting for benchmark PID=${BENCH_PID} to finish..."
  wait "${BENCH_PID}" || true

  cleanup_all
  echo "[mini_eval] Run ${run_id} done"
done

echo ""
echo "==== All 3 mini eval runs complete ===="
echo "Results in: ${OUT_ROOT}/wall_risk_tiebreaker/"
ls -la "${OUT_ROOT}/wall_risk_tiebreaker/" 2>/dev/null || echo "(no results dir yet)"
