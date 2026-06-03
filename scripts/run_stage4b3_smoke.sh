#!/usr/bin/env bash
# Stage 4B.3 smoke test — 3 runs to verify oscillation escape mode
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_DIR}/install/setup.bash"
set -u

WORLD="$(ros2 pkg prefix tunnel_worlds)/share/tunnel_worlds/worlds/branching_tunnel_y.sdf"
PARAMS="$(ros2 pkg prefix tunnel_frontier_explorer)/share/tunnel_frontier_explorer/config/stage4b3/wall_risk_weak_escape.yaml"
OUT_ROOT="${HOME}/stage4b3_smoke/oscillation_escape"

cleanup() {
  pkill -f "tunnel_frontier_explorer" 2>/dev/null || true
  pkill -f "stage0_simulation" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ruby" 2>/dev/null || true
  sleep 3
}

for run_id in 01 02 03; do
  echo ""
  echo "==== $(date -u +%H:%M:%S) smoke run ${run_id} ===="
  cleanup

  "${REPO_DIR}/scripts/run_stage2a_benchmark.sh" \
    --explorer-params-file "${PARAMS}" \
    --world "${WORLD}" \
    --output-dir "${OUT_ROOT}" \
    --run-id "${run_id}" \
    --stop-on-completed true \
    --stall-timeout-seconds 240 \
    --duration 900 &

  BENCH_PID=$!

  echo "[smoke] Waiting for Nav2..."
  NAV2_READY=false
  for i in $(seq 1 30); do
    if timeout 3 ros2 node list 2>/dev/null | grep -q "/bt_navigator"; then
      NAV2_READY=true
      break
    fi
    sleep 10
  done
  if ! ${NAV2_READY}; then
    echo "[smoke] FAILED: Nav2 not ready"
    kill "${BENCH_PID}" 2>/dev/null || true
    cleanup
    exit 1
  fi

  echo "[smoke] Nav2 ready, waiting for benchmark PID=${BENCH_PID}..."
  wait "${BENCH_PID}" || true
  cleanup
  echo "[smoke] Run ${run_id} done"
done

echo ""
echo "==== All smoke runs complete ===="
echo ""

# Summary
echo "--- Per-run status ---"
for run_id in 01 02 03; do
  f="${OUT_ROOT}/run_${run_id}/attempt_01/benchmark_results.md"
  log="${OUT_ROOT}/run_${run_id}/attempt_01/frontier_explorer.log"
  if [ -f "$f" ]; then
    status=$(grep "Run status" "$f" | sed 's/.*| \(.*\) |/\1/')
    goals=$(grep "Goals dispatched" "$f" | sed 's/.*| \(.*\) |/\1/')
    osc_events=$(grep -c "\[EntranceOscillation\]" "$log" 2>/dev/null || echo 0)
    escape_events=$(grep -c "\[OscillationEscape\]" "$log" 2>/dev/null || echo 0)
    echo "  run_${run_id}: ${status} goals=${goals} oscillation=${osc_events} escape=${escape_events}"
  fi
done

echo ""
echo "--- [OscillationEscape] lines ---"
grep -rn "\[OscillationEscape\]" "${OUT_ROOT}/"*/attempt_01/frontier_explorer.log 2>/dev/null | head -30 || echo "  (none found)"

echo ""
echo "--- [EntranceOscillation] WARN lines ---"
grep -rn "\[EntranceOscillation\]" "${OUT_ROOT}/"*/attempt_01/frontier_explorer.log 2>/dev/null | head -20 || echo "  (none found)"
