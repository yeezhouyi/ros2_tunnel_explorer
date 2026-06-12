#!/usr/bin/env bash
# Stage 4G.1: Final Default 20-run Validation
# 4 non-straight topologies × 5 runs = 20 runs
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_DIR}/install/setup.bash"
set -u

WORLDS_DIR="$(ros2 pkg prefix tunnel_worlds)/share/tunnel_worlds/worlds"
CONFIG_DIR="$(ros2 pkg prefix tunnel_frontier_explorer)/share/tunnel_frontier_explorer/config"
FINAL_PARAMS="${CONFIG_DIR}/final/baseline_bootstrap.yaml"
OUT_ROOT="${HOME}/stage4g1_final"

TOPOLOGIES=(l_turn_tunnel dead_end_branch branching_tunnel_y t_junction_tunnel)
RUN_IDS=(01 02 03 04 05)

cleanup() {
  pkill -f "tunnel_frontier_explorer" 2>/dev/null || true
  pkill -f "stage0_simulation" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ruby" 2>/dev/null || true
  sleep 3
}

total=0; passed=0; failed=0

for topo in "${TOPOLOGIES[@]}"; do
  WORLD="${WORLDS_DIR}/${topo}.sdf"
  for run_id in "${RUN_IDS[@]}"; do
    total=$((total + 1))
    echo "==== $(date -u +%H:%M:%S) ${topo} run ${run_id} (${total}/20) ===="
    cleanup

    "${REPO_DIR}/scripts/run_stage2a_benchmark.sh" \
      --explorer-params-file "${FINAL_PARAMS}" \
      --world "${WORLD}" \
      --output-dir "${OUT_ROOT}/${topo}" \
      --run-id "${run_id}" \
      --stop-on-completed true \
      --stall-timeout-seconds 240 \
      --wait-time 120 \
      --duration 900 2>&1 || true

    result_dir="${OUT_ROOT}/${topo}/run_${run_id}/attempt_01"
    f="${result_dir}/benchmark_results.json"
    if [ -f "$f" ]; then
      status=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('run_status','?'))" 2>/dev/null || echo "?")
      goals=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('goals_dispatched',0))" 2>/dev/null || echo "0")
      time=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('completion_time_seconds',0))" 2>/dev/null || echo "0")
      echo "[final] ${topo} run ${run_id}: ${status} goals=${goals} time=${time}s"
      [ "${status}" = "COMPLETED" ] && passed=$((passed + 1)) || failed=$((failed+1))
    fi
  done
done

echo ""
echo "=== Summary ==="
echo "  Completed: ${passed}/${total}"
echo "  Failed: ${failed}/${total}"
for topo in "${TOPOLOGIES[@]}"; do
  completed=0
  for run_id in "${RUN_IDS[@]}"; do
    f="${OUT_ROOT}/${topo}/run_${run_id}/attempt_01/benchmark_results.json"
    [ -f "$f" ] && status=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('run_status','?'))" 2>/dev/null || echo "?") && [ "${status}" = "COMPLETED" ] && completed=$((completed + 1))
  done
  echo "  ${topo}: ${completed}/5"
done
