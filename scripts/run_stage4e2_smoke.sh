#!/usr/bin/env bash
# Stage 4E.2 smoke test — 3 topologies × 3 runs = 9 runs
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_DIR}/install/setup.bash"
set -u

WORLDS_DIR="$(ros2 pkg prefix tunnel_worlds)/share/tunnel_worlds/worlds"
CONFIG_DIR="$(ros2 pkg prefix tunnel_frontier_explorer)/share/tunnel_frontier_explorer/config"
BASELINE_PARAMS="${CONFIG_DIR}/frontier_explorer_params_info_revisit_r075.yaml"
OUT_ROOT="${HOME}/stage4e2_smoke"

TOPOLOGIES=(l_turn_tunnel dead_end_branch branching_tunnel_y)
RUN_IDS=(01 02 03)

cleanup() {
  pkill -f "tunnel_frontier_explorer" 2>/dev/null || true
  pkill -f "stage0_simulation" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ruby" 2>/dev/null || true
  sleep 3
}

for topo in "${TOPOLOGIES[@]}"; do
  WORLD="${WORLDS_DIR}/${topo}.sdf"
  for run_id in "${RUN_IDS[@]}"; do
    echo ""
    echo "==== $(date -u +%H:%M:%S) ${topo} run ${run_id} ===="
    cleanup

    "${REPO_DIR}/scripts/run_stage2a_benchmark.sh" \
      --explorer-params-file "${BASELINE_PARAMS}" \
      --world "${WORLD}" \
      --output-dir "${OUT_ROOT}/${topo}" \
      --run-id "${run_id}" \
      --stop-on-completed true \
      --stall-timeout-seconds 240 \
      --wait-time 120 \
      --duration 900 &

    BENCH_PID=$!

    NAV2_READY=false
    for i in $(seq 1 30); do
      timeout 3 ros2 node list 2>/dev/null | grep -q "/bt_navigator" && NAV2_READY=true && break
      sleep 10
    done
    if ! ${NAV2_READY}; then
      echo "[smoke] FAIL: Nav2 not ready"; kill "${BENCH_PID}" 2>/dev/null; cleanup; exit 1
    fi

    set +e; wait "${BENCH_PID}"; set -e
    cleanup

    result_dir="${OUT_ROOT}/${topo}/run_${run_id}/attempt_01"
    f="${result_dir}/benchmark_results.json"
    if [ -f "$f" ]; then
      status=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('run_status','?'))" 2>/dev/null || echo "?")
      goals=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('goals_dispatched',0))" 2>/dev/null || echo "0")
      time=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('completion_time_seconds',0))" 2>/dev/null || echo "0")
      echo "[smoke] ${topo} run ${run_id}: ${status} goals=${goals} time=${time}s"
    fi
  done
done

echo ""
echo "=== Summary ==="
for topo in "${TOPOLOGIES[@]}"; do
  for run_id in "${RUN_IDS[@]}"; do
    f="${OUT_ROOT}/${topo}/run_${run_id}/attempt_01/benchmark_results.json"
    if [ -f "$f" ]; then
      status=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('run_status','?'))" 2>/dev/null || echo "?")
      goals=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('goals_dispatched',0))" 2>/dev/null || echo "0")
      echo "  ${topo}/${run_id}: ${status} goals=${goals}"
    fi
  done
done
