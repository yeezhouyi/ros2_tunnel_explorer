#!/usr/bin/env bash
# Stage 4F: Strategy Differentiation — Performance Evaluation
# 5 topologies × 4 variants × 3 runs = 60 runs
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
OUT_ROOT="${HOME}/stage4f_benchmark"

TOPOLOGIES=(l_turn_tunnel dead_end_branch branching_tunnel_y t_junction_tunnel straight_tunnel)
VARIANTS=(stage3d_baseline stage4d2_recovery stage4d3_forced_escape stage4b1_wall_risk_weak)
RUN_IDS=(01 02 03)

cleanup() {
  pkill -f "centerline_extractor" 2>/dev/null || true
  pkill -f "tunnel_frontier_explorer" 2>/dev/null || true
  pkill -f "stage0_simulation" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ruby" 2>/dev/null || true
  sleep 3
}

needs_centerline() {
  [ "$1" != "stage3d_baseline" ]
}

total_runs=0; passed=0; failed=0

for topo in "${TOPOLOGIES[@]}"; do
  WORLD="${WORLDS_DIR}/${topo}.sdf"
  [ ! -f "${WORLD}" ] && continue

  for variant in "${VARIANTS[@]}"; do
    config="${BASELINE_PARAMS}"
    [ "${variant}" = "stage4d2_recovery" ] && config="${CONFIG_DIR}/stage4d/cooldown_recovery.yaml"
    [ "${variant}" = "stage4d3_forced_escape" ] && config="${CONFIG_DIR}/stage4d/conservative_fallback.yaml"
    [ "${variant}" = "stage4b1_wall_risk_weak" ] && config="${CONFIG_DIR}/stage4b1/wall_risk_weak.yaml"
    use_centerline=false; needs_centerline "${variant}" && use_centerline=true

    for run_id in "${RUN_IDS[@]}"; do
      total_runs=$((total_runs + 1))
      log_health() { echo "[health $(date -u +%H:%M:%S)] $*"; }
      log_health "START: ${topo}/${variant} run ${run_id} (${total_runs}/60)"
      cleanup

      "${REPO_DIR}/scripts/run_stage2a_benchmark.sh" \
        --explorer-params-file "${config}" \
        --world "${WORLD}" \
        --output-dir "${OUT_ROOT}/${topo}/${variant}" \
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
        log_health "FAIL: Nav2 not ready"; kill "${BENCH_PID}" 2>/dev/null; cleanup; failed=$((failed+1)); continue
      fi

      if ${use_centerline}; then
        ros2 launch tunnel_centerline_extractor centerline_extractor.launch.py &
        CENTERLINE_PID=$!
        GATE_START=$(date +%s); MAP_OK=false; RISK_OK=false
        while true; do
          GE=$(($(date +%s) - GATE_START)); [ "${GE}" -ge "${GEOMETRY_GATE_SECONDS:-200}" ] && break
          ! ${MAP_OK} && timeout 3 ros2 topic echo /tunnel_centerline/distance_map --once 2>/dev/null | grep -q "data:" && MAP_OK=true
          ! ${RISK_OK} && timeout 3 ros2 topic echo /tunnel_centerline/risk_map --once 2>/dev/null | grep -q "data:" && RISK_OK=true
          ${MAP_OK} && ${RISK_OK} && break; sleep 10
        done
      fi

      set +e; wait "${BENCH_PID}"; set -e
      cleanup

      f="${OUT_ROOT}/${topo}/${variant}/run_${run_id}/attempt_01/benchmark_results.json"
      log="${OUT_ROOT}/${topo}/${variant}/run_${run_id}/attempt_01/frontier_explorer.log"
      if [ -f "$f" ]; then
        status=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('run_status','?'))" 2>/dev/null || echo "?")
        goals=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('goals_dispatched',0))" 2>/dev/null || echo "0")
        time=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('completion_time_seconds',0))" 2>/dev/null || echo "0")
        forced=$(grep -c "ForcedEscapeProbe" "$log" 2>/dev/null || echo 0)
        echo "[verify] ${topo}/${variant} run ${run_id}: ${status} goals=${goals} time=${time}s forced=${forced}"
        [ "${status}" = "COMPLETED" ] && passed=$((passed + 1)) || failed=$((failed+1))
      fi
    done
  done
done

log_health "BENCHMARK COMPLETE: ${passed}/${total_runs} completed"
echo "=== Summary ==="
for topo in "${TOPOLOGIES[@]}"; do
  for variant in "${VARIANTS[@]}"; do
    completed=0
    for run_id in "${RUN_IDS[@]}"; do
      f="${OUT_ROOT}/${topo}/${variant}/run_${run_id}/attempt_01/benchmark_results.json"
      [ -f "$f" ] && status=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('run_status','?'))" 2>/dev/null || echo "?") && [ "${status}" = "COMPLETED" ] && completed=$((completed + 1))
    done
    echo "  ${topo}/${variant}: ${completed}/3"
  done
done
