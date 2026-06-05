#!/usr/bin/env bash
# run_stage4c_full.sh — Stage 4C full multi-topology benchmark
# 6 topologies × 2 variants × 3 runs = 36 runs
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
HEALTH_LOG="/tmp/stage4c_full_health.log"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_DIR}/install/setup.bash"
set -u

WORLDS_DIR="$(ros2 pkg prefix tunnel_worlds)/share/tunnel_worlds/worlds"
CONFIG_DIR="$(ros2 pkg prefix tunnel_frontier_explorer)/share/tunnel_frontier_explorer/config"
BASELINE_PARAMS="${CONFIG_DIR}/frontier_explorer_params_info_revisit_r075.yaml"
ESCAPE_PARAMS="${CONFIG_DIR}/stage4b3/wall_risk_weak_escape.yaml"
OUT_ROOT="${HOME}/stage4c_full"
GEOMETRY_GATE_SECONDS=200

TOPOLOGIES=(straight_tunnel l_turn_tunnel branching_tunnel_y t_junction_tunnel dead_end_branch loop_tunnel)
VARIANTS=(stage3d_default stage4b4_escape)
RUN_IDS=(01 02 03)

log_health() {
  echo "[health $(date -u +%H:%M:%S)] $*" | tee -a "${HEALTH_LOG}"
}

cleanup() {
  pkill -f "centerline_extractor" 2>/dev/null || true
  pkill -f "tunnel_frontier_explorer" 2>/dev/null || true
  pkill -f "stage0_simulation" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ruby" 2>/dev/null || true
  sleep 3
}

needs_centerline() {
  [ "$1" != "stage3d_default" ]
}

total_runs=0; passed_runs=0; failed_runs=0

for topo in "${TOPOLOGIES[@]}"; do
  WORLD="${WORLDS_DIR}/${topo}.sdf"
  if [ ! -f "${WORLD}" ]; then log_health "SKIP: ${topo}"; continue; fi

  for variant in "${VARIANTS[@]}"; do
    for run_id in "${RUN_IDS[@]}"; do
      total_runs=$((total_runs + 1))
      config="${variant:+${CONFIG_DIR}/stage4b3/wall_risk_weak_escape.yaml}"
      [ "${variant}" = "stage3d_default" ] && config="${BASELINE_PARAMS}"
      use_centerline=false; needs_centerline "${variant}" && use_centerline=true

      log_health "START: ${topo}/${variant} run ${run_id} (${total_runs}/36)"
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

      # Poll Nav2
      NAV2_READY=false
      for i in $(seq 1 30); do
        timeout 3 ros2 node list 2>/dev/null | grep -q "/bt_navigator" && NAV2_READY=true && break
        sleep 10
      done
      if ! ${NAV2_READY}; then
        log_health "FAIL: Nav2 not ready — ${topo}/${variant}"
        kill "${BENCH_PID}" 2>/dev/null; cleanup; failed_runs=$((failed_runs + 1)); continue
      fi

      # Start centerline if needed
      if ${use_centerline}; then
        ros2 launch tunnel_centerline_extractor centerline_extractor.launch.py &
        CENTERLINE_PID=$!
        GATE_START=$(date +%s); MAP_OK=false; RISK_OK=false
        while true; do
          GE=$(($(date +%s) - GATE_START)); [ "${GE}" -ge "${GEOMETRY_GATE_SECONDS}" ] && break
          ! ${MAP_OK} && timeout 3 ros2 topic echo /tunnel_centerline/distance_map --once 2>/dev/null | grep -q "data:" && MAP_OK=true
          ! ${RISK_OK} && timeout 3 ros2 topic echo /tunnel_centerline/risk_map --once 2>/dev/null | grep -q "data:" && RISK_OK=true
          ${MAP_OK} && ${RISK_OK} && break; sleep 10
        done
        if ! ${MAP_OK} || ! ${RISK_OK}; then
          log_health "FAIL: geometry gate — ${topo}/${variant}"
          kill "${BENCH_PID}" 2>/dev/null; kill "${CENTERLINE_PID}" 2>/dev/null
          cleanup; failed_runs=$((failed_runs + 1)); continue
        fi
        log_health "Geometry gate PASSED"
      fi

      # Wait for benchmark
      set +e; wait "${BENCH_PID}"; set -e
      cleanup

      # Validate
      result_dir="${OUT_ROOT}/${topo}/${variant}/run_${run_id}/attempt_01"
      log="${result_dir}/frontier_explorer.log"
      status=$(python3 -c "import json; print(json.load(open('${result_dir}/benchmark_results.json')).get('run_status','?'))" 2>/dev/null || echo "?")
      forced=$(grep -c "ForcedEscapeProbe" "$log" 2>/dev/null || echo 0)
      type_a=$(grep -c "type=revisit_heavy" "$log" 2>/dev/null || echo 0)
      type_b=$(grep -c "type=alternating_pair" "$log" 2>/dev/null || echo 0)
      escape=$(grep -c "OscillationEscape" "$log" 2>/dev/null || echo 0)

      log_health "Result: ${topo}/${variant} run ${run_id} — ${status} forced=${forced} typeA=${type_a} typeB=${type_b} escape=${escape}"
      [ "${status}" = "COMPLETED" ] && passed_runs=$((passed_runs + 1)) || failed_runs=$((failed_runs + 1))
    done
  done
done

log_health "FULL COMPLETE: ${passed_runs}/${total_runs} completed, ${failed_runs} failed"
echo ""
echo "=== Summary ==="
for topo in "${TOPOLOGIES[@]}"; do
  echo "--- ${topo} ---"
  for variant in "${VARIANTS[@]}"; do
    completed=0
    for run_id in "${RUN_IDS[@]}"; do
      f="${OUT_ROOT}/${topo}/${variant}/run_${run_id}/attempt_01/benchmark_results.json"
      [ -f "$f" ] && status=$(python3 -c "import json; print(json.load(open('$f')).get('run_status','?'))" 2>/dev/null || echo "?") && [ "${status}" = "COMPLETED" ] && completed=$((completed + 1))
    done
    echo "  ${variant}: ${completed}/3"
  done
done
