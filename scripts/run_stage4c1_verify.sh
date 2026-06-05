#!/usr/bin/env bash
# Stage 4C.1 verification: Y-world ×3, T-junction ×3, straight ×3
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_DIR}/install/setup.bash"
set -u

WORLDS_DIR="$(ros2 pkg prefix tunnel_worlds)/share/tunnel_worlds/worlds"
CONFIG_DIR="$(ros2 pkg prefix tunnel_frontier_explorer)/share/tunnel_frontier_explorer/config"
ESCAPE_PARAMS="${CONFIG_DIR}/stage4b3/wall_risk_weak_escape.yaml"
OUT_ROOT="${HOME}/stage4c1_verify"
GEOMETRY_GATE_SECONDS=200

TOPOLOGIES=(branching_tunnel_y t_junction_tunnel straight_tunnel)
RUN_IDS=(01 02 03)

cleanup() {
  pkill -f "centerline_extractor" 2>/dev/null || true
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
      --explorer-params-file "${ESCAPE_PARAMS}" \
      --world "${WORLD}" \
      --output-dir "${OUT_ROOT}/${topo}" \
      --run-id "${run_id}" \
      --stop-on-completed true \
      --stall-timeout-seconds 240 \
      --wait-time 120 \
      --duration 900 &

    BENCH_PID=$!

    echo "[verify] Waiting for Nav2..."
    NAV2_READY=false
    for i in $(seq 1 30); do
      if timeout 3 ros2 node list 2>/dev/null | grep -q "/bt_navigator"; then
        NAV2_READY=true; break
      fi
      sleep 10
    done
    if ! ${NAV2_READY}; then
      echo "[verify] FAILED: Nav2 not ready"; kill "${BENCH_PID}" 2>/dev/null; cleanup; exit 1
    fi

    echo "[verify] Starting centerline..."
    ros2 launch tunnel_centerline_extractor centerline_extractor.launch.py &
    CENTERLINE_PID=$!

    GATE_START=$(date +%s); MAP_OK=false; RISK_OK=false
    while true; do
      GATE_ELAPSED=$(($(date +%s) - GATE_START))
      [ "${GATE_ELAPSED}" -ge "${GEOMETRY_GATE_SECONDS}" ] && break
      if ! ${MAP_OK} && timeout 3 ros2 topic echo /tunnel_centerline/distance_map --once 2>/dev/null | grep -q "data:"; then MAP_OK=true; fi
      if ! ${RISK_OK} && timeout 3 ros2 topic echo /tunnel_centerline/risk_map --once 2>/dev/null | grep -q "data:"; then RISK_OK=true; fi
      ${MAP_OK} && ${RISK_OK} && break
      sleep 10
    done
    if ! ${MAP_OK} || ! ${RISK_OK}; then
      echo "[verify] FAIL: geometry gate timeout"; kill "${BENCH_PID}" 2>/dev/null; kill "${CENTERLINE_PID}" 2>/dev/null; cleanup; exit 1
    fi

    echo "[verify] Waiting for benchmark..."
    set +e; wait "${BENCH_PID}"; set -e
    cleanup

    result_dir="${OUT_ROOT}/${topo}/run_${run_id}/attempt_01"
    log="${result_dir}/frontier_explorer.log"
    status=$(python3 -c "import json; print(json.load(open('${result_dir}/benchmark_results.json')).get('run_status','?'))" 2>/dev/null || echo "?")
    forced=$(grep -c "\[ForcedEscapeProbe\]" "$log" 2>/dev/null || echo 0)
    echo "[verify] ${topo} run ${run_id}: ${status} forced_probes=${forced}"
  done
done

echo ""
echo "==== Summary ===="
for topo in "${TOPOLOGIES[@]}"; do
  for run_id in "${RUN_IDS[@]}"; do
    result_dir="${OUT_ROOT}/${topo}/run_${run_id}/attempt_01"
    status=$(python3 -c "import json; print(json.load(open('${result_dir}/benchmark_results.json')).get('run_status','?'))" 2>/dev/null || echo "?")
    forced=$(grep -c "\[ForcedEscapeProbe\]" "${result_dir}/frontier_explorer.log" 2>/dev/null || echo 0)
    echo "  ${topo}/${run_id}: ${status} forced=${forced}"
  done
done
