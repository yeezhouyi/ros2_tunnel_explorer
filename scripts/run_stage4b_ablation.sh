#!/usr/bin/env bash
# run_stage4b_ablation.sh
#
# Stage 4B: Tunnel-Aware Scoring — 4×5 ablation benchmark (v2).
#
# Centerline extractor starts at 20s (before first explorer goal at ~60s).
# Hard gate: centerline must publish distance_map/risk_map within 180s.
# Run validation: each 4B run must have at least one non-zero geometry feature.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_DIR}/install/setup.bash"
set -u

WORLD="$(ros2 pkg prefix tunnel_worlds)/share/tunnel_worlds/worlds/branching_tunnel_y.sdf"
PARAM_DIR="$(ros2 pkg prefix tunnel_frontier_explorer)/share/tunnel_frontier_explorer/config/stage4b"
OUT_ROOT="${HOME}/stage4b_benchmark"

VARIANTS=("risk_only" "centerline_only" "full")
RUN_IDS=("01" "02" "03" "04" "05")
GEOMETRY_GATE_SECONDS=200

cleanup_centerline() {
  pkill -f "centerline_extractor.launch.py" 2>/dev/null || true
  pkill -f "tunnel_centerline_extractor" 2>/dev/null || true
  sleep 2
}

for variant in "${VARIANTS[@]}"; do
  for run_id in "${RUN_IDS[@]}"; do
    echo "==== $(date -u +%H:%M:%S) ${variant} run ${run_id} ===="
    cleanup_centerline

    "${REPO_DIR}/scripts/run_stage2a_benchmark.sh" \
      --explorer-params-file "${PARAM_DIR}/${variant}.yaml" \
      --world "${WORLD}" \
      --output-dir "${OUT_ROOT}/${variant}" \
      --run-id "${run_id}" \
      --stop-on-completed true \
      --stall-timeout-seconds 240 \
      --duration 900 &

    BENCH_PID=$!

    # Start centerline immediately — it subscribes to /map and publishes
    # when ready.  First goal may fall back to Stage 3D while centerline
    # processes the initial map (~110 s); subsequent goals use geometry.
    echo "[stage4b] Starting centerline immediately..."
    ros2 launch tunnel_centerline_extractor centerline_extractor.launch.py &
    CENTERLINE_PID=$!
    echo "[stage4b] Centerline PID=${CENTERLINE_PID}"

    # Hard gate: verify geometry topics within GEOMETRY_GATE_SECONDS.
    GATE_START=$(date +%s)
    MAP_OK=false
    RISK_OK=false
    while true; do
      GATE_ELAPSED=$(($(date +%s) - GATE_START))
      if [ "${GATE_ELAPSED}" -ge "${GEOMETRY_GATE_SECONDS}" ]; then
        break
      fi
      if timeout 3 ros2 topic echo /tunnel_centerline/distance_map --once >/dev/null 2>&1; then
        MAP_OK=true
      fi
      if timeout 3 ros2 topic echo /tunnel_centerline/risk_map --once >/dev/null 2>&1; then
        RISK_OK=true
      fi
      if ${MAP_OK} && ${RISK_OK}; then
        echo "[stage4b] Geometry topics confirmed after ${GATE_ELAPSED}s"
        break
      fi
      sleep 10
    done
    if ! ${MAP_OK} || ! ${RISK_OK}; then
      echo "[stage4b] FAILED: geometry topics not ready after ${GEOMETRY_GATE_SECONDS}s"
      echo "[stage4b] distance_map=${MAP_OK} risk_map=${RISK_OK}"
      kill "${BENCH_PID}" 2>/dev/null || true
      kill "${CENTERLINE_PID}" 2>/dev/null || true
      cleanup_centerline
      exit 1
    fi

    set +e; wait "${BENCH_PID}"; set -e

    # Run-level validation: at least one non-zero geometry feature.
    EXPLORER_LOG="${OUT_ROOT}/${variant}/run_${run_id}/attempt_01/frontier_explorer.log"
    if [ -f "${EXPLORER_LOG}" ]; then
      NZ_ALIGN=$(grep -cP 'align=0\.(?!000)[0-9]|align=[1-9]' "${EXPLORER_LOG}" 2>/dev/null || echo 0)
      NZ_RISK=$(grep -cP 'risk=0\.(?!000)[0-9]|risk=[1-9]' "${EXPLORER_LOG}" 2>/dev/null || echo 0)
      echo "[stage4b] Run validation: non-zero align=${NZ_ALIGN}, non-zero risk=${NZ_RISK}"
      if [ "${NZ_ALIGN}" -eq 0 ] && [ "${NZ_RISK}" -eq 0 ]; then
        echo "[stage4b] INVALID: no non-zero geometry features in ${variant} run ${run_id}"
      fi
    fi

    if [[ -n "${CENTERLINE_PID:-}" ]]; then
      kill "${CENTERLINE_PID}" 2>/dev/null || true
      cleanup_centerline
    fi
    sleep 10
  done
done

echo "[stage4b] All 15 runs complete."
python3 "${SCRIPT_DIR}/stage4b_comparison.py"
