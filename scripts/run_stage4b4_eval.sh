#!/usr/bin/env bash
# run_stage4b4_eval.sh — Stage 4B.4-Eval: oscillillation-aware escape benchmark
#
# Variants (5 runs each = 15 total):
#   1. fresh_stage3d_baseline            — no centerline
#   2. stage4b1_wall_risk_weak           — tunnel_aware, w_risk=0.3, centerline
#   3. stage4b4_wall_risk_weak_escape    — tunnel_aware + escape mode, centerline
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
HEALTH_LOG="/tmp/stage4b4_eval_health.log"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_DIR}/install/setup.bash"
set -u

WORLD="$(ros2 pkg prefix tunnel_worlds)/share/tunnel_worlds/worlds/branching_tunnel_y.sdf"
CONFIG_DIR="$(ros2 pkg prefix tunnel_frontier_explorer)/share/tunnel_frontier_explorer/config"
BASELINE_PARAMS="${CONFIG_DIR}/frontier_explorer_params_info_revisit_r075.yaml"
OUT_ROOT="${HOME}/stage4b4_eval/oscillation_escape"
GEOMETRY_GATE_SECONDS=200

# Variant: name, config_file, needs_centerline
declare -A VARIANT_CONFIG
VARIANT_CONFIG[fresh_stage3d_baseline]="${BASELINE_PARAMS}"
VARIANT_CONFIG[stage4b1_wall_risk_weak]="${CONFIG_DIR}/stage4b1/wall_risk_weak.yaml"
VARIANT_CONFIG[stage4b4_wall_risk_weak_escape]="${CONFIG_DIR}/stage4b3/wall_risk_weak_escape.yaml"

VARIANTS=(fresh_stage3d_baseline stage4b1_wall_risk_weak stage4b4_wall_risk_weak_escape)
RUN_IDS=(01 02 03 04 05)

log_health() {
  echo "[health $(date -u +%H:%M:%S)] $*" | tee -a "${HEALTH_LOG}"
}

cleanup_all() {
  pkill -f "centerline_extractor" 2>/dev/null || true
  pkill -f "tunnel_frontier_explorer" 2>/dev/null || true
  pkill -f "stage0_simulation" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ruby" 2>/dev/null || true
  sleep 3
}

needs_centerline() {
  local variant="$1"
  case "${variant}" in
    fresh_stage3d_baseline) return 1 ;;
    *) return 0 ;;
  esac
}

total_runs=0
passed_runs=0
failed_runs=0

for variant in "${VARIANTS[@]}"; do
  config_file="${VARIANT_CONFIG[${variant}]}"
  use_centerline=false
  if needs_centerline "${variant}"; then
    use_centerline=true
  fi

  for run_id in "${RUN_IDS[@]}"; do
    total_runs=$((total_runs + 1))
    echo ""
    echo "============================================================"
    log_health "START: ${variant} run ${run_id} (${total_runs}/15)"
    echo "============================================================"
    cleanup_all

    "${REPO_DIR}/scripts/run_stage2a_benchmark.sh" \
      --explorer-params-file "${config_file}" \
      --world "${WORLD}" \
      --output-dir "${OUT_ROOT}/${variant}" \
      --run-id "${run_id}" \
      --stop-on-completed true \
      --stall-timeout-seconds 240 \
      --wait-time 120 \
      --duration 900 &

    BENCH_PID=$!

    # ── Poll Nav2 readiness ──────────────────────────────────────────
    log_health "Polling /bt_navigator for ${variant}..."
    NAV2_READY=false
    for i in $(seq 1 30); do
      if timeout 3 ros2 node list 2>/dev/null | grep -q "/bt_navigator"; then
        NAV2_READY=true
        break
      fi
      sleep 10
    done
    if ! ${NAV2_READY}; then
      log_health "FAILED: Nav2 not ready after 300s — ${variant} run ${run_id}"
      kill "${BENCH_PID}" 2>/dev/null || true
      cleanup_all
      failed_runs=$((failed_runs + 1))
      continue
    fi

    # ── Start centerline if needed ──────────────────────────────────
    CENTERLINE_PID=""
    if ${use_centerline}; then
      log_health "Nav2 ready, starting centerline for ${variant}..."
      ros2 launch tunnel_centerline_extractor centerline_extractor.launch.py &
      CENTERLINE_PID=$!
      log_health "Centerline PID=${CENTERLINE_PID}"

      # Geometry gate
      log_health "Geometry gate: waiting up to ${GEOMETRY_GATE_SECONDS}s..."
      GATE_START=$(date +%s)
      MAP_OK=false
      RISK_OK=false
      while true; do
        GATE_ELAPSED=$(($(date +%s) - GATE_START))
        if [ "${GATE_ELAPSED}" -ge "${GEOMETRY_GATE_SECONDS}" ]; then
          break
        fi
        if ! ${MAP_OK} && timeout 3 ros2 topic echo /tunnel_centerline/distance_map --once 2>/dev/null | grep -q "data:"; then
          MAP_OK=true
          log_health "distance_map received at +${GATE_ELAPSED}s"
        fi
        if ! ${RISK_OK} && timeout 3 ros2 topic echo /tunnel_centerline/risk_map --once 2>/dev/null | grep -q "data:"; then
          RISK_OK=true
          log_health "risk_map received at +${GATE_ELAPSED}s"
        fi
        if ${MAP_OK} && ${RISK_OK}; then
          break
        fi
        sleep 10
      done
      if ! ${MAP_OK} || ! ${RISK_OK}; then
        log_health "FAIL: geometry gate timed out after ${GEOMETRY_GATE_SECONDS}s — ${variant} run ${run_id}"
        kill "${BENCH_PID}" 2>/dev/null || true
        kill "${CENTERLINE_PID}" 2>/dev/null || true
        cleanup_all
        failed_runs=$((failed_runs + 1))
        continue
      fi
      log_health "Geometry gate PASSED (map=${MAP_OK} risk=${RISK_OK})"
    fi

    # ── Wait for benchmark ──────────────────────────────────────────
    log_health "Waiting for benchmark PID=${BENCH_PID}..."
    set +e
    wait "${BENCH_PID}"
    bench_exit=$?
    set -e
    log_health "Benchmark exited with code ${bench_exit}"

    cleanup_all

    # ── Validate ────────────────────────────────────────────────────
    result_dir="${OUT_ROOT}/${variant}/run_${run_id}/attempt_01"
    explorer_log="${result_dir}/frontier_explorer.log"
    results_json="${result_dir}/benchmark_results.json"

    if [ ! -f "${results_json}" ]; then
      log_health "FAIL: ${variant} run ${run_id} — no benchmark_results.json"
      failed_runs=$((failed_runs + 1))
      continue
    fi

    run_status=$(python3 -c "
import json
with open('${results_json}') as f:
    d = json.load(f)
print(d.get('run_status', 'UNKNOWN'))
" 2>/dev/null || echo "PARSE_ERROR")

    # Count oscillation events from logs
    type_a=0; type_b=0; escape_count=0; fallback_count=0
    if [ -f "${explorer_log}" ]; then
      type_a=$(grep -c "type=revisit_heavy" "${explorer_log}" 2>/dev/null || echo 0)
      type_b=$(grep -c "type=alternating_pair" "${explorer_log}" 2>/dev/null || echo 0)
      escape_count=$(grep -c "\[OscillationEscape\]" "${explorer_log}" 2>/dev/null || echo 0)
      fallback_count=$(grep -ci "fallback\|geometry unavailable" "${explorer_log}" 2>/dev/null || echo 0)
    fi

    log_health "Result: ${variant} run ${run_id} — ${run_status} typeA=${type_a} typeB=${type_b} escape=${escape_count} fallback=${fallback_count}"

    if [ "${run_status}" = "COMPLETED" ]; then
      passed_runs=$((passed_runs + 1))
    else
      log_health "WARN: ${variant} run ${run_id} did not complete — check logs"
      failed_runs=$((failed_runs + 1))
    fi
  done
done

echo ""
echo "============================================================"
log_health "EVAL COMPLETE: ${passed_runs}/${total_runs} completed, ${failed_runs} failed"
echo "Results in: ${OUT_ROOT}/"
echo "============================================================"

# Final listing
for variant in "${VARIANTS[@]}"; do
  echo ""
  echo "--- ${variant} ---"
  completed=0
  for run_id in "${RUN_IDS[@]}"; do
    result_dir="${OUT_ROOT}/${variant}/run_${run_id}/attempt_01"
    if [ -f "${result_dir}/benchmark_results.json" ]; then
      status=$(python3 -c "
import json
with open('${result_dir}/benchmark_results.json') as f:
    d = json.load(f)
print(d.get('run_status','?'))
" 2>/dev/null || echo "PARSE_ERROR")
      type_a=$(grep -c "type=revisit_heavy" "${result_dir}/frontier_explorer.log" 2>/dev/null || echo 0)
      type_b=$(grep -c "type=alternating_pair" "${result_dir}/frontier_explorer.log" 2>/dev/null || echo 0)
      escape=$(grep -c "\[OscillationEscape\]" "${result_dir}/frontier_explorer.log" 2>/dev/null || echo 0)
      echo "  run_${run_id}: ${status} typeA=${type_a} typeB=${type_b} escape=${escape}"
      if [ "${status}" = "COMPLETED" ]; then
        completed=$((completed + 1))
      fi
    else
      echo "  run_${run_id}: NO_RESULTS"
    fi
  done
  echo "  => ${completed}/5 COMPLETED"
done
