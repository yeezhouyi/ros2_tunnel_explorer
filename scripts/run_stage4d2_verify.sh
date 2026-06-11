#!/usr/bin/env bash
# Stage 4D.2 verification: baseline vs baseline+cooldown_recovery vs forced_escape
# 3 variants × 3 runs = 9 runs
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
OUT_ROOT="${HOME}/stage4d2_verify"
WORLD="${WORLDS_DIR}/l_turn_tunnel.sdf"

RUN_IDS=(01 02 03)
VARIANTS=(stage3d_baseline stage4d2_recovery stage4d2_escape)

# Create a config with cooldown recovery disabled for baseline
mkdir -p /tmp/stage4d2_configs
cat > /tmp/stage4d2_configs/baseline_no_recovery.yaml << 'YAML'
/**:
  ros__parameters:
    selection_strategy: information_gain_revisit
    weight_information_gain: 1.0
    weight_distance: 1.0
    weight_revisit: 1.5
    startup_bootstrap_enabled: true
    startup_bootstrap_distance_m: 0.75
    startup_bootstrap_max_attempts: 1
    startup_bootstrap_no_frontier_cycles: 10
    cooldown_starvation_recovery_enabled: false
    entrance_oscillation_enabled: true
    entrance_oscillation_window_goals: 6
    entrance_oscillation_radius_m: 2.0
    entrance_oscillation_min_repeated_goals: 4
    entrance_oscillation_max_unique_bins: 2
    entrance_oscillation_min_revisit_ratio: 0.35
    entrance_oscillation_min_goals_to_check: 4
    entrance_oscillation_detect_alternating_pair: true
    entrance_oscillation_pair_cluster_radius_m: 0.75
    entrance_oscillation_pair_max_spatial_radius_m: 1.5
    entrance_oscillation_pair_min_cluster_count: 2
    entrance_oscillation_pair_min_alternation_score: 0.5
    entrance_oscillation_response_enabled: false
    entrance_oscillation_escape_goals: 3
    entrance_oscillation_suppression_radius_m: 1.5
    entrance_oscillation_escape_penalty: 0.75
    entrance_oscillation_force_escape_probe: false
YAML

cleanup() {
  pkill -f "tunnel_frontier_explorer" 2>/dev/null || true
  pkill -f "stage0_simulation" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ruby" 2>/dev/null || true
  sleep 3
}

for variant in "${VARIANTS[@]}"; do
  if [ "${variant}" = "stage3d_baseline" ]; then
    params="/tmp/stage4d2_configs/baseline_no_recovery.yaml"
  elif [ "${variant}" = "stage4d2_recovery" ]; then
    params="${BASELINE_PARAMS}"
  else
    params="${CONFIG_DIR}/stage4d3/forced_escape.yaml"
  fi

  for run_id in "${RUN_IDS[@]}"; do
    echo ""
    echo "==== $(date -u +%H:%M:%S) ${variant} run ${run_id} ===="
    cleanup

    "${REPO_DIR}/scripts/run_stage2a_benchmark.sh" \
      --explorer-params-file "${params}" \
      --world "${WORLD}" \
      --output-dir "${OUT_ROOT}/${variant}" \
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
      echo "[verify] FAIL: Nav2 not ready"; kill "${BENCH_PID}" 2>/dev/null; cleanup; exit 1
    fi

    set +e; wait "${BENCH_PID}"; set -e
    cleanup

    result_dir="${OUT_ROOT}/${variant}/run_${run_id}/attempt_01"
    f="${result_dir}/benchmark_results.json"
    log="${result_dir}/frontier_explorer.log"
    if [ -f "$f" ]; then
      status=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('run_status','?'))" 2>/dev/null || echo "?")
      goals=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('goals_dispatched',0))" 2>/dev/null || echo "0")
      time=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('completion_time_seconds',0))" 2>/dev/null || echo "0")
      bootstrap=$(grep -c "StartupBootstrap" "$log" 2>/dev/null || echo 0)
      recovery=$(grep -c "CooldownRecovery" "$log" 2>/dev/null || echo 0)
      echo "[verify] ${variant} run ${run_id}: ${status} goals=${goals} time=${time}s bootstrap=${bootstrap} recovery=${recovery}"
    fi
  done
done

echo ""
echo "=== Summary ==="
for variant in "${VARIANTS[@]}"; do
  completed=0
  for run_id in "${RUN_IDS[@]}"; do
    f="${OUT_ROOT}/${variant}/run_${run_id}/attempt_01/benchmark_results.json"
    if [ -f "$f" ]; then
      status=$(python3 -c "import json; d=json.load(open('$f')); print(d.get('run_status','?'))" 2>/dev/null || echo "?")
      [ "${status}" = "COMPLETED" ] && completed=$((completed + 1))
    fi
  done
  echo "  ${variant}: ${completed}/3"
done
