#!/usr/bin/env bash
# run_stage4b_ablation.sh
#
# Stage 4B: Tunnel-Aware Scoring — 4×5 ablation benchmark.
#
# Runs 4 variants (stage3d_baseline / risk_only / centerline_only / full)
# × 5 runs each on the Y-shaped branching tunnel world.
#
# The centerline extractor is started AFTER the benchmark simulation
# (after ~90 s Nav2 startup) to avoid ROS daemon / DDS disconnection
# from the benchmark's cleanup step.
#
# Usage:
#   ./scripts/run_stage4b_ablation.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ── Resolve paths ──────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "${REPO_DIR}/install/setup.bash"

WORLD="$(ros2 pkg prefix tunnel_worlds)/share/tunnel_worlds/worlds/branching_tunnel_y.sdf"
PARAM_DIR="$(ros2 pkg prefix tunnel_frontier_explorer)/share/tunnel_frontier_explorer/config/stage4b"
OUT_ROOT="${HOME}/stage4b_benchmark"

VARIANTS=("stage3d_baseline" "risk_only" "centerline_only" "full")
RUN_IDS=("01" "02" "03" "04" "05")

cleanup_centerline() {
  pkill -f "centerline_extractor.launch.py" 2>/dev/null || true
  pkill -f "tunnel_centerline_extractor" 2>/dev/null || true
  sleep 2
}

echo "============================================================"
echo " Stage 4B Tunnel-Aware Scoring — 4×5 Ablation"
echo " Output: ${OUT_ROOT}"
echo "============================================================"
echo ""

for variant in "${VARIANTS[@]}"; do
  for run_id in "${RUN_IDS[@]}"; do
    echo "============================================================"
    echo " Stage 4B: variant=${variant}  run=${run_id}  ($(date -u +%H:%M:%S))"
    echo "============================================================"

    cleanup_centerline

    # ── Launch benchmark (simulation + SLAM + Nav2 + explorer) ──────────
    "${SCRIPT_DIR}/run_stage2a_benchmark.sh" \
      --explorer-params-file "${PARAM_DIR}/${variant}.yaml" \
      --world "${WORLD}" \
      --output-dir "${OUT_ROOT}/${variant}" \
      --run-id "${run_id}" \
      --stop-on-completed true \
      --stall-timeout-seconds 240 \
      --duration 900 &

    BENCH_PID=$!

    # ── Start centerline extractor AFTER Nav2 is ready ──────────────────
    if [[ "${variant}" != "stage3d_baseline" ]]; then
      echo "[stage4b] Waiting 95 s before starting centerline extractor..."
      sleep 95

      echo "[stage4b] Launching centerline extractor..."
      ros2 launch tunnel_centerline_extractor centerline_extractor.launch.py &
      CENTERLINE_PID=$!
      echo "[stage4b] Centerline PID=${CENTERLINE_PID}"
    else
      CENTERLINE_PID=""
      echo "[stage4b] Stage 3D baseline: centerline extractor intentionally disabled."
    fi

    # ── Wait for benchmark to finish ────────────────────────────────────
    set +e
    wait "${BENCH_PID}"
    BENCH_EXIT=$?
    set -e

    # ── Stop centerline ─────────────────────────────────────────────────
    if [[ -n "${CENTERLINE_PID:-}" ]]; then
      kill "${CENTERLINE_PID}" 2>/dev/null || true
      cleanup_centerline
    fi

    if [[ "${BENCH_EXIT}" != "0" ]]; then
      echo "[stage4b] WARNING: benchmark exit code ${BENCH_EXIT} (variant=${variant}, run=${run_id})"
    fi

    echo "[stage4b] Done: variant=${variant}, run=${run_id}"
    sleep 10
  done
done

# ── Generate comparison table ────────────────────────────────────────
echo ""
echo "[stage4b] All 20 runs complete. Generating comparison..."
python3 "${SCRIPT_DIR}/stage4b_comparison.py"

echo ""
echo "[stage4b] Ablation complete. Results in ${OUT_ROOT}/"
