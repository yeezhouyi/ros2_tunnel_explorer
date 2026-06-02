#!/usr/bin/env bash
# Copyright 2026 zhouyi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# run_stage2a_benchmark.sh
#
# Stage 2A: Nearest-Frontier Baseline Benchmark — single run.
#
# Fully automated: launches simulation, waits for Nav2, runs the frontier
# explorer for a fixed duration, records a rosbag, and extracts metrics.
#
# Usage:
#   ./scripts/run_stage2a_benchmark.sh [options]
#
# Options:
#   --output-dir DIR  Output directory (default: ~/stage2a_benchmark)
#   --duration SEC    Max benchmark duration in seconds (default: 600 = 10 min)
#   --run-id ID       Run identifier (default: auto-increment)
#   --wait-time SEC   Nav2 startup wait per attempt (default: 90, WSL2 DDS)
#   --stop-on-completed BOOL  Stop early when explorer enters COMPLETED (default: false)
#   --completed-grace-seconds SEC  Stable COMPLETED grace window (default: 20)
#   --stall-timeout-seconds SEC  Stall timeout in seconds (default: 90)
#   --runtime-retries N          Number of runtime retries for STALLED/STARTUP_FAILED (default: 0)
#   --inject-stall-after-seconds SEC  Inject stall for testing (default: 0, disabled)
#   --explorer-params-file PATH  Frontier explorer YAML params file (default: installed frontier_explorer_params.yaml)
#   --world PATH      Gazebo world SDF/SDF.xacro file (default: launch file default = tb3_sandbox.sdf.xacro)
#   --headless        Run Gazebo headless (default: true)
#   --help            Show this message
#
# Output structure:
#   <output-dir>/run_<id>/
#     run_summary.md             — Attempt overview
#     attempt_01/
#       frontier_explorer.log    — Explorer node log
#       simulation.log           — Simulation launch log
#       cpu_ram_usage.csv        — CPU/RSS samples every 10 s
#       benchmark_results.md     — Extracted metrics
#       rosbag_metrics.md        — Odometry + map coverage from bag
#       rosbag_metrics.json      — Structured bag metrics
#       bag/                     — ROS2 bag
#     attempt_02/ ...
#
# Recorded topics (rosbag):
#   /map /scan /tf /tf_static /odom /cmd_vel /clock
#   /local_costmap/costmap_raw /global_costmap/costmap_raw
#   /local_costmap/published_footprint
#   /tunnel_frontier_explorer/frontier_markers
#
# Prerequisites:
#   - ROS2 Jazzy and workspace built (colcon build)
#   - No other simulation processes running (run cleanup_simulation.sh first)
#
# Exit codes:
#   0 — Benchmark completed (COMPLETED or TIMEOUT)
#   1 — Explorer crashed, Nav2 startup failed, stalled, or invalid termination

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

# ── Defaults ───────────────────────────────────────────────────────────

OUTPUT_DIR="${HOME}/stage2a_benchmark"
DURATION=600
RUN_ID=""
WAIT_TIME=90
HEADLESS="True"
MAX_STARTUP_ATTEMPTS=3
BIN_SIZE=0.25  # metres for spatial binning of goals
STOP_ON_COMPLETED=false
COMPLETED_GRACE_SECONDS=20
STALL_TIMEOUT_SECONDS=90
RUNTIME_RETRIES=0
INJECT_STALL_AFTER_SECONDS=0
EXPLORER_PARAMS_FILE=""
WORLD=""

# ── Parse arguments ────────────────────────────────────────────────────

while [ $# -gt 0 ]; do
  case "$1" in
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --duration)   DURATION="$2";   shift 2 ;;
    --run-id)     RUN_ID="$2";     shift 2 ;;
    --wait-time)  WAIT_TIME="$2";  shift 2 ;;
    --stop-on-completed) STOP_ON_COMPLETED="$2"; shift 2 ;;
    --completed-grace-seconds) COMPLETED_GRACE_SECONDS="$2"; shift 2 ;;
    --stall-timeout-seconds) STALL_TIMEOUT_SECONDS="$2"; shift 2 ;;
    --runtime-retries) RUNTIME_RETRIES="$2"; shift 2 ;;
    --inject-stall-after-seconds) INJECT_STALL_AFTER_SECONDS="$2"; shift 2 ;;
    --explorer-params-file) EXPLORER_PARAMS_FILE="$2"; shift 2 ;;
    --world)   WORLD="$2";   shift 2 ;;
    --headless)   HEADLESS="$2";   shift 2 ;;
    --help)
      sed -n 's/^# //p; /^$/q' "$0" | head -80
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

# Auto-increment run ID
if [ -z "${RUN_ID}" ]; then
  NEXT_RUN=1
  while [ -d "${OUTPUT_DIR}/run_$(printf '%02d' "${NEXT_RUN}")" ]; do
    NEXT_RUN=$((NEXT_RUN + 1))
  done
  RUN_ID="$(printf '%02d' "${NEXT_RUN}")"
fi

if [ "${INJECT_STALL_AFTER_SECONDS}" -gt 0 ]; then
  INJECTED_STALL_ENABLED=true
fi

RUN_DIR="${OUTPUT_DIR}/run_${RUN_ID}"
PARAMS_FILE="${REPO_DIR}/install/tunnel_explorer_bringup/share/tunnel_explorer_bringup/config/nav2_params_rotation_shim.yaml"

# Resolve default explorer params file.
if [ -z "${EXPLORER_PARAMS_FILE}" ]; then
  EXPLORER_PARAMS_FILE="${REPO_DIR}/install/tunnel_frontier_explorer/share/tunnel_frontier_explorer/config/frontier_explorer_params.yaml"
fi
STAGE_SESSION="nav2_stage0"
EXPLORER_SESSION="frontier_explorer"

# ── Source environment ─────────────────────────────────────────────────

if ! command -v ros2 &> /dev/null; then
  echo "[${SCRIPT_NAME}] Sourcing /opt/ros/jazzy/setup.bash..."
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
if [ -z "${COLCON_PREFIX_PATH:-}" ]; then
  echo "[${SCRIPT_NAME}] Sourcing workspace install/setup.bash..."
  set +u
  source "${REPO_DIR}/install/setup.bash"
  set -u
fi

mkdir -p "${RUN_DIR}"

# ── State variables (set during run) ───────────────────────────────────

EXPLORER_CRASHED=false
STARTUP_ATTEMPTS=0
NAV2_READY=false
RUN_STATUS="TIMEOUT"
COMPLETION_DETECTED=false
COMPLETION_AT_SEC=0
COMPLETION_GOAL_COUNT=-1
STALL_DETECTED=false
STALLED_AFTER_SECONDS=0
LAST_PROGRESS_EPOCH=0
LAST_PROGRESS_EVENT="none"
INJECTED_STALL_ENABLED=false
[ "${INJECT_STALL_AFTER_SECONDS}" -gt 0 ] 2>/dev/null && INJECTED_STALL_ENABLED=true
INJECTION_ACTIVE=false
INJECTION_ACTIVATED_EPOCH=0
COMPLETION_CANDIDATE_EPOCH=0
COMPLETION_RESET_COUNT=0
COMPLETION_RESET_REASON=""
STABLE_COMPLETION_DETECTED=false

ATTEMPT_NUM=1
ALL_ATTEMPT_STATUSES=""

# ── Helper: sample CPU/RSS ─────────────────────────────────────────────

sample_cpu_ram() {
  local output="$1"
  printf "elapsed_sec,explorer_pcpu,explorer_rss_kb,gzsim_pcpu,gzsim_rss_kb\n" > "${output}"
  local start_time
  start_time="$(date +%s)"
  while true; do
    local now elapsed
    now="$(date +%s)"
    elapsed=$((now - start_time))

    # TunnelFrontierExplorerNode
    local explorer_pid explorer_pcpu explorer_rss ps_data
    explorer_pid="$(pgrep -f "tunnel_frontier_explorer" 2>/dev/null | head -1 || true)"
    if [ -n "${explorer_pid}" ]; then
      ps_data="$(ps -p "${explorer_pid}" -o pcpu=,rss= 2>/dev/null || echo "0 0")"
      explorer_pcpu="$(echo "${ps_data}" | awk '{print $1}')"
      explorer_rss="$(echo "${ps_data}" | awk '{print $2}')"
    else
      explorer_pcpu="0"
      explorer_rss="0"
    fi

    # gz sim
    local gz_pid gz_pcpu gz_rss
    gz_pid="$(pgrep -f "gz sim" 2>/dev/null | head -1 || true)"
    if [ -n "${gz_pid}" ]; then
      ps_data="$(ps -p "${gz_pid}" -o pcpu=,rss= 2>/dev/null || echo "0 0")"
      gz_pcpu="$(echo "${ps_data}" | awk '{print $1}')"
      gz_rss="$(echo "${ps_data}" | awk '{print $2}')"
    else
      gz_pcpu="0"
      gz_rss="0"
    fi

    printf "%d,%.1f,%d,%.1f,%d\n" \
      "${elapsed}" "${explorer_pcpu}" "${explorer_rss}" "${gz_pcpu}" "${gz_rss}" \
      >> "${output}"
    sleep 10
  done
}

# ── Helper: cleanup on trap ────────────────────────────────────────────

cleanup() {
  echo "[${SCRIPT_NAME}] Cleaning up..."
  kill "${CPU_SAMPLE_PID:-}" 2>/dev/null || true
  kill "${BAG_PID:-}" 2>/dev/null || true
  tmux kill-session -t "${EXPLORER_SESSION}" 2>/dev/null || true
  sleep 1
  tmux kill-session -t "${STAGE_SESSION}" 2>/dev/null || true
  sleep 2
  "${SCRIPT_DIR}/cleanup_simulation.sh" 2>/dev/null || true
  echo "[${SCRIPT_NAME}] Cleanup done."
}
trap cleanup EXIT INT TERM

# ══════════════════════════════════════════════════════════════════════
# Retry loop
# ══════════════════════════════════════════════════════════════════════

while [ "${ATTEMPT_NUM}" -le $((RUNTIME_RETRIES + 1)) ]; do
  ATTEMPT_DIR="${RUN_DIR}/attempt_$(printf '%02d' ${ATTEMPT_NUM})"
  SIM_LOG="${ATTEMPT_DIR}/simulation.log"
  EXPLORER_LOG="${ATTEMPT_DIR}/frontier_explorer.log"
  CPU_LOG="${ATTEMPT_DIR}/cpu_ram_usage.csv"
  RESULTS_FILE="${ATTEMPT_DIR}/benchmark_results.md"
  BAG_DIR="${ATTEMPT_DIR}/bag"
  mkdir -p "${ATTEMPT_DIR}"

  # Reset per-attempt state
  EXPLORER_CRASHED=false
  NAV2_READY=false
  COMPLETION_DETECTED=false
  COMPLETION_AT_SEC=0
  COMPLETION_GOAL_COUNT=-1
  STALL_DETECTED=false
  STALLED_AFTER_SECONDS=0
  LAST_PROGRESS_EPOCH=0
  LAST_PROGRESS_EVENT="none"
  INJECTED_STALL_ENABLED=false
  [ "${INJECT_STALL_AFTER_SECONDS}" -gt 0 ] 2>/dev/null && INJECTED_STALL_ENABLED=true
  INJECTION_ACTIVE=false
  INJECTION_ACTIVATED_EPOCH=0
  COMPLETION_CANDIDATE_EPOCH=0
  COMPLETION_RESET_COUNT=0
  COMPLETION_RESET_REASON=""
  STABLE_COMPLETION_DETECTED=false
  STARTUP_ATTEMPTS=0
  PIPELINE_FAILED=false
  RUN_STATUS="TIMEOUT"
  LOG_LINES_CHECKED=0

  echo "[${SCRIPT_NAME}] === Attempt ${ATTEMPT_NUM}/$((RUNTIME_RETRIES + 1)) ==="

  # ── Run header ───────────────────────────────────────────────────────

  {
    echo "============================================"
    echo " Stage 2A Baseline Benchmark — Run ${RUN_ID}"
    echo "============================================"
    echo ""
    echo " Output:  ${ATTEMPT_DIR}"
    echo " Duration: ${DURATION} s"
    echo " Date:    $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo " Tag:     stage1c-nearest-frontier-pass"
    echo " Params:  ${PARAMS_FILE}"
    echo ""
  } | tee "${RESULTS_FILE}"

  # ══════════════════════════════════════════════════════════════════════
  # Step 1: Clean up stale processes
  # ══════════════════════════════════════════════════════════════════════

  echo "[${SCRIPT_NAME}] Step 1/6: Cleaning up stale processes..."
  tmux kill-session -t "${EXPLORER_SESSION}" 2>/dev/null || true
  tmux kill-session -t "${STAGE_SESSION}" 2>/dev/null || true
  "${SCRIPT_DIR}/cleanup_simulation.sh"

  # ══════════════════════════════════════════════════════════════════════
  # Steps 2-3: Launch simulation + verify Nav2 (with retry)
  # ══════════════════════════════════════════════════════════════════════

  echo "[${SCRIPT_NAME}] Step 2-3/6: Launching simulation and verifying Nav2..."

  for attempt in $(seq 1 "${MAX_STARTUP_ATTEMPTS}"); do
    STARTUP_ATTEMPTS="${attempt}"

    if [ "${attempt}" -gt 1 ]; then
      echo "[${SCRIPT_NAME}] Retry attempt ${attempt}/${MAX_STARTUP_ATTEMPTS}..."
      tmux kill-session -t "${STAGE_SESSION}" 2>/dev/null || true
      sleep 2
      "${SCRIPT_DIR}/cleanup_simulation.sh"
      sleep 1
    fi

    # Launch simulation
    tmux kill-session -t "${STAGE_SESSION}" 2>/dev/null || true
    SIM_LAUNCH_ARGS="headless:=${HEADLESS} use_composition:=False params_file:=${PARAMS_FILE}"
    if [[ -n "${WORLD}" ]]; then
      SIM_LAUNCH_ARGS="${SIM_LAUNCH_ARGS} world:=${WORLD}"
    fi

    tmux new-session -d -s "${STAGE_SESSION}" \
      "bash -c 'source /opt/ros/jazzy/setup.bash && source ${REPO_DIR}/install/setup.bash && \
        ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py \
          ${SIM_LAUNCH_ARGS} 2>&1 | tee ${SIM_LOG}'"

    echo "[${SCRIPT_NAME}] Simulation starting in tmux session '${STAGE_SESSION}'..."
    echo "[${SCRIPT_NAME}] Waiting ${WAIT_TIME}s for Nav2 lifecycle startup (attempt ${attempt})..."
    sleep 30  # Give initial DDS discovery window

    if "${SCRIPT_DIR}/wait_for_nav2_active.sh" --timeout "${WAIT_TIME}"; then
      NAV2_READY=true
      break
    fi

    echo "[${SCRIPT_NAME}] Nav2 not ready on attempt ${attempt}."
  done

  if ! ${NAV2_READY}; then
    echo "[${SCRIPT_NAME}] ERROR: Nav2 not ready after ${MAX_STARTUP_ATTEMPTS} attempts." >&2
    tail -40 "${SIM_LOG}" >&2
    RUN_STATUS="STARTUP_FAILED"
    PIPELINE_FAILED=true
  fi

  if ! ${PIPELINE_FAILED}; then
    # Confirm nodes explicitly
    echo "[${SCRIPT_NAME}] Nav2 lifecycle nodes confirmed active:"
    for node in /planner_server /controller_server /bt_navigator /behavior_server; do
      state="$(ros2 lifecycle get "${node}" 2>/dev/null || echo "unknown")"
      echo "  ${node}: ${state}"
    done

    echo "[${SCRIPT_NAME}] Nav2 is ready."
    echo "[${SCRIPT_NAME}] Waiting 15s for DDS action server discovery..."
    sleep 15

    # ══════════════════════════════════════════════════════════════════════
    # Step 4: Launch frontier explorer
    # ══════════════════════════════════════════════════════════════════════

    echo "[${SCRIPT_NAME}] Step 4/6: Launching tunnel_frontier_explorer..."

    EXPLORER_START_EPOCH="$(date +%s)"

    tmux kill-session -t "${EXPLORER_SESSION}" 2>/dev/null || true
    tmux new-session -d -s "${EXPLORER_SESSION}" \
      "bash -c 'source /opt/ros/jazzy/setup.bash && source ${REPO_DIR}/install/setup.bash && \
        stdbuf -oL -eL ros2 launch tunnel_frontier_explorer frontier_explorer.launch.py \
          params_file:=${EXPLORER_PARAMS_FILE} \
        2>&1 | tee ${EXPLORER_LOG}'"

    sleep 5

    # Verify explorer started
    if ! tmux has-session -t "${EXPLORER_SESSION}" 2>/dev/null; then
      echo "[${SCRIPT_NAME}] ERROR: Frontier explorer failed to start." >&2
      RUN_STATUS="STARTUP_FAILED"
      PIPELINE_FAILED=true
    else
      echo "[${SCRIPT_NAME}] Frontier explorer started (tmux session '${EXPLORER_SESSION}')."
    fi
  fi

  if ! ${PIPELINE_FAILED}; then
    # ══════════════════════════════════════════════════════════════════════
    # Step 5: Record rosbag + CPU/RSS samples + progress
    # ══════════════════════════════════════════════════════════════════════

    echo "[${SCRIPT_NAME}] Step 5/6: Recording rosbag for ${DURATION}s..."

    # Start rosbag recording (no online compression to avoid CPU interference)
    ros2 bag record \
      /map /scan /tf /tf_static /odom /cmd_vel /clock \
      /local_costmap/costmap_raw /global_costmap/costmap_raw \
      /local_costmap/published_footprint \
      /tunnel_frontier_explorer/frontier_markers \
      -o "${BAG_DIR}" \
      --max-cache-size 104857600 \
      > /dev/null 2>&1 &
    BAG_PID=$!

    # Start CPU/RSS sampling
    sample_cpu_ram "${CPU_LOG}" &
    CPU_SAMPLE_PID=$!

    echo "[${SCRIPT_NAME}] Bag PID: ${BAG_PID}"
    echo "[${SCRIPT_NAME}] CPU/RSS logging to ${CPU_LOG}"
    echo "[${SCRIPT_NAME}] Benchmark running... (${DURATION}s)"

    # Progress indicator with crash detection, early-stop-on-completed,
    # progress tracking for STALLED detection
    LOOP_START_EPOCH="$(date +%s)"
    while true; do
      now="$(date +%s)"
      elapsed=$((now - LOOP_START_EPOCH))

      # Check max duration
      if [ "${elapsed}" -ge "${DURATION}" ]; then
        echo "[${SCRIPT_NAME}] Max duration ${DURATION}s reached."
        break
      fi

      # Progress print every 30 s
      if [ $((elapsed % 30)) -eq 0 ] || [ "${elapsed}" -eq 0 ]; then
        remaining=$((DURATION - elapsed))
        echo "[${SCRIPT_NAME}] ${elapsed}s elapsed, ${remaining}s remaining..."
      fi

      # Crash check
      if ! tmux has-session -t "${EXPLORER_SESSION}" 2>/dev/null; then
        echo "[${SCRIPT_NAME}] WARNING: Frontier explorer died prematurely." >&2
        EXPLORER_CRASHED=true
        break
      fi

      # Explorer-level STALLED check (recovery probes exhausted)
      if grep -q "Exploration stalled" "${EXPLORER_LOG}" 2>/dev/null; then
        echo "[${SCRIPT_NAME}] Explorer declared STALLED — ending run"
        STALL_DETECTED=true
        STALLED_AFTER_SECONDS="${elapsed}"
        break
      fi

      # Completion check (only when --stop-on-completed true)
      if [ "${STOP_ON_COMPLETED}" = "true" ]; then
        if [ "${COMPLETION_DETECTED}" = "false" ]; then
          # First detection of the COMPLETED state (INFO-level message in explorer log)
          if grep -q "exploration complete" "${EXPLORER_LOG}" 2>/dev/null; then
            COMPLETION_DETECTED=true
            COMPLETION_AT_SEC="${elapsed}"
            COMPLETION_CANDIDATE_EPOCH="${now}"
            COMPLETION_GOAL_COUNT="$(grep -c "Goal:" "${EXPLORER_LOG}" 2>/dev/null || echo 0)"
            echo "[${SCRIPT_NAME}] Exploration COMPLETED at ~${elapsed}s (${COMPLETION_GOAL_COUNT} goals)"
            echo "[${SCRIPT_NAME}] Grace period: ${COMPLETED_GRACE_SECONDS}s..."
          fi
        fi

        if [ "${COMPLETION_DETECTED}" = "true" ]; then
          # Once "exploration complete" is logged, treat it as final.
          # Late map messages may cause the explorer to briefly re-enter
          # IDLE, but that's transient — it will re-complete within a few
          # cycles.  Re-counting Goal: lines is fragile (old lines in the
          # growing log can match), so we no longer reset on new goals.
          grace_elapsed=$((elapsed - COMPLETION_AT_SEC))
          remaining=$((DURATION - elapsed))
          if [ "${grace_elapsed}" -ge "${COMPLETED_GRACE_SECONDS}" ] || \
             [ "${remaining}" -lt "${COMPLETED_GRACE_SECONDS}" ]; then
            STABLE_COMPLETION_DETECTED=true
            echo "[${SCRIPT_NAME}] COMPLETED stable for ${grace_elapsed}s — ending run early"
            break
          fi
        fi
      fi

      # ── Progress tracking for STALLED detection ──────────────────────
      if [ -f "${EXPLORER_LOG}" ] && [ "${INJECTION_ACTIVE}" = "false" ]; then
        LOG_LINES_TOTAL=$(wc -l < "${EXPLORER_LOG}" 2>/dev/null || echo 0)
        if [ "${LOG_LINES_TOTAL}" -gt "${LOG_LINES_CHECKED}" ]; then
          NEW_LOG_CONTENT=$(tail -n +$((LOG_LINES_CHECKED + 1)) "${EXPLORER_LOG}" 2>/dev/null || true)
          if echo "${NEW_LOG_CONTENT}" | grep -q "Navigation to goal succeeded"; then
            LAST_PROGRESS_EPOCH="${now}"
            LAST_PROGRESS_EVENT="navigation_succeeded"
          elif echo "${NEW_LOG_CONTENT}" | grep -q "Navigation aborted"; then
            LAST_PROGRESS_EPOCH="${now}"
            LAST_PROGRESS_EVENT="navigation_aborted"
          elif echo "${NEW_LOG_CONTENT}" | grep -q "Goal timed out after"; then
            LAST_PROGRESS_EPOCH="${now}"
            LAST_PROGRESS_EVENT="navigation_timed_out"
          elif echo "${NEW_LOG_CONTENT}" | grep -q "Goal accepted by Nav2 server"; then
            LAST_PROGRESS_EPOCH="${now}"
            LAST_PROGRESS_EVENT="goal_dispatched"
          fi
        fi
        LOG_LINES_CHECKED="${LOG_LINES_TOTAL}"
      fi

      # ── INJECTED STALL activation ────────────────────────────────────
      if [ "${INJECTED_STALL_ENABLED}" = "true" ] && \
         [ "${INJECTION_ACTIVE}" = "false" ] && \
         [ "${elapsed}" -ge "${INJECT_STALL_AFTER_SECONDS}" ]; then
        INJECTION_ACTIVE=true
        INJECTION_ACTIVATED_EPOCH="${now}"
        LAST_PROGRESS_EPOCH="${now}"
        LAST_PROGRESS_EVENT="injected_stall_activated"
        echo "[${SCRIPT_NAME}] INJECTED STALL activated at ~${elapsed}s — now waiting ${STALL_TIMEOUT_SECONDS}s for STALLED detection"
      fi

      # ── STALLED detection ────────────────────────────────────────────
      if [ "${INJECTION_ACTIVE}" = "true" ]; then
        # Test mode: STALLED takes priority over COMPLETED
        stall_elapsed=$((now - LAST_PROGRESS_EPOCH))
        if [ "${stall_elapsed}" -ge "${STALL_TIMEOUT_SECONDS}" ]; then
          echo "[${SCRIPT_NAME}] STALLED: ${stall_elapsed}s without progress (injected) — ending run"
          STALL_DETECTED=true
          STALLED_AFTER_SECONDS="${elapsed}"
          break
        fi
      elif [ "${STOP_ON_COMPLETED}" = "true" ] && [ "${COMPLETION_DETECTED}" = "false" ]; then
        if [ "${LAST_PROGRESS_EPOCH}" -gt 0 ]; then
          stall_elapsed=$((now - LAST_PROGRESS_EPOCH))
          if [ "${stall_elapsed}" -ge "${STALL_TIMEOUT_SECONDS}" ]; then
            echo "[${SCRIPT_NAME}] STALLED: no progress for ${stall_elapsed}s (event: ${LAST_PROGRESS_EVENT}) — ending run"
            STALL_DETECTED=true
            STALLED_AFTER_SECONDS="${elapsed}"
            break
          fi
        fi
      fi

      sleep 5
    done

    echo "[${SCRIPT_NAME}] Benchmark period complete."
  fi

  # Determine run status (priority: COMPLETED > CRASHED > STALLED > TIMEOUT)
  if [ "${RUN_STATUS}" = "STARTUP_FAILED" ]; then
    :  # already set
  elif [ "${STABLE_COMPLETION_DETECTED}" = "true" ]; then
    RUN_STATUS="COMPLETED"
  elif ${EXPLORER_CRASHED}; then
    RUN_STATUS="CRASHED"
  elif ${STALL_DETECTED}; then
    RUN_STATUS="STALLED"
  else
    RUN_STATUS="TIMEOUT"
  fi
  echo "[${SCRIPT_NAME}] Run status: ${RUN_STATUS}"

  # ── Stop recording ─────────────────────────────────────────────────────

  kill "${CPU_SAMPLE_PID:-}" 2>/dev/null || true
  wait "${CPU_SAMPLE_PID:-}" 2>/dev/null || true

  kill "${BAG_PID:-}" 2>/dev/null || true
  wait "${BAG_PID:-}" 2>/dev/null || true
  echo "[${SCRIPT_NAME}] Rosbag saved to ${BAG_DIR}"

  EXPLORER_END_EPOCH="$(date +%s)"

  if ! ${PIPELINE_FAILED}; then
    # ══════════════════════════════════════════════════════════════════════
    # Final Nav2 lifecycle check
    # ══════════════════════════════════════════════════════════════════════

    echo "[${SCRIPT_NAME}] Final Nav2 lifecycle check:"
    NAV2_ALL_ACTIVE=true
    for node in /planner_server /controller_server /bt_navigator /behavior_server; do
      state="$(ros2 lifecycle get "${node}" 2>/dev/null || echo "unreachable")"
      echo "  ${node}: ${state}"
      case "${state}" in
        active*) ;;
        *) NAV2_ALL_ACTIVE=false ;;
      esac
    done

    # ══════════════════════════════════════════════════════════════════════
    # Step 6: Extract metrics
    # ══════════════════════════════════════════════════════════════════════

    echo "[${SCRIPT_NAME}] Step 6/6: Extracting metrics..."

    # ── Explorer wall-clock duration ──────────────────────────────────────
    EXPLORER_DURATION=$((EXPLORER_END_EPOCH - EXPLORER_START_EPOCH))
    EXPLORER_DURATION_MIN="$(echo "scale=1; ${EXPLORER_DURATION} / 60" | bc)"

    # ── Navigation goals (exact log patterns from frontier_explorer_node.cpp) ──
    #   "Goal: (x, y) [N cells, centroid=... m, goal=... m]"
    #   "Navigation to goal succeeded"
    #   "Goal timed out after N.N s — cancelling"
    #   "Navigation aborted"
    GOAL_LINES="$(grep -cF "Goal:" "${EXPLORER_LOG}" 2>/dev/null || true)"
    GOAL_LINES="${GOAL_LINES:-0}"
    SUCCESS_LINES="$(grep -cF "Navigation to goal succeeded" "${EXPLORER_LOG}" 2>/dev/null || true)"
    SUCCESS_LINES="${SUCCESS_LINES:-0}"
    TIMEOUT_LINES="$(grep -cF "Goal timed out after" "${EXPLORER_LOG}" 2>/dev/null || true)"
    TIMEOUT_LINES="${TIMEOUT_LINES:-0}"
    ABORTED_LINES="$(grep -cF "Navigation aborted" "${EXPLORER_LOG}" 2>/dev/null || true)"
    ABORTED_LINES="${ABORTED_LINES:-0}"

    # ── Stage 3D recovery probe metrics ────────────────────────────────────
    RECOVERY_PROBE_DISPATCHED="$(grep -cF "Recovery probe goal:" "${EXPLORER_LOG}" 2>/dev/null || true)"
    RECOVERY_PROBE_DISPATCHED="${RECOVERY_PROBE_DISPATCHED:-0}"
    RECOVERY_PROBE_SUCCEEDED="$(grep -cF "Recovery probe succeeded" "${EXPLORER_LOG}" 2>/dev/null || true)"
    RECOVERY_PROBE_SUCCEEDED="${RECOVERY_PROBE_SUCCEEDED:-0}"
    LOOP_DETECTED_COUNT="$(grep -cF "Local loop detected" "${EXPLORER_LOG}" 2>/dev/null || true)"
    LOOP_DETECTED_COUNT="${LOOP_DETECTED_COUNT:-0}"
    RECOVERY_PROBE_FAILED=0
    if [ "${RECOVERY_PROBE_DISPATCHED}" -gt 0 ]; then
      RECOVERY_PROBE_FAILED=$((RECOVERY_PROBE_DISPATCHED - RECOVERY_PROBE_SUCCEEDED))
    fi

    # ── Unique goals via spatial binning ─────────────────────────────────
    # Bin each goal to BIN_SIZE grid, count unique bins
    if [ -s "${EXPLORER_LOG}" ]; then
      UNIQUE_BINS="$(grep -oP 'Goal: \(\K[^)]+' "${EXPLORER_LOG}" 2>/dev/null | \
        awk -v bin="${BIN_SIZE}" '{
          split($0, a, ",");
          x = a[1] + 0;
          y = a[2] + 0;
          bin_x = sprintf("%.0f", x / bin);
          bin_y = sprintf("%.0f", y / bin);
          print bin_x, bin_y;
        }' | sort -u | wc -l || true)"
      TOTAL_GOALS="$(grep -oP 'Goal: \(\K[^)]+' "${EXPLORER_LOG}" 2>/dev/null | wc -l || true)"
    else
      UNIQUE_BINS=0
      TOTAL_GOALS=0
    fi
    UNIQUE_BINS="${UNIQUE_BINS:-0}"
    TOTAL_GOALS="${TOTAL_GOALS:-0}"

    REPEATED_GOALS=0
    if [ "${TOTAL_GOALS}" -gt 0 ] && [ "${UNIQUE_BINS}" -gt 0 ]; then
      REPEATED_GOALS=$((TOTAL_GOALS - UNIQUE_BINS))
    fi

    # ── Revisit rate ──────────────────────────────────────────────────────
    if [ "${TOTAL_GOALS}" -gt 1 ]; then
      REVISIT_RATE="$(echo "scale=1; 100 * ${REPEATED_GOALS} / ${TOTAL_GOALS}" | bc)"
    else
      REVISIT_RATE="0.0"
    fi

    # ── Goal reachability rate ────────────────────────────────────────────
    if [ "${GOAL_LINES}" -gt 0 ]; then
      REACH_RATE="$(echo "scale=1; 100 * ${SUCCESS_LINES} / ${GOAL_LINES}" | bc)"
    else
      REACH_RATE="0.0"
    fi

    # ── Goal distance distribution ────────────────────────────────────────
    MIN_GOAL_DIST="$(grep -oP 'goal=\K[0-9.]+' "${EXPLORER_LOG}" 2>/dev/null | sort -n | head -1 || echo "N/A")"
    MAX_GOAL_DIST="$(grep -oP 'goal=\K[0-9.]+' "${EXPLORER_LOG}" 2>/dev/null | sort -n | tail -1 || echo "N/A")"

    # ── Filter activation (centroid < 0.5m AND goal >= 0.5m) ─────────────
    FILTER_COUNT=0
    while IFS= read -r line; do
      centroid="$(echo "$line" | grep -oP 'centroid=\K[0-9.]+')"
      goal="$(echo "$line" | grep -oP 'goal=\K[0-9.]+')"
      if [ -n "$centroid" ] && [ -n "$goal" ]; then
        if [ "$(echo "${centroid} < 0.5" | bc -l 2>/dev/null)" = "1" ] && \
           [ "$(echo "${goal} >= 0.5" | bc -l 2>/dev/null)" = "1" ]; then
          FILTER_COUNT=$((FILTER_COUNT + 1))
        fi
      fi
    done < <(grep -oP 'Goal: \([^)]+\) \[\d+ cells, centroid=[0-9.]+ m, goal=[0-9.]+ m]' "${EXPLORER_LOG}" 2>/dev/null || true)

    # ── Cluster cell count range ──────────────────────────────────────────
    MIN_CELLS="$(grep -oP '\[\K[0-9]+(?= cells)' "${EXPLORER_LOG}" 2>/dev/null | sort -n | head -1 || echo "N/A")"
    MAX_CELLS="$(grep -oP '\[\K[0-9]+(?= cells)' "${EXPLORER_LOG}" 2>/dev/null | sort -n | tail -1 || echo "N/A")"

    # ── Node crashes ──────────────────────────────────────────────────────
    if ${EXPLORER_CRASHED}; then
      NODE_CRASHES=1
    else
      NODE_CRASHES=0
    fi

    # ── Rosbag existence ──────────────────────────────────────────────────
    BAG_EXISTS="false"
    BAG_METADATA="${BAG_DIR}/metadata.yaml"
    if [ -d "${BAG_DIR}" ] && [ -f "${BAG_METADATA}" ]; then
      BAG_EXISTS="true"
    fi
  fi

  # ══════════════════════════════════════════════════════════════════════
  # Write results
  # ══════════════════════════════════════════════════════════════════════

  cat >> "${RESULTS_FILE}" << RESULTS

## Run ${RUN_ID} Results

### Summary

| Metric | Value |
|--------|-------|
| Run status | ${RUN_STATUS} |
| Completion detected | ${COMPLETION_DETECTED} |
| Completion time | ${COMPLETION_AT_SEC} s |
| Stable completion | ${STABLE_COMPLETION_DETECTED} |
| Completion reset count | ${COMPLETION_RESET_COUNT} |
| Completion reset reason | ${COMPLETION_RESET_REASON} |
| Injected stall enabled | ${INJECTED_STALL_ENABLED} |
| Injected stall after (s) | ${INJECT_STALL_AFTER_SECONDS} |
| Injection activated epoch | ${INJECTION_ACTIVATED_EPOCH} |
| Stall detected | ${STALL_DETECTED} |
| Stall timeout seconds | ${STALL_TIMEOUT_SECONDS} |
| Last progress event | ${LAST_PROGRESS_EVENT} |
| Stalled after seconds | ${STALLED_AFTER_SECONDS} |
| Completed grace seconds | ${COMPLETED_GRACE_SECONDS} |
| Max duration (configured) | ${DURATION} s |
| Explorer active duration | ${EXPLORER_DURATION_MIN:-0} min |
| Nav2 startup attempts | ${STARTUP_ATTEMPTS} |
| Nav2 startup wait per attempt | ${WAIT_TIME} s |

### Startup Reliability

| Metric | Value |
|--------|-------|
| Nav2 startup attempts | ${STARTUP_ATTEMPTS} |
| Nav2 end-of-run all active | ${NAV2_ALL_ACTIVE:-false} |

### Exploration Efficiency

| Metric | Value |
|--------|-------|
| Goals dispatched | ${GOAL_LINES:-0} |
| Goals succeeded | ${SUCCESS_LINES:-0} |
| Goals timed out | ${TIMEOUT_LINES:-0} |
| Goals aborted | ${ABORTED_LINES:-0} |
| Navigation goal success rate | ${REACH_RATE:-0.0}% |
| Unique goal bins (${BIN_SIZE} m grid) | ${UNIQUE_BINS:-0} |
| Repeated goals | ${REPEATED_GOALS:-0} |
| Revisit rate | ${REVISIT_RATE:-0.0}% |

### Goal Distance

| Metric | Value |
|--------|-------|
| Min goal distance | ${MIN_GOAL_DIST:-N/A} m |
| Max goal distance | ${MAX_GOAL_DIST:-N/A} m |
| FrontierGoalSelector filter activations | ${FILTER_COUNT:-0} |

### Cluster Size

| Metric | Value |
|--------|-------|
| Min cluster cells | ${MIN_CELLS:-N/A} |
| Max cluster cells | ${MAX_CELLS:-N/A} |

### System Stability

| Metric | Value |
|--------|-------|
| Node crashes | ${NODE_CRASHES:-0} |
| Nav2 lifecycle failures (end) | $(${NAV2_ALL_ACTIVE:-false} && echo 0 || echo 1) |

### Stage 3D Recovery Probe

| Metric | Value |
|--------|-------|
| Loop detected count | ${LOOP_DETECTED_COUNT:-0} |
| Recovery probes dispatched | ${RECOVERY_PROBE_DISPATCHED:-0} |
| Recovery probes succeeded | ${RECOVERY_PROBE_SUCCEEDED:-0} |
| Recovery probes failed | ${RECOVERY_PROBE_FAILED:-0} |

### Rosbag

| Metric | Value |
|--------|-------|
| Bag recorded | ${BAG_EXISTS:-false} |
| Bag path | ${BAG_DIR} |
| Topics | /map /scan /tf /tf_static /odom /cmd_vel /clock costmaps markers |

### Reproducibility

- Tag: \`stage1c-nearest-frontier-pass\`
- Commit: $(git -C "${REPO_DIR}" rev-parse --short HEAD 2>/dev/null || echo "N/A")
- Params: \`nav2_params_rotation_shim.yaml\`
- Explorer config: \`frontier_explorer_params.yaml\`

RESULTS

  # ── Write structured JSON results ──────────────────────────────────
  JSON_FILE="${ATTEMPT_DIR}/benchmark_results.json"
  # Compute params file hash and extract selection strategy.
  if [ -f "${EXPLORER_PARAMS_FILE}" ]; then
    EXPLORER_PARAMS_SHA256=$(sha256sum "${EXPLORER_PARAMS_FILE}" | cut -d' ' -f1)
    EXPLORER_SELECTION_STRATEGY=$(grep -oP 'selection_strategy:\s*\K\S+' "${EXPLORER_PARAMS_FILE}" 2>/dev/null || echo "nearest")
  else
    EXPLORER_PARAMS_SHA256="file_not_found"
    EXPLORER_SELECTION_STRATEGY="unknown"
  fi

  # ── terminal_state and completion_reason ─────────────────────────────
  if [ "${RUN_STATUS}" = "COMPLETED" ]; then
    TERMINAL_STATE="COMPLETED"
    COMPLETION_REASON="no_frontiers_for_10_cycles"
  elif grep -q "Exploration stalled" "${EXPLORER_LOG}" 2>/dev/null; then
    TERMINAL_STATE="STALLED"
    COMPLETION_REASON="local_loop_no_safe_recovery_probe"
  elif [ "${RUN_STATUS}" = "STALLED" ]; then
    TERMINAL_STATE="STALLED"
    COMPLETION_REASON="no_progress"
  elif [ "${RUN_STATUS}" = "CRASHED" ]; then
    TERMINAL_STATE="CRASHED"
    COMPLETION_REASON=""
  elif [ "${RUN_STATUS}" = "STARTUP_FAILED" ]; then
    TERMINAL_STATE="STARTUP_FAILED"
    COMPLETION_REASON=""
  else
    TERMINAL_STATE="TIMEOUT"
    COMPLETION_REASON=""
  fi

  JSON_SCRIPT=$(mktemp /tmp/benchmark_json_XXXXXXXX.py)
  cat > "${JSON_SCRIPT}" << JSONPYEOF
import json
data = {
    'run_status': '${RUN_STATUS:-TIMEOUT}',
    'terminal_state': '${TERMINAL_STATE:-TIMEOUT}',
    'completion_reason': '${COMPLETION_REASON:-}',
    'stable_completion_detected': $( [ "${STABLE_COMPLETION_DETECTED:-false}" = "true" ] && echo True || echo False ),
    'completion_candidate_detected': $( [ "${COMPLETION_DETECTED:-false}" = "true" ] && echo True || echo False ),
    'completion_candidate_time_seconds': ${COMPLETION_AT_SEC:-0},
    'completion_time_seconds': ${COMPLETION_AT_SEC:-0},
    'completion_candidate_epoch': ${COMPLETION_CANDIDATE_EPOCH:-0},
    'completion_reset_count': ${COMPLETION_RESET_COUNT:-0},
    'completion_reset_reason': '${COMPLETION_RESET_REASON:-}',
    'injected_stall_enabled': $( [ "${INJECTED_STALL_ENABLED:-false}" = "true" ] && echo True || echo False ),
    'injected_stall_after_seconds': ${INJECT_STALL_AFTER_SECONDS:-0},
    'injection_activated_epoch': ${INJECTION_ACTIVATED_EPOCH:-0},
    'stall_detected': $( [ "${STALL_DETECTED:-false}" = "true" ] && echo True || echo False ),
    'stall_timeout_seconds': ${STALL_TIMEOUT_SECONDS:-90},
    'stalled_after_seconds': ${STALLED_AFTER_SECONDS:-0},
    'last_progress_event': '${LAST_PROGRESS_EVENT:-none}',
    'goals_dispatched': ${GOAL_LINES:-0},
    'goals_succeeded': ${SUCCESS_LINES:-0},
    'goals_aborted': ${ABORTED_LINES:-0},
    'unique_goal_bins': ${UNIQUE_BINS:-0},
    'revisit_rate': ${REVISIT_RATE:-0.0},
    'navigation_goal_success_rate': ${REACH_RATE:-0.0},
    'loop_detected_count': ${LOOP_DETECTED_COUNT:-0},
    'recovery_probe_dispatched': ${RECOVERY_PROBE_DISPATCHED:-0},
    'recovery_probe_succeeded': ${RECOVERY_PROBE_SUCCEEDED:-0},
    'recovery_probe_failed': ${RECOVERY_PROBE_FAILED:-0},
    'explorer_params_file': '${EXPLORER_PARAMS_FILE}',
    'explorer_params_sha256': '${EXPLORER_PARAMS_SHA256}',
    'selection_strategy': '${EXPLORER_SELECTION_STRATEGY}'
}
with open('${JSON_FILE}', 'w') as f:
    json.dump(data, f, indent=2)
JSONPYEOF
  python3 "${JSON_SCRIPT}" || echo "[${SCRIPT_NAME}] Warning: JSON results not written"
  rm -f "${JSON_SCRIPT}"
  echo "[${SCRIPT_NAME}] Results written to ${RESULTS_FILE}"

  if ! ${PIPELINE_FAILED}; then
    # ── Run Python post-processor for odometry + map coverage ─────────────

    if [ "${BAG_EXISTS}" = "true" ]; then
      echo "[${SCRIPT_NAME}] Running rosbag analysis..."
      python3 "${SCRIPT_DIR}/analyze_rosbag.py" \
        --bag-dir "${BAG_DIR}" \
        --output-dir "${ATTEMPT_DIR}" \
        --map-topic /map \
        --odom-topic /odom 2>&1 | sed "s/^/[analyze_rosbag] /" || \
        echo "[${SCRIPT_NAME}] Warning: rosbag analysis failed (rosbag2_py may not be installed)"
    else
      echo "[${SCRIPT_NAME}] Skipping rosbag analysis (bag not found)"
    fi

    # ── Final summary ─────────────────────────────────────────────────────

    echo ""
    echo "============================================"
    echo " Run ${RUN_ID} complete."
    echo "============================================"
    echo ""
    echo " Status:           ${RUN_STATUS}"
    echo " Goals:            ${GOAL_LINES:-0} total, ${SUCCESS_LINES:-0} succeeded, ${UNIQUE_BINS:-0} unique bins"
    echo " Revisit rate:     ${REVISIT_RATE:-0.0}%"
    echo " Navigation goal success rate:     ${REACH_RATE:-0.0}%"
    echo " Completion time: ${COMPLETION_AT_SEC} s (detected: ${COMPLETION_DETECTED})"
    echo " Filter activations: ${FILTER_COUNT:-0}"
    echo " Startup attempts: ${STARTUP_ATTEMPTS}"
    echo " Node crashes:     ${NODE_CRASHES:-0}"
    echo ""
    echo " Full results: ${RESULTS_FILE}"
    echo " Rosbag:       ${BAG_DIR}"
    echo " Explorer log: ${EXPLORER_LOG}"
    echo " CPU/RSS log:  ${CPU_LOG}"
    echo ""
  fi

  ALL_ATTEMPT_STATUSES="${ALL_ATTEMPT_STATUSES} ${RUN_STATUS}"

  # Decide whether to retry
  case "${RUN_STATUS}" in
    COMPLETED)
      echo "[${SCRIPT_NAME}] COMPLETED — stopping retry loop"
      break
      ;;
    STALLED|STARTUP_FAILED)
      if [ "${ATTEMPT_NUM}" -le "${RUNTIME_RETRIES}" ]; then
        echo "[${SCRIPT_NAME}] ${RUN_STATUS} — will retry (attempt ${ATTEMPT_NUM}/${RUNTIME_RETRIES})"
        # Cleanup before retry
        kill "${CPU_SAMPLE_PID:-}" 2>/dev/null || true
        kill "${BAG_PID:-}" 2>/dev/null || true
        tmux kill-session -t "${EXPLORER_SESSION}" 2>/dev/null || true
        tmux kill-session -t "${STAGE_SESSION}" 2>/dev/null || true
        sleep 2
        "${SCRIPT_DIR}/cleanup_simulation.sh"
        ATTEMPT_NUM=$((ATTEMPT_NUM + 1))
        continue
      fi
      ;;
    *) ;;
  esac
  break
done

# ── Write run summary ──────────────────────────────────────────────────

{
  echo "# Run ${RUN_ID} Summary"
  echo ""
  echo "| Metric | Value |"
  echo "|--------|-------|"
  echo "| Attempts total | ${ATTEMPT_NUM} |"
  echo "| Attempt statuses | ${ALL_ATTEMPT_STATUSES} |"
  echo "| Final status | ${RUN_STATUS} |"
  echo ""
} > "${RUN_DIR}/run_summary.md"

echo "[${SCRIPT_NAME}] Run summary written to ${RUN_DIR}/run_summary.md"

# Exit with appropriate code
case "${RUN_STATUS}" in
  COMPLETED|TIMEOUT) exit 0 ;;
  STALLED|CRASHED)   exit 1 ;;
  STARTUP_FAILED)    exit 1 ;;
  *)                 exit 1 ;;
esac
