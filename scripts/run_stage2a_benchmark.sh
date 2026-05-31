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
#   --headless        Run Gazebo headless (default: true)
#   --help            Show this message
#
# Output structure:
#   <output-dir>/run_<id>/
#     frontier_explorer.log    — Explorer node log
#     simulation.log           — Simulation launch log
#     cpu_ram_usage.csv        — CPU/RSS samples every 10 s
#     benchmark_results.md     — Extracted metrics
#     rosbag_metrics.md        — Odometry + map coverage from bag
#     rosbag_metrics.json      — Structured bag metrics
#     bag/                     — ROS2 bag
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
#   1 — Explorer crashed, Nav2 startup failed, or invalid termination

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

# ── Parse arguments ────────────────────────────────────────────────────

while [ $# -gt 0 ]; do
  case "$1" in
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --duration)   DURATION="$2";   shift 2 ;;
    --run-id)     RUN_ID="$2";     shift 2 ;;
    --wait-time)  WAIT_TIME="$2";  shift 2 ;;
    --stop-on-completed) STOP_ON_COMPLETED="$2"; shift 2 ;;
    --completed-grace-seconds) COMPLETED_GRACE_SECONDS="$2"; shift 2 ;;
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

RUN_DIR="${OUTPUT_DIR}/run_${RUN_ID}"
SIM_LOG="${RUN_DIR}/simulation.log"
EXPLORER_LOG="${RUN_DIR}/frontier_explorer.log"
CPU_LOG="${RUN_DIR}/cpu_ram_usage.csv"
RESULTS_FILE="${RUN_DIR}/benchmark_results.md"
BAG_DIR="${RUN_DIR}/bag"
PARAMS_FILE="${REPO_DIR}/install/tunnel_explorer_bringup/share/tunnel_explorer_bringup/config/nav2_params_rotation_shim.yaml"
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

# ── Run header ─────────────────────────────────────────────────────────

{
  echo "============================================"
  echo " Stage 2A Baseline Benchmark — Run ${RUN_ID}"
  echo "============================================"
  echo ""
  echo " Output:  ${RUN_DIR}"
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
  tmux new-session -d -s "${STAGE_SESSION}" \
    "bash -c 'source /opt/ros/jazzy/setup.bash && source ${REPO_DIR}/install/setup.bash && \
      ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py \
        headless:=${HEADLESS} use_composition:=False \
        params_file:=${PARAMS_FILE} 2>&1 | tee ${SIM_LOG}'"

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
  exit 1
fi

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
    2>&1 | tee ${EXPLORER_LOG}'"

sleep 5

# Verify explorer started
if ! tmux has-session -t "${EXPLORER_SESSION}" 2>/dev/null; then
  echo "[${SCRIPT_NAME}] ERROR: Frontier explorer failed to start." >&2
  exit 2
fi

echo "[${SCRIPT_NAME}] Frontier explorer started (tmux session '${EXPLORER_SESSION}')."

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

# Progress indicator with crash detection and early-stop-on-completed
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

  # Completion check (only when --stop-on-completed true)
  if [ "${STOP_ON_COMPLETED}" = "true" ]; then
    if [ "${COMPLETION_DETECTED}" = "false" ]; then
      # First detection of the COMPLETED state (INFO-level message in explorer log)
      if grep -q "exploration complete" "${EXPLORER_LOG}" 2>/dev/null; then
        COMPLETION_DETECTED=true
        COMPLETION_AT_SEC="${elapsed}"
        COMPLETION_GOAL_COUNT="$(grep -c "Goal:" "${EXPLORER_LOG}" 2>/dev/null || echo 0)"
        echo "[${SCRIPT_NAME}] Exploration COMPLETED at ~${elapsed}s (${COMPLETION_GOAL_COUNT} goals)"
        echo "[${SCRIPT_NAME}] Grace period: ${COMPLETED_GRACE_SECONDS}s..."
      fi
    fi

    if [ "${COMPLETION_DETECTED}" = "true" ]; then
      # Check if the robot has resumed exploration
      if grep -q "New map received while COMPLETED" "${EXPLORER_LOG}" 2>/dev/null; then
        echo "[${SCRIPT_NAME}] Robot resumed exploration while COMPLETED — resetting grace timer"
        COMPLETION_DETECTED=false
        COMPLETION_GOAL_COUNT=-1
      else
        current_goals="$(grep -c "Goal:" "${EXPLORER_LOG}" 2>/dev/null || echo 0)"
        if [ "${COMPLETION_GOAL_COUNT}" -ge 0 ] && \
           [ "${current_goals}" -gt "${COMPLETION_GOAL_COUNT}" ]; then
          echo "[${SCRIPT_NAME}] New goals (${current_goals} > ${COMPLETION_GOAL_COUNT}) — exploration resumed"
          COMPLETION_DETECTED=false
          COMPLETION_GOAL_COUNT=-1
          COMPLETION_AT_SEC=0
        fi
      fi

      # If still COMPLETED, check grace period
      if [ "${COMPLETION_DETECTED}" = "true" ]; then
        grace_elapsed=$((elapsed - COMPLETION_AT_SEC))
        if [ "${grace_elapsed}" -ge "${COMPLETED_GRACE_SECONDS}" ]; then
          echo "[${SCRIPT_NAME}] COMPLETED stable for ${grace_elapsed}s — ending run early"
          break
        fi
      fi
    fi
  fi

  sleep 5
done

echo "[${SCRIPT_NAME}] Benchmark period complete."

# Determine run status (priority: COMPLETED > CRASHED > TIMEOUT)
if [ "${COMPLETION_DETECTED}" = "true" ]; then
  RUN_STATUS="COMPLETED"
elif ${EXPLORER_CRASHED}; then
  RUN_STATUS="CRASHED"
else
  RUN_STATUS="TIMEOUT"
fi
echo "[${SCRIPT_NAME}] Run status: ${RUN_STATUS}"

# ── Stop recording ─────────────────────────────────────────────────────

kill "${CPU_SAMPLE_PID}" 2>/dev/null || true
wait "${CPU_SAMPLE_PID}" 2>/dev/null || true

kill "${BAG_PID}" 2>/dev/null || true
wait "${BAG_PID}" 2>/dev/null || true
echo "[${SCRIPT_NAME}] Rosbag saved to ${BAG_DIR}"

EXPLORER_END_EPOCH="$(date +%s)"

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
| Completed grace seconds | ${COMPLETED_GRACE_SECONDS} |
| Max duration (configured) | ${DURATION} s |
| Explorer active duration | ${EXPLORER_DURATION_MIN} min |
| Nav2 startup attempts | ${STARTUP_ATTEMPTS} |
| Nav2 startup wait per attempt | ${WAIT_TIME} s |

### Startup Reliability

| Metric | Value |
|--------|-------|
| Nav2 startup attempts | ${STARTUP_ATTEMPTS} |
| Nav2 end-of-run all active | ${NAV2_ALL_ACTIVE} |

### Exploration Efficiency

| Metric | Value |
|--------|-------|
| Goals dispatched | ${GOAL_LINES} |
| Goals succeeded | ${SUCCESS_LINES} |
| Goals timed out | ${TIMEOUT_LINES} |
| Goals aborted | ${ABORTED_LINES} |
| Goal reachability rate | ${REACH_RATE}% |
| Unique goal bins (${BIN_SIZE} m grid) | ${UNIQUE_BINS} |
| Repeated goals | ${REPEATED_GOALS} |
| Revisit rate | ${REVISIT_RATE}% |

### Goal Distance

| Metric | Value |
|--------|-------|
| Min goal distance | ${MIN_GOAL_DIST} m |
| Max goal distance | ${MAX_GOAL_DIST} m |
| FrontierGoalSelector filter activations | ${FILTER_COUNT} |

### Cluster Size

| Metric | Value |
|--------|-------|
| Min cluster cells | ${MIN_CELLS} |
| Max cluster cells | ${MAX_CELLS} |

### System Stability

| Metric | Value |
|--------|-------|
| Node crashes | ${NODE_CRASHES} |
| Nav2 lifecycle failures (end) | $(${NAV2_ALL_ACTIVE} && echo 0 || echo 1) |

### Rosbag

| Metric | Value |
|--------|-------|
| Bag recorded | ${BAG_EXISTS} |
| Bag path | ${BAG_DIR} |
| Topics | /map /scan /tf /tf_static /odom /cmd_vel /clock costmaps markers |

### Reproducibility

- Tag: \`stage1c-nearest-frontier-pass\`
- Commit: $(git -C "${REPO_DIR}" rev-parse --short HEAD 2>/dev/null || echo "N/A")
- Params: \`nav2_params_rotation_shim.yaml\`
- Explorer config: \`frontier_explorer_params.yaml\`

RESULTS

echo "[${SCRIPT_NAME}] Results written to ${RESULTS_FILE}"

# ── Run Python post-processor for odometry + map coverage ─────────────

if [ "${BAG_EXISTS}" = "true" ]; then
  echo "[${SCRIPT_NAME}] Running rosbag analysis..."
  python3 "${SCRIPT_DIR}/analyze_rosbag.py" \
    --bag-dir "${BAG_DIR}" \
    --output-dir "${RUN_DIR}" \
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
echo " Goals:            ${GOAL_LINES} total, ${SUCCESS_LINES} succeeded, ${UNIQUE_BINS} unique bins"
echo " Revisit rate:     ${REVISIT_RATE}%"
echo " Reachability:     ${REACH_RATE}%"
echo " Completion time: ${COMPLETION_AT_SEC} s (detected: ${COMPLETION_DETECTED})"
echo " Filter activations: ${FILTER_COUNT}"
echo " Startup attempts: ${STARTUP_ATTEMPTS}"
echo " Node crashes:     ${NODE_CRASHES}"
echo ""
echo " Full results: ${RESULTS_FILE}"
echo " Rosbag:       ${BAG_DIR}"
echo " Explorer log: ${EXPLORER_LOG}"
echo " CPU/RSS log:  ${CPU_LOG}"
echo ""

# Exit with appropriate code
case "${RUN_STATUS}" in
  COMPLETED|TIMEOUT) exit 0 ;;
  CRASHED)           exit 1 ;;
  STARTUP_FAILED)    exit 1 ;;
  *)                 exit 1 ;;
esac
