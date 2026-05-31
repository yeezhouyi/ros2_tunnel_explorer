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
# run_stage1c_smoke.sh
#
# Stage 1C Frontier Explorer simulation smoke test.  Use after launching
# the Stage 0 simulation stack with nav2_params_rotation_shim.yaml.
#
# Usage:
#   ./scripts/run_stage1c_smoke.sh [output-dir]
#
#   output-dir defaults to ~/stage1c_smoke if not specified.
#
# This script:
#   1. Waits for Nav2 lifecycle nodes to become active
#   2. Launches the tunnel_frontier_explorer node
#   3. Prints the recommended rosbag record command
#
# Prerequisites:
#   - Stage 0 simulation running with nav2_params_rotation_shim.yaml
#   - ./scripts/cleanup_simulation.sh has been run before launching
#   - ROS2 Jazzy and workspace are built (colcon build)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

# ── Configurable defaults ────────────────────────────────────────────

OUTPUT_DIR="${1:-${HOME}/stage1c_smoke}"

# ── Source ROS2 and workspace ───────────────────────────────────────

if ! command -v ros2 &> /dev/null; then
  echo "[${SCRIPT_NAME}] Sourcing /opt/ros/jazzy/setup.bash..."
  source /opt/ros/jazzy/setup.bash
fi
if [ -z "${COLCON_PREFIX_PATH:-}" ]; then
  echo "[${SCRIPT_NAME}] Sourcing workspace install/setup.bash..."
  source "${REPO_DIR}/install/setup.bash"
fi

mkdir -p "${OUTPUT_DIR}"

# ── Step 1: Wait for Nav2 lifecycle nodes ───────────────────────────

echo ""
echo "============================================"
echo " Stage 1C Smoke Test"
echo "============================================"
echo ""
echo " Output directory: ${OUTPUT_DIR}"
echo ""

echo "[${SCRIPT_NAME}] Step 1/2: Waiting for Nav2 lifecycle nodes..."
if ! "${SCRIPT_DIR}/wait_for_nav2_active.sh"; then
  echo "[${SCRIPT_NAME}] ERROR: Nav2 not ready." >&2
  echo "" >&2
  echo "  Ensure simulation is running with RotationShim config:" >&2
  echo "  ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py \\" >&2
  echo "    headless:=True \\" >&2
  echo "    use_composition:=False \\" >&2
  echo '    params_file:="$(ros2 pkg prefix --share tunnel_explorer_bringup)/config/nav2_params_rotation_shim.yaml"' >&2
  exit 1
fi

# ── Step 2: Launch frontier explorer ────────────────────────────────

LOG_FILE="${OUTPUT_DIR}/frontier_explorer.log"
echo "[${SCRIPT_NAME}] Step 2/2: Launching tunnel_frontier_explorer..."
echo "[${SCRIPT_NAME}] Log: ${LOG_FILE}"

ros2 launch tunnel_frontier_explorer frontier_explorer.launch.py \
  2>&1 | tee "${LOG_FILE}" &

EXPLORER_PID=$!
# shellcheck disable=SC2064
trap "kill ${EXPLORER_PID} 2>/dev/null || true" EXIT INT TERM

# Give the node a moment to initialise.
sleep 3

echo ""
echo "============================================"
echo " Stage 1C Running"
echo "============================================"
echo ""
echo " Explorer PID:   ${EXPLORER_PID}"
echo " Explorer log:   ${LOG_FILE}"
echo ""
echo " Recommended rosbag record command"
echo " (run in a separate terminal):"
echo ""
echo "  ros2 bag record \\"
echo "    /map \\"
echo "    /scan \\"
echo "    /tf \\"
echo "    /tf_static \\"
echo "    /odom \\"
echo "    /cmd_vel \\"
echo "    /local_costmap/costmap_raw \\"
echo "    /global_costmap/costmap_raw \\"
echo "    /local_costmap/published_footprint \\"
echo "    /tunnel_frontier_explorer/frontier_markers \\"
echo "    -o ${OUTPUT_DIR}/frontier_smoke_bag"
echo ""
echo " RViz markers topic: /tunnel_frontier_explorer/frontier_markers"
echo ""
echo " Press Ctrl+C to stop the Frontier Explorer."
echo "============================================"

wait "${EXPLORER_PID}"
