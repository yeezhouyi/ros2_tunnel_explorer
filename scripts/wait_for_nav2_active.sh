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
# wait_for_nav2_active.sh
#
# Poll Nav2 lifecycle nodes until all are active or a timeout expires.
#
# Usage:
#   ./scripts/wait_for_nav2_active.sh
#
# Exit codes:
#   0  — all required lifecycle nodes are active
#   1  — timed out waiting for one or more nodes
#
# Dependencies:
#   - ROS2 Jazzy environment sourced
#   - ros2 lifecycle command available (ros2cli)

set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
TIMEOUT_SECONDS=60
INTERVAL_SECONDS=2

# Source ROS2 and workspace if not already in environment.
if ! command -v ros2 &> /dev/null; then
  source /opt/ros/jazzy/setup.bash
fi
if [ -z "${COLCON_PREFIX_PATH:-}" ]; then
  if [ -f "${HOME}/ros2_tunnel_explorer/install/setup.bash" ]; then
    source "${HOME}/ros2_tunnel_explorer/install/setup.bash"
  fi
fi

REQUIRED_NODES=(
  "/planner_server"
  "/controller_server"
  "/bt_navigator"
  "/behavior_server"
)

echo "[${SCRIPT_NAME}] Waiting for Nav2 lifecycle nodes to become active..."

elapsed=0
while [ "${elapsed}" -lt "${TIMEOUT_SECONDS}" ]; do
  all_active=true
  for node in "${REQUIRED_NODES[@]}"; do
    state="$(ros2 lifecycle get "${node}" 2>/dev/null || echo "unknown")"
    if [ "${state}" != "active" ]; then
      all_active=false
      break
    fi
  done

  if ${all_active}; then
    echo "[${SCRIPT_NAME}] All Nav2 lifecycle nodes are active."
    exit 0
  fi

  sleep "${INTERVAL_SECONDS}"
  elapsed=$((elapsed + INTERVAL_SECONDS))
done

# ── Timeout diagnostics ─────────────────────────────────────────────

echo "[${SCRIPT_NAME}] TIMEOUT: Nav2 nodes not active within ${TIMEOUT_SECONDS}s." >&2
echo "" >&2

echo "  -- ros2 node list --" >&2
if ! ros2 node list 2>/dev/null; then
  echo "  (ros2 node list failed)" >&2
fi
echo "" >&2

echo "  -- lifecycle states --" >&2
for node in "${REQUIRED_NODES[@]}"; do
  state="$(ros2 lifecycle get "${node}" 2>/dev/null || echo "unreachable")"
  echo "  ${node}: ${state}" >&2
done
echo "" >&2

echo "  -- /navigate_to_pose action info --" >&2
if ! ros2 action info /navigate_to_pose 2>/dev/null; then
  echo "  /navigate_to_pose action server not found" >&2
fi
echo "" >&2

echo "[${SCRIPT_NAME}] Hint: ensure simulation is running with nav2_params_rotation_shim.yaml" >&2

exit 1
