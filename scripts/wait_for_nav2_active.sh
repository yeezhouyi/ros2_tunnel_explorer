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

# Allow --timeout or --timeout-seconds override from args
while [ $# -gt 0 ]; do
  case "$1" in
    --timeout) TIMEOUT_SECONDS="$2"; shift 2 ;;
    --timeout-seconds) TIMEOUT_SECONDS="$2"; shift 2 ;;
    --help)
      sed -n 's/^# //p; /^$/q' "$0" | head -40
      exit 0
      ;;
    *) shift ;;  # ignore unknown for compatibility
  esac
done

# Source ROS2 and workspace if not already in environment.
if ! command -v ros2 &> /dev/null; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
if [ -z "${COLCON_PREFIX_PATH:-}" ]; then
  if [ -f "${HOME}/ros2_tunnel_explorer/install/setup.bash" ]; then
    set +u
    source "${HOME}/ros2_tunnel_explorer/install/setup.bash"
    set -u
  fi
fi

REQUIRED_NODES=(
  "/planner_server"
  "/controller_server"
  "/bt_navigator"
  "/behavior_server"
)

echo "[${SCRIPT_NAME}] Waiting for Nav2 lifecycle nodes to become active..."

first_pass_done=false
elapsed=0
while [ "${elapsed}" -lt "${TIMEOUT_SECONDS}" ]; do
  all_active=true
  for node in "${REQUIRED_NODES[@]}"; do
    state="$(ros2 lifecycle get "${node}" 2>/dev/null || echo "unknown")"
    # ros2 lifecycle get returns "active [3]" — match on word prefix.
    case "${state}" in
      active*) ;;
      *) all_active=false; break ;;
    esac
  done

  if ${all_active}; then
    if ${first_pass_done}; then
      # Second consecutive pass — lifecycle is stable.
      # Now verify the critical map→base_link TF, which may take
      # extra time in WSL2 due to Gazebo→ROS bridge startup latency.
      echo "[${SCRIPT_NAME}] Lifecycle stable — verifying TF tree..."
      tf_out="$(mktemp)"
      timeout 10 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null \
        > "${tf_out}" || true
      if grep -q "At time" "${tf_out}" 2>/dev/null; then
        rm -f "${tf_out}"
        echo "[${SCRIPT_NAME}] All Nav2 lifecycle nodes are active."
        exit 0
      fi
      rm -f "${tf_out}"
      # TF not ready yet — keep first_pass_done=true so we retry TF
      # immediately on next cycle (no 5s stabilization penalty).
      echo "[${SCRIPT_NAME}] TF not ready yet, continuing..."
      sleep 5
      elapsed=$((elapsed + 5))
      continue
    fi
    # First time seeing all active — wait for stabilization
    # (global_costmap may asynchronously fail after activation if the
    #  map→base_link TF is not yet available from SLAM)
    first_pass_done=true
    sleep 5
    elapsed=$((elapsed + 5))
    continue
  fi

  first_pass_done=false
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
