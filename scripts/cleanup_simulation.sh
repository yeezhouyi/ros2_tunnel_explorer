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
# cleanup_simulation.sh
#
# Clean up stale simulation processes and shared-memory state before
# launching a new ROS2 + Gazebo simulation run.
#
# Usage:
#   ./scripts/cleanup_simulation.sh
#
# Safe to run even when no prior simulation processes exist.

set -euo pipefail

# Collect PIDs of ALL simulation-related processes.
# We use ps+awk instead of pkill to avoid accidentally killing the
# calling shell (e.g. pkill matching "gz sim" can kill the Claude/Bash
# process that invoked this script).
MATCH="gz sim|ros_gz_bridge|lifecycle_manager|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|collision_monitor|docking_server|waypoint_follower|rviz2|ros2 launch|parameter_bridge|robot_state_publisher|map_saver|slam_toolbox"

pids="$(ps aux | grep -E "${MATCH}" | grep -v grep | awk '{print $2}' || true)"
if [ -n "${pids}" ]; then
  echo "[cleanup] Killing processes: $(echo ${pids} | tr '\n' ' ')"
  # SIGTERM first, then SIGKILL after 5s for survivors
  kill ${pids} 2>/dev/null || true
  sleep 5
  kill -9 ${pids} 2>/dev/null || true
else
  echo "[cleanup] No matching processes found."
fi

echo "[cleanup] Stopping ros2 daemon..."
ros2 daemon stop 2>/dev/null || true

echo "[cleanup] Removing stale Fast DDS shared-memory files..."
find /dev/shm \
  -maxdepth 1 \
  -type f \
  -name '*fastrtps*' \
  -print \
  -delete 2>/dev/null || true

echo "[cleanup] Done."
