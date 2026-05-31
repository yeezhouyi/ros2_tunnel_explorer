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
# dump_nav2_diagnostic_params.sh — Snapshot Nav2 parameter state for diagnosis.
#
# Saves node list, controller/costmap parameters, topic list, and action list
# to a timestamped output directory. Handles missing nodes gracefully.
#
# Usage:
#   ./scripts/dump_nav2_diagnostic_params.sh
#   ./scripts/dump_nav2_diagnostic_params.sh /custom/output/dir
#
# No sudo, no destructive operations, no new dependencies.

set -u

OUTPUT_DIR="${1:-${HOME}/stage0b_diagnostic/params}"
mkdir -p "${OUTPUT_DIR}"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SUMMARY="${OUTPUT_DIR}/summary_${TIMESTAMP}.txt"

echo "=== Nav2 Diagnostic Parameter Snapshot ===" | tee "${SUMMARY}"
echo "Timestamp: $(date)" | tee -a "${SUMMARY}"
echo "Output dir: ${OUTPUT_DIR}" | tee -a "${SUMMARY}"
echo "" | tee -a "${SUMMARY}"

# ------------------------------------------------------------------
# Node list
# ------------------------------------------------------------------
echo "--- ros2 node list ---" | tee -a "${SUMMARY}"
ros2 node list 2>/dev/null \
  | tee "${OUTPUT_DIR}/node_list_${TIMESTAMP}.txt" \
  | tee -a "${SUMMARY}"
echo "" | tee -a "${SUMMARY}"

# ------------------------------------------------------------------
# controller_server parameters
# ----------------------------------------------------------------->
NODE="/controller_server"
echo "--- ${NODE} parameters ---" | tee -a "${SUMMARY}"
if ros2 node info "${NODE}" >/dev/null 2>&1; then
  ros2 param dump "${NODE}" \
    > "${OUTPUT_DIR}/controller_server_${TIMESTAMP}.yaml" 2>&1
  echo "  Saved to controller_server_${TIMESTAMP}.yaml" | tee -a "${SUMMARY}"
else
  echo "  WARNING: ${NODE} not found — skipping" | tee -a "${SUMMARY}"
fi
echo "" | tee -a "${SUMMARY}"

# ------------------------------------------------------------------
# Local costmap node(s)
# ------------------------------------------------------------------
echo "--- local costmap nodes ---" | tee -a "${SUMMARY}"
LOCAL_COSTMAP_NODES=$(ros2 node list 2>/dev/null | grep -i "local_costmap" || true)
if [ -z "${LOCAL_COSTMAP_NODES}" ]; then
  echo "  WARNING: No local costmap nodes found" | tee -a "${SUMMARY}"
else
  while IFS= read -r NODE; do
    [ -z "${NODE}" ] && continue
    SAFE_NAME=$(echo "${NODE}" | tr '/' '_')
    echo "  Dumping ${NODE} ..." | tee -a "${SUMMARY}"
    ros2 param dump "${NODE}" \
      > "${OUTPUT_DIR}/${SAFE_NAME}_${TIMESTAMP}.yaml" 2>&1 \
      || echo "  WARNING: Failed to dump ${NODE}" | tee -a "${SUMMARY}"
  done <<< "${LOCAL_COSTMAP_NODES}"
fi
echo "" | tee -a "${SUMMARY}"

# ------------------------------------------------------------------
# Global costmap node(s)
# ------------------------------------------------------------------
echo "--- global costmap nodes ---" | tee -a "${SUMMARY}"
GLOBAL_COSTMAP_NODES=$(ros2 node list 2>/dev/null | grep -i "global_costmap" || true)
if [ -z "${GLOBAL_COSTMAP_NODES}" ]; then
  echo "  WARNING: No global costmap nodes found" | tee -a "${SUMMARY}"
else
  while IFS= read -r NODE; do
    [ -z "${NODE}" ] && continue
    SAFE_NAME=$(echo "${NODE}" | tr '/' '_')
    echo "  Dumping ${NODE} ..." | tee -a "${SUMMARY}"
    ros2 param dump "${NODE}" \
      > "${OUTPUT_DIR}/${SAFE_NAME}_${TIMESTAMP}.yaml" 2>&1 \
      || echo "  WARNING: Failed to dump ${NODE}" | tee -a "${SUMMARY}"
  done <<< "${GLOBAL_COSTMAP_NODES}"
fi
echo "" | tee -a "${SUMMARY}"

# ------------------------------------------------------------------
# Topic list
# ------------------------------------------------------------------
echo "--- ros2 topic list ---" | tee -a "${SUMMARY}"
ros2 topic list 2>/dev/null \
  | tee "${OUTPUT_DIR}/topic_list_${TIMESTAMP}.txt" \
  | tee -a "${SUMMARY}"
echo "" | tee -a "${SUMMARY}"

# ------------------------------------------------------------------
# Action list
# ------------------------------------------------------------------
echo "--- ros2 action list ---" | tee -a "${SUMMARY}"
ros2 action list 2>/dev/null \
  | tee "${OUTPUT_DIR}/action_list_${TIMESTAMP}.txt" \
  | tee -a "${SUMMARY}"
echo "" | tee -a "${SUMMARY}"

# ------------------------------------------------------------------
# Done
# ------------------------------------------------------------------
echo "=== Snapshot complete ===" | tee -a "${SUMMARY}"
echo "Summary: ${SUMMARY}" | tee -a "${SUMMARY}"
