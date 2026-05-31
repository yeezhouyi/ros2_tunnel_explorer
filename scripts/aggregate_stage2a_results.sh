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
# aggregate_stage2a_results.sh
#
# Aggregate 5-run Stage 2A benchmark results into a cross-run summary.
# Only includes COMPLETED (or --allow-timeout) runs in the statistics.
# Excluded runs are reported with their status and directory path.
#
# Usage:
#   ./scripts/aggregate_stage2a_results.sh <results-dir> [--allow-timeout]
#
# Example:
#   ./scripts/aggregate_stage2a_results.sh ~/stage2a_benchmark
#   ./scripts/aggregate_stage2a_results.sh ~/stage2a_benchmark --allow-timeout

set -euo pipefail

RESULTS_DIR="${1:-${HOME}/stage2a_benchmark}"
SCRIPT_NAME="$(basename "$0")"
ALLOW_TIMEOUT=false
ALLOWED_STATUSES="COMPLETED"

if [ $# -ge 2 ] && [ "$2" = "--allow-timeout" ]; then
  ALLOW_TIMEOUT=true
  ALLOWED_STATUSES="COMPLETED TIMEOUT"
fi

if [ ! -d "${RESULTS_DIR}" ]; then
  echo "Usage: $0 <results-dir> [--allow-timeout]" >&2
  exit 1
fi

# ── Helper: compute median, min, max ──────────────────────────────────
stats() {
  local -a values=()
  while IFS= read -r val; do
    val="${val//[^0-9.]/}"
    [ -n "${val}" ] && values+=("${val}")
  done

  local count="${#values[@]}"
  [ "${count}" -eq 0 ] && { echo "N/A N/A N/A"; return; }

  IFS=$'\n' sorted=($(sort -n <<<"${values[*]}"))
  unset IFS

  local min="${sorted[0]}"
  local max="${sorted[$((count - 1))]}"
  local mid=$((count / 2))

  if [ $((count % 2)) -eq 0 ] && [ "${count}" -ge 2 ]; then
    local median
    median=$(echo "scale=1; (${sorted[$((mid - 1))]} + ${sorted[${mid}]}) / 2" | bc)
    echo "${median} ${min} ${max}"
  else
    echo "${sorted[${mid}]} ${min} ${max}"
  fi
}

# ── Collect run directories ───────────────────────────────────────────
declare -a RUN_DIRS=()
for d in "${RESULTS_DIR}"/run_*; do
  [ -f "${d}/benchmark_results.md" ] && RUN_DIRS+=("${d}")
done

RUN_COUNT="${#RUN_DIRS[@]}"
echo "[${SCRIPT_NAME}] Found ${RUN_COUNT} run(s) in ${RESULTS_DIR}"
[ "${RUN_COUNT}" -eq 0 ] && { echo "[${SCRIPT_NAME}] No runs found." >&2; exit 1; }

# ── Extract helpers ───────────────────────────────────────────────────
extract() {
  local file="$1" pattern="$2"
  grep -iE "${pattern}" "${file}" 2>/dev/null | grep -oP '\b[0-9]+(\.[0-9]+)?' | head -1 || echo "0"
}

extract_str() {
  local file="$1" pattern="$2"
  grep -iE "${pattern}" "${file}" 2>/dev/null | head -1 | sed 's/.*| *//; s/ *|.*//; s/^ *//; s/ *$//' || echo ""
}

# ── Classify runs ─────────────────────────────────────────────────────
declare -a VALID_DIRS=()
declare -a EXCLUDED_DIRS=()
declare -a EXCLUDED_REASONS=()

for d in "${RUN_DIRS[@]}"; do
  md="${d}/benchmark_results.md"
  run_status="$(extract_str "${md}" "Run status")"

  case "${run_status}" in
    COMPLETED)
      VALID_DIRS+=("${d}")
      ;;
    TIMEOUT)
      if ${ALLOW_TIMEOUT}; then
        VALID_DIRS+=("${d}")
      else
        EXCLUDED_DIRS+=("${d}")
        EXCLUDED_REASONS+=("TIMEOUT (use --allow-timeout to include)")
      fi
      ;;
    CRASHED|STARTUP_FAILED|INVALID_ORCHESTRATION_TERMINATION)
      EXCLUDED_DIRS+=("${d}")
      EXCLUDED_REASONS+=("${run_status}")
      ;;
    "")
      EXCLUDED_DIRS+=("${d}")
      EXCLUDED_REASONS+=("unknown (run_status not found)")
      ;;
    *)
      EXCLUDED_DIRS+=("${d}")
      EXCLUDED_REASONS+=("${run_status}")
      ;;
  esac
done

VALID_COUNT="${#VALID_DIRS[@]}"
EXCLUDED_COUNT="${#EXCLUDED_DIRS[@]}"

echo "[${SCRIPT_NAME}] Valid runs: ${VALID_COUNT}, Excluded: ${EXCLUDED_COUNT}"

# ── Extract metrics from valid runs only ──────────────────────────────
G_LIST=""; S_LIST=""; T_LIST=""; A_LIST=""; R_LIST=""
U_LIST=""; RP_LIST=""; RV_LIST=""; F_LIST=""
MN_LIST=""; MX_LIST=""; MC_LIST=""; XC_LIST=""
ST_LIST=""; TR_LIST=""; CV_LIST=""
CT_LIST=""  # completion time

for d in "${VALID_DIRS[@]}"; do
  md="${d}/benchmark_results.md"

  g=$(extract "${md}" "Goals dispatched")
  s=$(extract "${md}" "Goals succeeded")
  t=$(extract "${md}" "Goals timed out")
  a=$(extract "${md}" "Goals aborted")
  r=$(extract "${md}" "Goal reachability")
  u=$(extract "${md}" "Unique goal bins")
  rp=$(extract "${md}" "Repeated goals")
  rv=$(extract "${md}" "Revisit rate")
  f=$(extract "${md}" "filter activations")
  mn=$(extract "${md}" "Min goal distance")
  mx=$(extract "${md}" "Max goal distance")
  mc=$(extract "${md}" "Min cluster cells")
  xc=$(extract "${md}" "Max cluster cells")
  st=$(extract "${md}" "Nav2 startup attempts" | head -1)
  ct=$(extract "${md}" "Completion time")

  G_LIST="${G_LIST}${g} "; S_LIST="${S_LIST}${s} "; T_LIST="${T_LIST}${t} "
  A_LIST="${A_LIST}${a} "; R_LIST="${R_LIST}${r} "
  U_LIST="${U_LIST}${u} "; RP_LIST="${RP_LIST}${rp} "; RV_LIST="${RV_LIST}${rv} "
  F_LIST="${F_LIST}${f} "
  MN_LIST="${MN_LIST}${mn} "; MX_LIST="${MX_LIST}${mx} "
  MC_LIST="${MC_LIST}${mc} "; XC_LIST="${XC_LIST}${xc} "
  ST_LIST="${ST_LIST}${st} "; CT_LIST="${CT_LIST}${ct} "

  # Rosbag metrics
  json="${d}/rosbag_metrics.json"
  if [ -f "${json}" ]; then
    tr=$(python3 -c "import json; print(json.load(open('${json}')).get('travel_distance_m', 0))" 2>/dev/null || echo "0")
    cv=$(python3 -c "import json; d=json.load(open('${json}')); print(d.get('map_coverage', {}).get('coverage_proxy', 0))" 2>/dev/null || echo "0")
  else
    tr="0"; cv="0"
  fi
  TR_LIST="${TR_LIST}${tr} "; CV_LIST="${CV_LIST}${cv} "
done

# ── Compute statistics ────────────────────────────────────────────────
row() {
  local name="$1" list="$2" unit="$3"
  read -r med min max <<< "$(echo "${list}" | tr ' ' '\n' | grep -v '^$' | stats)"
  printf "| %-37s | %8s | %8s | %8s |\n" "${name}" "${med}" "${min}" "${max}"
}

# ── Write output ──────────────────────────────────────────────────────
OUTPUT="${RESULTS_DIR}/aggregated_results.md"
{
  echo "# Stage 2A: Cross-Run Aggregated Results"
  echo ""
  echo "- Total runs: ${RUN_COUNT}"
  echo "- Valid runs: ${VALID_COUNT} (${ALLOWED_STATUSES})"
  echo "- Excluded:   ${EXCLUDED_COUNT}"
  echo "- Date: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "- Tag: stage1c-nearest-frontier-pass"
  echo ""

  if [ "${EXCLUDED_COUNT}" -gt 0 ]; then
    echo "## Excluded Runs"
    echo ""
    echo "| Run | Status |"
    echo "|-----|--------|"
    for i in "${!EXCLUDED_DIRS[@]}"; do
      dirname="$(basename "${EXCLUDED_DIRS[$i]}")"
      echo "| ${dirname} | ${EXCLUDED_REASONS[$i]} |"
    done
    echo ""
  fi

  if [ "${VALID_COUNT}" -eq 0 ]; then
    echo "**No valid runs to aggregate.**"
    echo ""
    echo "---"
    echo "Generated by ${SCRIPT_NAME}"
    tee "${OUTPUT}"
    echo "[${SCRIPT_NAME}] No valid runs — output written to ${OUTPUT}" >&2
    exit 0
  fi

  echo "## Exploration Efficiency"
  echo ""
  echo "| Metric | Median | Min | Max |"
  echo "|--------|--------|-----|-----|"
  row "Goals dispatched"          "${G_LIST}"  ""
  row "Goals succeeded"           "${S_LIST}"  ""
  row "Goals timed out"           "${T_LIST}"  ""
  row "Goals aborted"             "${A_LIST}"  ""
  row "Reachability rate (%)"     "${R_LIST}"  ""
  row "Unique goal bins (0.25m)"  "${U_LIST}"  ""
  row "Repeated goals"            "${RP_LIST}" ""
  row "Revisit rate (%)"          "${RV_LIST}" ""
  echo ""
  echo "## Goal Quality"
  echo ""
  echo "| Metric | Median | Min | Max |"
  echo "|--------|--------|-----|-----|"
  row "Filter activations"        "${F_LIST}"  ""
  row "Min goal distance (m)"     "${MN_LIST}" ""
  row "Max goal distance (m)"     "${MX_LIST}" ""
  echo ""
  echo "## Cluster Characteristics"
  echo ""
  echo "| Metric | Median | Min | Max |"
  echo "|--------|--------|-----|-----|"
  row "Min cluster cells"         "${MC_LIST}" ""
  row "Max cluster cells"         "${XC_LIST}" ""
  echo ""
  echo "## System Stability"
  echo ""
  echo "| Metric | Median | Min | Max |"
  echo "|--------|--------|-----|-----|"
  row "Nav2 startup attempts"     "${ST_LIST}" ""
  echo ""
  echo "## Completion"
  echo ""
  echo "| Metric | Median | Min | Max |"
  echo "|--------|--------|-----|-----|"
  row "Completion time (s)"       "${CT_LIST}" ""
  echo ""
  echo "## Offline Metrics (from rosbag)"
  echo ""
  echo "| Metric | Median | Min | Max |"
  echo "|--------|--------|-----|-----|"
  row "Travel distance (m)"       "${TR_LIST}" ""
  row "Coverage proxy"            "${CV_LIST}" ""
  echo ""
  echo "---"
  echo "Generated by ${SCRIPT_NAME}"
} | tee "${OUTPUT}"

echo "[${SCRIPT_NAME}] Done."
