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
# Aggregate Stage 2A benchmark results from per-attempt benchmark_results.json
# files.  Three-layer classification:
#
#   1. Algorithm Outcomes
#      - COMPLETED  (stable_completion_detected == true)
#      - TIMEOUT    (included only with --allow-timeout)
#   2. Algorithm Exclusions
#      - COMPLETED but injected_stall_enabled == true
#      - COMPLETED but stable_completion_detected == false
#   3. Infrastructure Exclusions
#      - STALLED, CRASHED, STARTUP_FAILED, INVALID_ORCHESTRATION_TERMINATION
#
# For each run directory the BEST (last) attempt with a
# benchmark_results.json is used.
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

if [ $# -ge 2 ] && [ "$2" = "--allow-timeout" ]; then
  ALLOW_TIMEOUT=true
fi

if [ ! -d "${RESULTS_DIR}" ]; then
  echo "Usage: $0 <results-dir> [--allow-timeout]" >&2
  exit 1
fi

echo "[${SCRIPT_NAME}] Aggregating results from ${RESULTS_DIR}"

# ─────────────────────────────────────────────────────────────────────────────
# Step 1: Python classification + statistics
# ─────────────────────────────────────────────────────────────────────────────
# Variables are passed via the environment so the heredoc can be single-quote
# delimited (no unwanted shell expansion inside the Python code).
export _AGGR_RESULTS_DIR="${RESULTS_DIR}"
export _AGGR_ALLOW_TIMEOUT="${ALLOW_TIMEOUT}"

PY_SCRIPT=$(mktemp /tmp/aggregate_stage2a_XXXXXXXX.py)
cat > "${PY_SCRIPT}" << 'PYEOF'
import json
import os
from pathlib import Path

base_dir = Path(os.environ["_AGGR_RESULTS_DIR"])
allow_timeout = os.environ.get("_AGGR_ALLOW_TIMEOUT", "false").lower() == "true"

run_dirs = sorted([d for d in base_dir.iterdir() if d.name.startswith("run_")])

# ── Three-layer buckets ──────────────────────────────────────────────────
algorithm_completed = []   # (run_id, data)
algorithm_timeout   = []   # (run_id, data)
algorithm_excluded  = []   # (run_id, reason, data)
infra_excluded      = []   # (run_id, status, data)
injected_runs       = []   # (run_id, status, data)

for run_dir in run_dirs:
    attempt_dirs = sorted([d for d in run_dir.iterdir() if d.name.startswith("attempt_")])

    best_data = None
    for attempt_dir in reversed(attempt_dirs):
        json_file = attempt_dir / "benchmark_results.json"
        if json_file.exists():
            best_data = json.loads(json_file.read_text())
            break

    if best_data is None:
        continue  # no JSON found for this run — skip

    run_id = run_dir.name
    status = best_data.get("run_status", "")
    injected = best_data.get("injected_stall_enabled", False)
    stable   = best_data.get("stable_completion_detected", False)

    if injected:
        injected_runs.append((run_id, status, best_data))
    elif status == "COMPLETED" and stable:
        algorithm_completed.append((run_id, best_data))
    elif status == "TIMEOUT":
        algorithm_timeout.append((run_id, best_data))
    elif status == "COMPLETED" and not stable:
        algorithm_timeout.append((run_id, best_data))
    else:
        infra_excluded.append((run_id, status, best_data))

# ── Statistics helpers (on algorithm COMPLETED only) ─────────────────────

def compute_stats(values):
    """Return {median, min, max} from a list of numbers."""
    if not values:
        return {"median": "N/A", "min": "N/A", "max": "N/A"}
    s = sorted(values)
    n = len(s)
    if n % 2 == 0:
        median = (s[n // 2 - 1] + s[n // 2]) / 2.0
    else:
        median = float(s[n // 2])
    return {
        "median": round(median, 1),
        "min": round(s[0], 1),
        "max": round(s[-1], 1),
    }


def g(d, key, default=0, fallback=None):
    """Safely get a numeric field, with optional fallback key."""
    v = d.get(key)
    if v is None and fallback is not None:
        v = d.get(fallback)
    if v is None:
        return default
    return v


comp_ct   = [g(r[1], "completion_time_seconds", 0) for r in algorithm_completed]
comp_g    = [g(r[1], "goals_dispatched") for r in algorithm_completed]
comp_s    = [g(r[1], "goals_succeeded") for r in algorithm_completed]
comp_u    = [g(r[1], "unique_goal_bins") for r in algorithm_completed]
comp_rv   = [g(r[1], "revisit_rate", 0) for r in algorithm_completed]
comp_rch  = [g(r[1], "navigation_goal_success_rate", 0, "reachability_rate") for r in algorithm_completed]

# ── Build output ─────────────────────────────────────────────────────────

completed_table = []
for run_id, d in algorithm_completed:
    completed_table.append({
        "run_id": run_id,
        "status": "COMPLETED",
        "goals_dispatched": g(d, "goals_dispatched"),
        "goals_succeeded": g(d, "goals_succeeded"),
        "unique_goal_bins": g(d, "unique_goal_bins"),
        "revisit_rate": g(d, "revisit_rate", 0),
        "navigation_goal_success_rate": g(d, "navigation_goal_success_rate", 0, "reachability_rate"),
        "completion_time_seconds": g(d, "completion_time_seconds", 0),
    })

timeout_table = []
for run_id, d in algorithm_timeout:
    timeout_table.append({
        "run_id": run_id,
        "status": "TIMEOUT",
        "goals_dispatched": g(d, "goals_dispatched"),
        "goals_succeeded": g(d, "goals_succeeded"),
        "unique_goal_bins": g(d, "unique_goal_bins"),
        "revisit_rate": g(d, "revisit_rate", 0),
        "navigation_goal_success_rate": g(d, "navigation_goal_success_rate", 0, "reachability_rate"),
    })

excluded_table = []
for run_id, reason, d in algorithm_excluded:
    excluded_table.append({
        "run_id": run_id,
        "status": d.get("run_status", ""),
        "reason": reason,
    })
for run_id, status, _d in injected_runs:
    excluded_table.append({
        "run_id": run_id,
        "status": status,
        "reason": "INJECTED_STALL",
    })
for run_id, status, _d in infra_excluded:
    excluded_table.append({
        "run_id": run_id,
        "status": status,
        "reason": status,
    })

infra_counts = {}
for _run_id, status, _d in infra_excluded:
    infra_counts[status] = infra_counts.get(status, 0) + 1

result = {
    "total_run_dirs": len(run_dirs),
    "algorithm_total": len(algorithm_completed) + len(algorithm_timeout),
    "completion_count": len(algorithm_completed),
    "timeout_count": len(algorithm_timeout),
    "completed_table": completed_table,
    "timeout_table": timeout_table,
    "excluded_table": excluded_table,
    "infra_counts": infra_counts,
    "stats": {
        "completion_time_seconds": compute_stats(comp_ct),
        "goals_dispatched": compute_stats(comp_g),
        "goals_succeeded": compute_stats(comp_s),
        "unique_goal_bins": compute_stats(comp_u),
        "revisit_rate": compute_stats(comp_rv),
        "navigation_goal_success_rate": compute_stats(comp_rch),
    },
}

print(json.dumps(result))
PYEOF
DATA_JSON=$(python3 "${PY_SCRIPT}" 2>/dev/null || echo '{"error":"classification_step_failed"}')
rm -f "${PY_SCRIPT}"

# Validate Python output
if ! echo "${DATA_JSON}" | python3 -c "
import json, sys
d = json.load(sys.stdin)
assert 'total_run_dirs' in d
" 2>/dev/null; then
  echo "[${SCRIPT_NAME}] Error: classification step failed." >&2
  echo "${DATA_JSON}" >&2
  exit 1
fi

# ─────────────────────────────────────────────────────────────────────────────
# Step 2: Bash helpers to extract data from the JSON classification result
# ─────────────────────────────────────────────────────────────────────────────

# get_val KEY  - extract a top-level string/number value from DATA_JSON
get_val() {
  echo "${DATA_JSON}" | python3 -c "
import json, sys
d = json.load(sys.stdin)
v = d.get('$1', 'N/A')
print(v)
" 2>/dev/null || echo "N/A"
}

# get_table_json TABLE-NAME  - dump a JSON array from DATA_JSON so its
# contents can be consumed row-by-row in a pipe.
get_table_json() {
  echo "${DATA_JSON}" | python3 -c "
import json, sys
d = json.load(sys.stdin)
rows = d.get('$1', [])
print(json.dumps(rows))
" 2>/dev/null || echo "[]"
}

# stat_cell METRIC FIELD  - extract a single stat cell (median/min/max)
stat_cell() {
  echo "${DATA_JSON}" | python3 -c "
import json, sys
d = json.load(sys.stdin)
s = d.get('stats', {}).get('$1', {})
print(s.get('$2', 'N/A'))
" 2>/dev/null || echo "N/A"
}

# ── Top-level counts ─────────────────────────────────────────────────────
TOTAL_RUN_DIRS=$(get_val "total_run_dirs")
ALGORITHM_TOTAL=$(get_val "algorithm_total")
COMPLETION_COUNT=$(get_val "completion_count")
TIMEOUT_COUNT=$(get_val "timeout_count")
EXCLUDED_COUNT=$(echo "${DATA_JSON}" | python3 -c "
import json, sys
d = json.load(sys.stdin)
print(len(d.get('excluded_table', [])))
" 2>/dev/null || echo "0")

# Extract selection_strategy from first completed/timeout run's JSON.
AGGREGATED_STRATEGY="nearest"
for run_dir in $(ls -d "${RESULTS_DIR}"/run_* 2>/dev/null | sort); do
  attempt_dir=$(ls -d "${run_dir}"/attempt_* 2>/dev/null | sort | tail -1)
  if [ -n "${attempt_dir}" ] && [ -f "${attempt_dir}/benchmark_results.json" ]; then
    candidate=$(python3 -c "
import json
with open('${attempt_dir}/benchmark_results.json') as f:
    print(json.load(f).get('selection_strategy', 'nearest'))
" 2>/dev/null || echo "")
    if [ -n "${candidate}" ]; then
      AGGREGATED_STRATEGY="${candidate}"
      break
    fi
  fi
done

# Completion rate (avoid division by zero)
if [ "${ALGORITHM_TOTAL}" -gt 0 ] 2>/dev/null; then
  COMPLETION_RATE=$((100 * COMPLETION_COUNT / ALGORITHM_TOTAL))
else
  COMPLETION_RATE="N/A"
fi

# ── Helper: print a markdown table row ──────────────────────────────────
md_row() {
  printf "| %s" "$1"
  shift
  for col in "$@"; do
    printf " | %s" "${col}"
  done
  printf " |\n"
}

# ─────────────────────────────────────────────────────────────────────────────
# Step 3: Write aggregated report
# ─────────────────────────────────────────────────────────────────────────────
OUTPUT="${RESULTS_DIR}/aggregated_results.md"
{
  echo "# Stage 2A: Cross-Run Aggregated Results"
  echo ""
  echo "- Total run directories: ${TOTAL_RUN_DIRS}"
  echo "- Algorithm runs total: ${ALGORITHM_TOTAL}"
  echo "- Algorithm COMPLETED: ${COMPLETION_COUNT}"
  echo "- Algorithm TIMEOUT: ${TIMEOUT_COUNT}"
  echo "- Completion rate: ${COMPLETION_RATE}%"
  echo "- Strategy: ${AGGREGATED_STRATEGY:-nearest}"
  echo ""

  # ─────── Algorithm Outcomes ─────────────────────────────────────────────
  echo "## Algorithm Outcomes"
  echo ""
  md_row "Run ID" "Status" "Goals" "Succeeded" "Unique" "Revisit %" "Nav Succ %" "Completion (s)"
  md_row "---" "---" "---" "---" "---" "---" "---" "---"

  # Completed rows
  COMPLETED_JSON=$(get_table_json "completed_table")
  echo "${COMPLETED_JSON}" | python3 -c "
import json, sys
rows = json.load(sys.stdin)
for r in rows:
    print(f\"| {r['run_id']} | {r['status']} | {r['goals_dispatched']} | {r['goals_succeeded']} | {r['unique_goal_bins']} | {r['revisit_rate']} | {r['navigation_goal_success_rate']} | {r['completion_time_seconds']} |\")
" 2>/dev/null

  # Timeout rows (always shown — TIMEOUT is a valid algorithm outcome)
  TIMEOUT_JSON=$(get_table_json "timeout_table")
  TIMEOUT_COUNT=$(echo "${TIMEOUT_JSON}" | python3 -c "import json,sys; print(len(json.load(sys.stdin)))" 2>/dev/null || echo 0)
  if [ "${TIMEOUT_COUNT}" -gt 0 ]; then
    echo "${TIMEOUT_JSON}" | python3 -c "
import json, sys
rows = json.load(sys.stdin)
for r in rows:
    print(f\"| {r['run_id']} | {r['status']} | {r['goals_dispatched']} | {r['goals_succeeded']} | {r['unique_goal_bins']} | {r['revisit_rate']} | {r['navigation_goal_success_rate']} | N/A |\")
" 2>/dev/null
  fi
  echo ""

  # ─────── Time-to-Completion ─────────────────────────────────────────────
  echo "### Time-to-Completion (stable COMPLETED only)"
  echo ""
  TTC_MED=$(stat_cell "completion_time_seconds" "median")
  TTC_MIN=$(stat_cell "completion_time_seconds" "min")
  TTC_MAX=$(stat_cell "completion_time_seconds" "max")
  echo "${TTC_MED} / ${TTC_MIN} / ${TTC_MAX} s"
  echo ""

  # ─────── Exploration Efficiency ─────────────────────────────────────────
  echo "### Exploration Efficiency (COMPLETED runs)"
  echo ""
  md_row "Metric" "Median" "Min" "Max"
  md_row "---" "---" "---" "---"

  for entry in \
    "goals_dispatched:Goals dispatched" \
    "goals_succeeded:Goals succeeded" \
    "unique_goal_bins:Unique goal bins" \
    "revisit_rate:Revisit rate (%)" \
    "navigation_goal_success_rate:Navigation goal success rate (%)"
  do
    KEY="${entry%%:*}"
    LABEL="${entry#*:}"
    MED=$(stat_cell "${KEY}" "median")
    MIN=$(stat_cell "${KEY}" "min")
    MAX=$(stat_cell "${KEY}" "max")
    md_row "${LABEL}" "${MED}" "${MIN}" "${MAX}"
  done
  echo ""

  # ─────── Excluded Runs ──────────────────────────────────────────────────
  if [ "${EXCLUDED_COUNT}" -gt 0 ]; then
    echo "## Excluded Runs"
    echo ""
    echo "### Algorithm Exclusions (injected or incomplete)"
    echo ""
    md_row "Run" "Status" "Reason"
    md_row "---" "---" "---"

    EXCLUDED_JSON=$(get_table_json "excluded_table")
    echo "${EXCLUDED_JSON}" | python3 -c "
import json, sys
rows = json.load(sys.stdin)
for r in rows:
    if r.get('reason') in ('INJECTED_STALL', 'NOT_STABLE'):
        print(f\"| {r['run_id']} | {r.get('status','')} | {r.get('reason','')} |\")
" 2>/dev/null

    echo ""
    echo "### Infrastructure Exclusions"
    echo ""
    md_row "Status" "Count"
    md_row "---" "---"

    echo "${DATA_JSON}" | python3 -c "
import json, sys
d = json.load(sys.stdin)
for status in sorted(d.get('infra_counts', {}).keys()):
    cnt = d['infra_counts'][status]
    print(f\"| {status} | {cnt} |\")
" 2>/dev/null
    echo ""
  fi

  echo "---"
  echo "Generated by ${SCRIPT_NAME}"
} | tee "${OUTPUT}"

echo "[${SCRIPT_NAME}] Aggregation written to ${OUTPUT}"
