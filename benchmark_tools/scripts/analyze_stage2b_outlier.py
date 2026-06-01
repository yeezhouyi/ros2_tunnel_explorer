#!/usr/bin/env python3
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
# analyze_stage2b_outlier.py
#
# Analyse a Stage 2B outlier run (e.g. run_4 with 65 % revisit rate, 387 s TTC).
# Parses frontier_explorer.log, simulation.log, and benchmark_results.json to
# classify the failure mode.  No parameter changes — diagnosis only.
#
# Usage:
#   python3 analyze_stage2b_outlier.py \
#     --run-dir ~/stage2b_info_revisit_formal/run_4 \
#     --output-dir ~/stage2b_info_revisit_formal/run_4/analysis
#
# Output:
#   stage2b_run4_outlier.json   — structured data
#   stage2b_run4_outlier.md     — diagnostic report

import argparse
import json
import math
import os
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path


# ── Constants ─────────────────────────────────────────────────────────────────

SPATIAL_BIN_SIZE = 0.5  # metres, matching Stage 2A/2B benchmark binning

# Fields extracted from each goal-dispatch log line
GOAL_LOG_RE = re.compile(
    r"Goal:\s*"
    r"\((?P<gx>-?\d+\.\d+),\s*(?P<gy>-?\d+\.\d+)\)\s+"
    r"dist=(?P<dist>-?\d+\.\d+)\s+"
    r"gain=(?P<raw_gain>\d+)\(raw\)/(?P<tr_gain>\d+\.\d+)\(tr\)/(?P<norm_gain>\d+\.\d+)\(norm\)\s+"
    r"revisit=(?P<raw_rev>\d+)\(raw\)/(?P<cl_rev>\d+)\(cl\)/(?P<norm_rev>\d+\.\d+)\(norm\)\s+"
    r"score=(?P<score>-?\d+\.\d+)\s+"
    r"\[(?P<cand>\d+) cand\]"
)

# Result lines
RESULT_SUCCEEDED = re.compile(r"Navigation to goal succeeded")
RESULT_ABORTED = re.compile(r"Navigation aborted")
RESULT_TIMEOUT = re.compile(r"Navigation timed out")
BLACKLIST_RE = re.compile(r"Blacklisted\s+\((-?\d+\.\d+),\s*(-?\d+\.\d+)\)\s+for\s+(\d+)\s*s")
BLACKLIST_ALL_RE = re.compile(r"All \d+ distance-candidates blacklisted")
NO_FRONTIERS_RE = re.compile(r"No frontiers")
DWB_ERROR_RE = re.compile(r"No valid trajectories out of \d+")

# ── Helpers ───────────────────────────────────────────────────────────────────


def spatial_bin(x, y, size=SPATIAL_BIN_SIZE):
    """Return (bx, by) bin coordinates for a given (x, y)."""
    return (math.floor(x / size) * size, math.floor(y / size) * size)


def epoch_to_sec(epoch_str):
    """Parse ROS timestamp float (seconds.nanoseconds) to seconds."""
    return float(epoch_str)


def adjacent_distance_m(g1, g2):
    """Euclidean distance between two goal positions."""
    dx = g1["goal_x"] - g2["goal_x"]
    dy = g1["goal_y"] - g2["goal_y"]
    return math.sqrt(dx * dx + dy * dy)


# ── Parsers ───────────────────────────────────────────────────────────────────


def parse_frontier_log(log_path):
    """Parse frontier_explorer.log and return structured goal events."""
    goals = []
    current_goal = None
    lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()

    for lineno, line in enumerate(lines, 1):
        # Strip ANSI escape codes
        stripped = re.sub(r"\x1b\[[0-9;]*m", "", line)

        # Goal dispatch
        m = GOAL_LOG_RE.search(stripped)
        if m:
            if current_goal is not None:
                # Previous goal never got a result (unusual)
                current_goal["navigation_result"] = "UNKNOWN"
                goals.append(current_goal)

            current_goal = {
                "line_number": lineno,
                "timestamp_epoch": None,  # filled from ROS timestamp
                "goal_x": float(m.group("gx")),
                "goal_y": float(m.group("gy")),
                "goal_distance": float(m.group("dist")),
                "raw_information_gain": int(m.group("raw_gain")),
                "transformed_information_gain": float(m.group("tr_gain")),
                "normalized_information_gain": float(m.group("norm_gain")),
                "raw_revisit_count": int(m.group("raw_rev")),
                "clamped_revisit_count": int(m.group("cl_rev")),
                "normalized_revisit_penalty": float(m.group("norm_rev")),
                "final_score": float(m.group("score")),
                "candidate_count": int(m.group("cand")),
                "navigation_result": None,
                "bin": spatial_bin(float(m.group("gx")), float(m.group("gy"))),
            }

            # Extract ROS timestamp from the log line
            ts_m = re.search(r"\[(\d+\.\d+)\]", stripped)
            if ts_m:
                current_goal["timestamp_epoch"] = float(ts_m.group(1))

            continue

        # Navigation result
        if current_goal is not None:
            if RESULT_SUCCEEDED.search(stripped):
                current_goal["navigation_result"] = "SUCCEEDED"
                goals.append(current_goal)
                current_goal = None
                continue
            elif RESULT_ABORTED.search(stripped):
                current_goal["navigation_result"] = "ABORTED"
                goals.append(current_goal)
                current_goal = None
                continue
            elif RESULT_TIMEOUT.search(stripped):
                current_goal["navigation_result"] = "TIMEOUT"
                goals.append(current_goal)
                current_goal = None
                continue

    # Flush any unclosed goal
    if current_goal is not None:
        current_goal["navigation_result"] = "UNKNOWN"
        goals.append(current_goal)

    return goals


def parse_blacklist_events(log_path):
    """Count blacklist events from frontier_explorer.log."""
    blacklist_count = 0
    blacklist_all_count = 0
    no_frontiers_cycles = 0
    lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()

    for line in lines:
        stripped = re.sub(r"\x1b\[[0-9;]*m", "", line)
        if BLACKLIST_RE.search(stripped):
            blacklist_count += 1
        if BLACKLIST_ALL_RE.search(stripped):
            blacklist_all_count += 1
        if NO_FRONTIERS_RE.search(stripped):
            no_frontiers_cycles += 1

    return {
        "blacklist_insertions": blacklist_count,
        "blacklist_all_events": blacklist_all_count,
        "no_frontiers_cycles": no_frontiers_cycles,
    }


def parse_dwb_errors(sim_log_path):
    """Count DWB 'No valid trajectories' errors from simulation.log."""
    if not sim_log_path.exists():
        return {"dwb_no_valid_trajectories": -1, "simulation_log_found": False}

    count = 0
    for line in sim_log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = re.sub(r"\x1b\[[0-9;]*m", "", line)
        if DWB_ERROR_RE.search(stripped):
            count += 1

    return {"dwb_no_valid_trajectories": count, "simulation_log_found": True}


def load_benchmark_json(json_path):
    """Load benchmark_results.json."""
    if not json_path.exists():
        return None
    return json.loads(json_path.read_text())


# ── Analysis ──────────────────────────────────────────────────────────────────


def compute_pattern_metrics(goals):
    """Analyse goal sequence for cycling patterns."""
    if not goals:
        return {}

    # Per-bin statistics
    bin_counts = Counter(g["bin"] for g in goals)
    bin_success = defaultdict(lambda: {"total": 0, "succeeded": 0})
    for g in goals:
        b = g["bin"]
        bin_success[b]["total"] += 1
        if g["navigation_result"] == "SUCCEEDED":
            bin_success[b]["succeeded"] += 1

    max_revisits_per_bin = {}
    for b in bin_counts:
        max_revisits_per_bin[str(b)] = max(
            g["clamped_revisit_count"]
            for g in goals
            if g["bin"] == b
        )

    # Adjacent goal distances
    adj_distances = []
    for i in range(1, len(goals)):
        d = adjacent_distance_m(goals[i - 1], goals[i])
        adj_distances.append(d)

    # Cycling detection: consecutive goals in a small set of bins
    # Count how many times the robot returns to a bin it just left
    cycle_metrics = _detect_cycles(goals)

    # Penalty saturation analysis
    penalty_saturation_events = sum(
        1 for g in goals
        if g["clamped_revisit_count"] >= 3 and g["normalized_revisit_penalty"] >= 1.0
    )

    # Candidate starvation: goals with only 1 candidate
    starvation_count = sum(1 for g in goals if g["candidate_count"] <= 1)

    # Information gain stagnation: goals with very low gain
    low_gain_count = sum(1 for g in goals if g["raw_information_gain"] < 50)

    # All candidates equal-score (score == -1.5 is the saturated pattern)
    saturated_score_count = sum(1 for g in goals if g["final_score"] <= -1.5)

    # Count bin transitions for cycle detection
    bin_sequence = [g["bin"] for g in goals]
    # A "back-and-forth" is when bin[i] == bin[i-2] and bin[i] != bin[i-1]
    back_and_forth = 0
    for i in range(2, len(bin_sequence)):
        if bin_sequence[i] == bin_sequence[i - 2] and bin_sequence[i] != bin_sequence[i - 1]:
            back_and_forth += 1

    return {
        "total_goals": len(goals),
        "unique_spatial_bins": len(bin_counts),
        "max_visits_per_bin": max(bin_counts.values()),
        "bin_visit_distribution": {str(k): v for k, v in bin_counts.most_common()},
        "max_clamped_revisit_per_bin": max_revisits_per_bin,
        "adjacent_goal_distances_m": {
            "min": round(min(adj_distances), 3) if adj_distances else None,
            "max": round(max(adj_distances), 3) if adj_distances else None,
            "mean": round(sum(adj_distances) / len(adj_distances), 3) if adj_distances else None,
            "values": [round(d, 3) for d in adj_distances],
        },
        "cycle_metrics": cycle_metrics,
        "penalty_saturation": {
            "total_penalty_saturated_goals": penalty_saturation_events,
            "penalty_saturation_rate": round(penalty_saturation_events / len(goals), 3) if goals else 0,
        },
        "candidate_starvation": {
            "single_candidate_goals": starvation_count,
            "single_candidate_rate": round(starvation_count / len(goals), 3) if goals else 0,
        },
        "low_gain_goals": {
            "count": low_gain_count,
            "rate": round(low_gain_count / len(goals), 3) if goals else 0,
        },
        "saturated_score_goals": {
            "count": saturated_score_count,
            "rate": round(saturated_score_count / len(goals), 3) if goals else 0,
        },
        "back_and_forth_transitions": back_and_forth,
        "bin_sequence_indices": _bin_index_sequence(bin_sequence),
    }


def _detect_cycles(goals):
    """Identify cycling patterns in the goal sequence."""
    bin_sequence = [g["bin"] for g in goals]
    if not bin_sequence:
        return {}

    # Find the dominant cycle (most common 2-step and 3-step patterns)
    two_step = Counter()
    three_step = Counter()

    for i in range(len(bin_sequence)):
        if i + 1 < len(bin_sequence):
            two_step[bin_sequence[i], bin_sequence[i + 1]] += 1
        if i + 2 < len(bin_sequence):
            three_step[bin_sequence[i], bin_sequence[i + 1], bin_sequence[i + 2]] += 1

    # If the robot is cycling, we'll see repeating patterns
    most_common_2 = two_step.most_common(3)
    most_common_3 = three_step.most_common(3)

    # Find contiguous cycle runs: longest stretch where bins repeat in a small set
    unique_bins_in_sequence = len(set(bin_sequence))
    # If only ~3 bins are used for a long stretch, likely a cycle

    # Cycle ratio: what fraction of goals are in the top-3 most-visited bins
    bin_counts = Counter(bin_sequence)
    top3_visits = sum(count for _, count in bin_counts.most_common(3))
    cycle_ratio = top3_visits / len(bin_sequence) if bin_sequence else 0

    return {
        "most_common_2step_patterns": [
            {"pattern": [list(p[0][0]), list(p[0][1])], "count": p[1]}
            for p in most_common_2
        ],
        "most_common_3step_patterns": [
            {"pattern": [list(p[0][0]), list(p[0][1]), list(p[0][2])], "count": p[1]}
            for p in most_common_3
        ],
        "unique_bins_in_sequence": len(set(bin_sequence)),
        "top3_bins_cover_ratio": round(cycle_ratio, 3),
    }


def _bin_index_sequence(bin_sequence):
    """Map bins to integer indices for sequence visualisation."""
    unique = sorted(set(bin_sequence), key=lambda b: (b[0], b[1]))
    mapping = {b: i for i, b in enumerate(unique)}
    return [mapping[b] for b in bin_sequence]


def classify_failure(goals_metrics, dwb_count, blacklist_count, benchmark):
    """Classify the outlier into one or more failure modes."""
    findings = []

    # A. Penalty saturation
    ps = goals_metrics.get("penalty_saturation", {})
    if ps.get("penalty_saturation_rate", 0) > 0.3:
        findings.append({
            "type": "A_REVISIT_PENALTY_SATURATION",
            "severity": "HIGH",
            "evidence": (
                f"{ps.get('total_penalty_saturated_goals')}/{goals_metrics.get('total_goals', 0)} "
                f"goals ({ps.get('penalty_saturation_rate', 0)*100:.0f}%) had revisit penalty "
                f"clamped at max_revisit_count=3 — scoring lost all discriminative power."
            ),
        })

    # B. Revisit radius too small
    adj = goals_metrics.get("adjacent_goal_distances_m", {})
    if adj.get("values"):
        # Check if many adjacent distances are between 0.5 and 1.0 (just outside revisit_radius=0.5)
        near_miss = sum(1 for d in adj["values"] if 0.5 < d < 1.0)
        total_adj = len(adj["values"])
        near_miss_rate = near_miss / total_adj if total_adj > 0 else 0
        if near_miss_rate > 0.3:
            findings.append({
                "type": "B_REVISIT_RADIUS_TOO_SMALL",
                "severity": "MEDIUM",
                "evidence": (
                    f"{near_miss}/{total_adj} ({near_miss_rate*100:.0f}%) adjacent goals are "
                    f"0.5-1.0 m apart — just outside revisit_radius=0.5 m, allowing undetected revisits."
                ),
            })

    # C. Frontier churn (SLAM updates creating new frontiers)
    low_gain = goals_metrics.get("low_gain_goals", {})
    if low_gain.get("rate", 0) > 0.3:
        findings.append({
            "type": "C_FRONTIER_CHURN",
            "severity": "MEDIUM",
            "evidence": (
                f"{low_gain.get('count')}/{goals_metrics.get('total_goals', 0)} goals "
                f"({low_gain.get('rate', 0)*100:.0f}%) had raw_information_gain < 50 — "
                f"frontiers may be regenerated by SLAM updates faster than they are consumed."
            ),
        })

    # D. Heading oscillation (back-and-forth transitions)
    bf = goals_metrics.get("back_and_forth_transitions", 0)
    total = goals_metrics.get("total_goals", 0)
    if total > 0 and bf / total > 0.3:
        findings.append({
            "type": "D_HEADING_OSCILLATION",
            "severity": "LOW",
            "evidence": (
                f"{bf}/{total} goals ({bf/total*100:.0f}%) are back-and-forth transitions — "
                f"robot may be turning 180° frequently, which incurs unrewarded path cost."
            ),
        })

    # E. Infrastructure anomaly
    if dwb_count > 10:
        findings.append({
            "type": "E_INFRASTRUCTURE_ANOMALY",
            "severity": "MEDIUM",
            "evidence": (
                f"DWB reported '{dwb_count} No valid trajectories' errors — "
                f"local planner struggled, potentially due to narrow passages or pose uncertainty."
            ),
        })

    # F. Insufficient evidence (catch-all)
    if not findings:
        findings.append({
            "type": "F_INSUFFICIENT_EVIDENCE",
            "severity": "LOW",
            "evidence": (
                "None of the predefined patterns clearly match. "
                "Manual log inspection recommended."
            ),
        })

    return findings


# ── Output ────────────────────────────────────────────────────────────────────


def write_json(output, data):
    """Write structured JSON output."""
    output.write_text(json.dumps(data, indent=2, default=str))
    return output


def write_md_report(output, data, benchmark, dwb_info):
    """Write human-readable markdown diagnostic report."""
    goals = data["goal_events"]
    m = data["metrics"]
    findings = data["failure_classification"]

    lines = []
    _w = lines.append

    _w("# Stage 2B Outlier Analysis: Run 4")
    _w("")
    _w(f"- **Status**: {benchmark.get('run_status', 'N/A')}")
    _w(f"- **TTC**: {benchmark.get('completion_time_seconds', 'N/A')} s")
    _w(f"- **Goals dispatched**: {benchmark.get('goals_dispatched', 'N/A')}")
    _w(f"- **Revisit rate**: {benchmark.get('revisit_rate', 'N/A')} %")
    _w(f"- **Strategy**: {benchmark.get('selection_strategy', 'N/A')}")
    _w("")

    _w("## Goal Sequence")
    _w("")
    _w("| # | Timestamp | Bin | Candidate | Raw Gain | Clamped Revisit | Score | Result |")
    _w("|---|-----------|-----|-----------|----------|----------------|-------|--------|")

    for i, g in enumerate(goals, 1):
        ts = ""
        if g.get("timestamp_epoch"):
            ts = f"{g['timestamp_epoch']:.1f}"
        bx, by = g.get("bin", (0, 0))
        _w(
            f"| {i} | {ts} | ({bx:.1f}, {by:.1f}) | {g.get('candidate_count', '?')} | "
            f"{g.get('raw_information_gain', '?')} | {g.get('clamped_revisit_count', '?')} | "
            f"{g.get('final_score', '?')} | {g.get('navigation_result', '?')} |"
        )
    _w("")

    _w("## Pattern Metrics")
    _w("")
    _w(f"| Metric | Value |")
    _w(f"|--------|-------|")
    _w(f"| Total goals | {m.get('total_goals', 'N/A')} |")
    _w(f"| Unique spatial bins | {m.get('unique_spatial_bins', 'N/A')} |")
    _w(f"| Max visits to one bin | {m.get('max_visits_per_bin', 'N/A')} |")
    _w(f"| Penalty-saturated goals | {m.get('penalty_saturation', {}).get('total_penalty_saturated_goals', 'N/A')} |")
    _w(f"| Single-candidate goals | {m.get('candidate_starvation', {}).get('single_candidate_goals', 'N/A')} |")
    _w(f"| Back-and-forth transitions | {m.get('back_and_forth_transitions', 'N/A')} |")
    _w(f"| DWB 'No valid trajectories' | {dwb_info.get('dwb_no_valid_trajectories', 'N/A')} |")
    _w(f"| Blacklist insertions | {dwb_info.get('blacklist_insertions', 'N/A')} |")
    _w("")

    _w("### Adjacent Goal Distances")
    _w("")
    adj = m.get("adjacent_goal_distances_m", {})
    _w(f"- Min: {adj.get('min', 'N/A')} m")
    _w(f"- Max: {adj.get('max', 'N/A')} m")
    _w(f"- Mean: {adj.get('mean', 'N/A')} m")
    _w("")

    _w("### Bin Visit Distribution")
    _w("")
    _w("| Bin | Visits |")
    _w("|-----|--------|")
    for bin_str, count in m.get("bin_visit_distribution", {}).items():
        # bin_str is "(bx, by)" from Counter
        _w(f"| {bin_str} | {count} |")
    _w("")

    _w("### Bin Sequence (index)")
    _w("")
    seq = m.get("bin_sequence_indices", [])
    if seq:
        _w(f"```")
        _w(f"Goal index: {''.join(str(i % 10) for i in range(1, len(seq)+1))}")
        _w(f"Bin idx:    {''.join(str(s) for s in seq)}")
        _w(f"```")
        _w("")
        unique_bins = sorted(set(seq))
        bin_idx_info = {}
        for g in goals:
            bx, by = g["bin"]
            i = seq[goals.index(g)]
            if i not in bin_idx_info:
                bin_idx_info[i] = f"({bx:.2f}, {by:.2f})"
        _w("| Index | Bin Center |")
        _w("|-------|------------|")
        for i in sorted(bin_idx_info):
            _w(f"| {i} | {bin_idx_info[i]} |")
        _w("")

    _w("## Failure Classification")
    _w("")
    for f in findings:
        _w(f"### {f['type']} (severity: {f['severity']})")
        _w(f"")
        _w(f"{f['evidence']}")
        _w("")

    _w("## Observation Narrative")
    _w("")
    _w(_build_narrative(goals, m, dwb_info))
    _w("")

    _w("## Recommended Next Step")
    _w("")
    _w(_build_recommendation(findings))
    _w("")

    _w("---")
    _w("Generated by analyze_stage2b_outlier.py")

    output.write_text("\n".join(lines))
    return output


def _build_narrative(goals, metrics, dwb_info):
    """Build a concise narrative of what happened."""
    parts = []

    if not goals:
        return "No goal events parsed."

    # Phase 1: initial exploration
    sg = [g for g in goals if g.get("navigation_result") == "SUCCEEDED"]
    ab = [g for g in goals if g.get("navigation_result") == "ABORTED"]

    parts.append(
        f"The robot dispatched {len(goals)} goals ({len(sg)} succeeded, {len(ab)} aborted)."
    )

    # Identify the cycling phase
    ps = metrics.get("penalty_saturation", {})
    if ps.get("total_penalty_saturated_goals", 0) > 0:
        sat_goals = [g for g in goals if g.get("clamped_revisit_count", 0) >= 3]
        if sat_goals:
            first_sat = sat_goals[0]
            last_sat = sat_goals[-1]
            sat_bins = set(g["bin"] for g in sat_goals)
            parts.append(
                f"After initial exploration, the robot entered a local cycle of "
                f"{len(sat_goals)} goals spanning {len(sat_bins)} spatial bins "
                f"(epoch {first_sat.get('timestamp_epoch', 0):.0f}–{last_sat.get('timestamp_epoch', 0):.0f}). "
                f"During this phase revisit penalty was saturated at max_revisit_count=3 "
                f"for all candidates, and only 1 candidate was available per dispatch — "
                f"the scoring function could not differentiate choices."
            )

    # Frontier churn mention
    lg = metrics.get("low_gain_goals", {})
    if lg.get("rate", 0) > 0.3 and goals:
        parts.append(
            f"Information gain was near-zero in {lg.get('count')} of {len(goals)} dispatches, "
            f"suggesting SLAM update cycles may have regenerated frontiers faster than "
            f"the robot could consume them (frontier churn)."
        )

    # DWB issues
    dwb_n = dwb_info.get("dwb_no_valid_trajectories", 0)
    if dwb_n > 10:
        parts.append(
            f"DWB reported {dwb_n} 'No valid trajectories' errors, indicating the local "
            f"planner repeatedly struggled to find feasible paths — possibly due to "
            f"narrow passages, collision cost, or pose uncertainty in the local costmap."
        )

    # The breakout
    breakout_goals = [g for g in goals if g.get("candidate_count", 0) > 3 and g.get("raw_information_gain", 0) > 100]
    if breakout_goals:
        first_breakout = breakout_goals[0]
        parts.append(
            f"The cycle broke at goal index {goals.index(first_breakout) + 1} "
            f"(epoch {first_breakout.get('timestamp_epoch', 0):.0f}) when "
            f"{first_breakout.get('candidate_count', '?')} candidates became available "
            f"with revisit=0 — the scoring function could once again discriminate. "
            f"The robot then completed exploration rapidly."
        )

    return " ".join(parts)


def _build_recommendation(findings):
    """Generate a recommended next action based on classification."""
    types = {f["type"] for f in findings}

    if "A_REVISIT_PENALTY_SATURATION" in types:
        return (
            "**Primary finding: revisit penalty saturation.** "
            "The robot entered a local region where all candidates had revisit=3 (max) and "
            "only 1 candidate was available. The scoring function lost all discriminative power. "
            "Two approaches to consider: (1) increase `max_revisit_count` to 5 or 7 to delay "
            "saturation; (2) increase `revisit_radius_meters` from 0.50 to 0.75-1.00 so that "
            "nearby bins share revisit history, preventing the cycle from forming. "
            "**Do not change both simultaneously** — test one variable at a time."
        )
    elif "D_HEADING_OSCILLATION" in types:
        return (
            "**Primary finding: heading oscillation.** "
            "The robot frequently reversed direction between consecutive goals. "
            "Consider adding a `turn_cost` term to the scoring function to penalise "
            "large heading changes. This is a structural scoring change and should "
            "be implemented as a separate stage (Stage 2C or 2D), not a parameter tweak."
        )
    else:
        return (
            "**No single dominant failure mode identified.** "
            "Manual inspection of the full log and rosbag is recommended before "
            "deciding on parameter changes."
        )


# ── Main ──────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Analyse Stage 2B outlier run"
    )
    parser.add_argument(
        "--run-dir", required=True,
        help="Path to run directory (e.g. ~/stage2b_info_revisit_formal/run_4)"
    )
    parser.add_argument(
        "--output-dir", default=None,
        help="Output directory (defaults to <run-dir>/analysis)"
    )
    args = parser.parse_args()

    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.exists():
        print(f"Error: run directory not found: {run_dir}", file=sys.stderr)
        sys.exit(1)

    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else run_dir / "analysis"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Find the latest attempt
    attempt_dirs = sorted(
        [d for d in run_dir.iterdir() if d.name.startswith("attempt_")]
    )
    if not attempt_dirs:
        print(f"Error: no attempt directories in {run_dir}", file=sys.stderr)
        sys.exit(1)

    attempt_dir = attempt_dirs[-1]
    print(f"[analyze_stage2b_outlier] Using attempt: {attempt_dir}")

    # Load benchmark JSON
    json_path = attempt_dir / "benchmark_results.json"
    benchmark = load_benchmark_json(json_path)
    if benchmark is None:
        print(f"Warning: benchmark_results.json not found in {attempt_dir}")
        benchmark = {}

    # Parse frontier log
    log_path = attempt_dir / "frontier_explorer.log"
    if not log_path.exists():
        print(f"Error: frontier_explorer.log not found in {attempt_dir}", file=sys.stderr)
        sys.exit(1)

    goals = parse_frontier_log(log_path)
    print(f"[analyze_stage2b_outlier] Parsed {len(goals)} goal events from {log_path.name}")

    blacklist_info = parse_blacklist_events(log_path)

    # Parse simulation log
    sim_log_path = attempt_dir / "simulation.log"
    dwb_info = parse_dwb_errors(sim_log_path)

    # Merge blacklist info into dwb_info for output
    dwb_info.update(blacklist_info)

    # Compute metrics
    metrics = compute_pattern_metrics(goals)

    # Classify failure
    findings = classify_failure(
        metrics,
        dwb_info.get("dwb_no_valid_trajectories", 0),
        blacklist_info.get("blacklist_insertions", 0),
        benchmark,
    )

    # Build output data
    output_data = {
        "run_dir": str(run_dir),
        "benchmark": benchmark,
        "goal_events": goals,
        "metrics": metrics,
        "dwb_and_blacklist": dwb_info,
        "failure_classification": findings,
    }

    # Write JSON
    json_out = output_dir / "stage2b_run4_outlier.json"
    write_json(json_out, output_data)
    print(f"[analyze_stage2b_outlier] JSON written to {json_out}")

    # Write Markdown
    md_out = output_dir / "stage2b_run4_outlier.md"
    write_md_report(md_out, output_data, benchmark, dwb_info)
    print(f"[analyze_stage2b_outlier] Report written to {md_out}")

    # Summary
    print(f"\nFailure classification:")
    for f in findings:
        print(f"  {f['type']} ({f['severity']})")

    print("\nDone.")


if __name__ == "__main__":
    main()
