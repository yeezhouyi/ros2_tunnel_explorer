#!/usr/bin/env python3
"""Stage 4D.0b: Failure Attribution for baseline benchmark runs.

Reads benchmark_results.json + frontier_explorer.log for each run,
classifies failure type, and outputs a per-run attribution table.
"""
import json
import os
import re
import sys
from pathlib import Path

TOPOLOGIES = [
    "straight_tunnel", "l_turn_tunnel", "branching_tunnel_y",
    "t_junction_tunnel", "dead_end_branch", "loop_tunnel"
]
RUN_IDS = ["01", "02", "03", "04", "05"]


def count_pattern(log_path, pattern):
    """Count occurrences of a regex pattern in a log file."""
    try:
        with open(log_path, "r", errors="replace") as f:
            return len(re.findall(pattern, f.read()))
    except (FileNotFoundError, OSError):
        return 0


def classify_failure(br, log_counts):
    """Classify failure type by priority."""
    # Priority 0: Invalid startup — map complete before explorer started
    if (br.get("goals_dispatched", 0) == 0 and
        br.get("run_status") == "COMPLETED"):
        return "INVALID_STARTUP_COMPLETE_MAP"

    if br.get("run_status") == "COMPLETED":
        return "SUCCESS_LIKELY"

    # Priority 1: Nav2 controller failure
    if br.get("goals_aborted", 0) > 3:
        return "NAV2_FAILURE_208"

    # Priority 2: Frontier starvation / cooldown suppression
    if (br.get("goals_aborted", 0) == 0 and
        log_counts.get("cooldown_suppressed", 0) > 50):
        return "FRONTIER_STARVATION_COOLDOWN"

    # Priority 3: Exploration stuck
    if (br.get("unique_goal_bins", 99) < 3 and
        br.get("revisit_rate", 0) > 60):
        return "EXPLORATION_STUCK"

    # Priority 4: Entrance oscillillation
    if log_counts.get("entrance_oscillation", 0) > 10:
        return "ENTRANCE_OSCILLATION"

    # Priority 5: Unknown
    return "UNKNOWN_FAILURE"


def analyze_run(base_dir, topology, run_id):
    """Analyze a single run and return attribution dict."""
    run_dir = Path(base_dir) / topology / f"run_{run_id}" / "attempt_01"
    json_path = run_dir / "benchmark_results.json"
    log_path = run_dir / "frontier_explorer.log"

    if not json_path.exists():
        return None

    with open(json_path) as f:
        br = json.load(f)

    # Count patterns in explorer log
    log_counts = {
        "cooldown_suppressed": count_pattern(
            log_path, r"All \d+ frontiers suppressed by cooldown"),
        "entrance_oscillation": count_pattern(
            log_path, r"\[EntranceOscillation\]"),
        "error_code_208": count_pattern(
            log_path, r"error_code.*208"),
        "navigation_aborted": count_pattern(
            log_path, r"Navigation canceled|navigation_aborted"),
    }

    failure_type = classify_failure(br, log_counts)

    # Build evidence string
    evidence_parts = []
    if log_counts["cooldown_suppressed"] > 0:
        evidence_parts.append(f"cooldown_suppressed={log_counts['cooldown_suppressed']}")
    if log_counts["entrance_oscillation"] > 0:
        evidence_parts.append(f"oscillation_count={log_counts['entrance_oscillation']}")
    if br.get("goals_aborted", 0) > 0:
        evidence_parts.append(f"aborted={br['goals_aborted']}")
    if log_counts["error_code_208"] > 0:
        evidence_parts.append(f"error_208={log_counts['error_code_208']}")

    return {
        "topology": topology,
        "run_id": run_id,
        "status": br.get("run_status", "?"),
        "goals": br.get("goals_dispatched", 0),
        "aborts": br.get("goals_aborted", 0),
        "unique_bins": br.get("unique_goal_bins", 0),
        "revisit": br.get("revisit_rate", 0),
        "oscillation": log_counts["entrance_oscillation"],
        "cooldown_suppressed": log_counts["cooldown_suppressed"],
        "failure_type": failure_type,
        "evidence": "; ".join(evidence_parts) if evidence_parts else "clean",
    }


def main():
    base_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/stage4d0_reproducibility")

    results = []
    for topo in TOPOLOGIES:
        for run_id in RUN_IDS:
            r = analyze_run(base_dir, topo, run_id)
            if r:
                results.append(r)

    # Print table
    print(f"{'Topology':<20} {'Run':<5} {'Status':<10} {'Goals':<6} "
          f"{'Aborts':<7} {'Bins':<5} {'Revisit':<8} {'Osc':<5} "
          f"{'Cooldown':<10} {'Failure Type':<30} {'Evidence'}")
    print("-" * 140)

    # Count by type
    type_counts = {}
    for r in results:
        t = r["failure_type"]
        type_counts[t] = type_counts.get(t, 0) + 1
        print(f"{r['topology']:<20} {r['run_id']:<5} {r['status']:<10} "
              f"{r['goals']:<6} {r['aborts']:<7} {r['unique_bins']:<5} "
              f"{r['revisit']:<8.1f} {r['oscillation']:<5} "
              f"{r['cooldown_suppressed']:<10} {r['failure_type']:<30} "
              f"{r['evidence']}")

    print()
    print("=== Failure Type Summary ===")
    for t, c in sorted(type_counts.items(), key=lambda x: -x[1]):
        print(f"  {t}: {c}")

    # Per-topology summary
    print()
    print("=== Per-Topology Summary ===")
    for topo in TOPOLOGIES:
        topo_runs = [r for r in results if r["topology"] == topo]
        completed = sum(1 for r in topo_runs if r["status"] == "COMPLETED")
        print(f"  {topo}: {completed}/{len(topo_runs)} completed")
        for r in topo_runs:
            if r["status"] != "COMPLETED":
                print(f"    run_{r['run_id']}: {r['failure_type']} ({r['evidence']})")


if __name__ == "__main__":
    main()
