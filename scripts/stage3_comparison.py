#!/usr/bin/env python3
"""Generate Stage 3C vs Stage 3D comparison table from archived benchmarks."""

import json
import os
import sys
from pathlib import Path

STAGE3C_DIR = "artifacts/stage3c_branching_y_failed_eval/results"
STAGE3D_DIR = "artifacts/stage3d_entrance_loop_recovery/results"


def load_runs(results_dir: str) -> list[dict]:
    runs = []
    base = Path(results_dir)
    if not base.exists():
        print(f"WARNING: {results_dir} not found", file=sys.stderr)
        return runs
    for run_dir in sorted(base.iterdir()):
        if not run_dir.is_dir() or "dry_run" in run_dir.name:
            continue
        json_path = run_dir / "attempt_01" / "benchmark_results.json"
        if json_path.exists():
            with open(json_path) as f:
                runs.append(json.load(f))
    return runs


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def median(values: list[float]) -> float:
    s = sorted(values)
    n = len(s)
    if n == 0:
        return 0.0
    if n % 2 == 1:
        return s[n // 2]
    return (s[n // 2 - 1] + s[n // 2]) / 2.0


def completion_rate(runs: list[dict]) -> tuple[float, str]:
    """Return (pct, note) — uses explorer-level evidence for older runs."""
    if not runs:
        return 0.0, ""
    explorer_completed = sum(
        1 for r in runs
        if r.get("terminal_state") == "COMPLETED"
        or r.get("run_status") == "COMPLETED"
        or r.get("stable_completion_detected") is True
        or r.get("completion_candidate_detected") is True
    )
    harness_completed = sum(
        1 for r in runs
        if r.get("terminal_state") == "COMPLETED"
        or r.get("run_status") == "COMPLETED"
        or r.get("stable_completion_detected") is True
    )
    pct = explorer_completed / len(runs) * 100.0
    note = ""
    if explorer_completed > harness_completed:
        note = f" ({explorer_completed - harness_completed} runs explorer-completed but harness TIMEOUT)"
    return pct, note


def summarize(stage: str, runs: list[dict]) -> dict:
    n = len(runs)
    nav_rates = [float(r.get("navigation_goal_success_rate", 0)) for r in runs]
    revisits = [float(r.get("revisit_rate", 0)) for r in runs]
    bins = [int(r.get("unique_goal_bins", 0)) for r in runs]

    probes = [r for r in runs if r.get("recovery_probe_dispatched", 0) > 0]
    probe_total = sum(int(r.get("recovery_probe_dispatched", 0)) for r in runs)
    probe_ok = sum(int(r.get("recovery_probe_succeeded", 0)) for r in runs)

    c_pct, c_note = completion_rate(runs)
    return {
        "stage": stage,
        "runs": n,
        "completion_pct": c_pct,
        "completion_note": c_note,
        "median_revisit": median(revisits),
        "mean_revisit": mean(revisits),
        "mean_bins": mean(bins),
        "median_nav": median(nav_rates),
        "runs_with_probe": len(probes),
        "probe_success": f"{probe_ok}/{probe_total}",
    }


def main():
    repo = os.environ.get("REPO_DIR", ".")
    c3 = load_runs(os.path.join(repo, STAGE3C_DIR))
    c4 = load_runs(os.path.join(repo, STAGE3D_DIR))

    s3c = summarize("3C", c3)
    s3d = summarize("3D", c4)

    print()
    print("## Stage 3C vs Stage 3D Comparison")
    print()
    print("| Metric | Stage 3C | Stage 3D | Delta |")
    print("|--------|:--------:|:--------:|:-----:|")

    def row(label, c3v, d3v, fmt="{}"):
        d = ""
        try:
            delta = float(d3v) - float(c3v)
            if delta > 0:
                d = f"+{delta:.1f}"
            elif delta < 0:
                d = f"{delta:.1f}"
            else:
                d = "0"
        except (ValueError, TypeError):
            d = "—"
        print(f"| {label} | {fmt.format(c3v)} | {fmt.format(d3v)} | {d} |")

    row("Runs", s3c["runs"], s3d["runs"])
    row("Completion rate", f"{s3c['completion_pct']:.0f}%{s3c.get('completion_note','')}",
        f"{s3d['completion_pct']:.0f}%{s3d.get('completion_note','')}")
    row("Mean unique bins", f"{s3c['mean_bins']:.1f}",
        f"{s3d['mean_bins']:.1f}")
    row("Median revisit rate", f"{s3c['median_revisit']:.1f}%",
        f"{s3d['median_revisit']:.1f}%")
    row("Mean revisit rate", f"{s3c['mean_revisit']:.1f}%",
        f"{s3d['mean_revisit']:.1f}%")
    row("Median nav success", f"{s3c['median_nav']:.1f}%",
        f"{s3d['median_nav']:.1f}%")
    row("Runs with recovery probe",
        f"{s3c['runs_with_probe']}/{s3c['runs']}",
        f"{s3d['runs_with_probe']}/{s3d['runs']}")
    row("Recovery probe success", s3c["probe_success"],
        s3d["probe_success"])

    print()
    if s3d["runs_with_probe"] > 0:
        print(f"Recovery probe: dispatched in {s3d['runs_with_probe']}/{s3d['runs']} "
              f"runs, succeeded {s3d['probe_success']} times.")
    if s3d["completion_pct"] > s3c["completion_pct"]:
        print(f"Completion improved: {s3c['completion_pct']:.0f}% → "
              f"{s3d['completion_pct']:.0f}% "
              f"(+{s3d['completion_pct'] - s3c['completion_pct']:.0f} pp)")
    print()


if __name__ == "__main__":
    main()
