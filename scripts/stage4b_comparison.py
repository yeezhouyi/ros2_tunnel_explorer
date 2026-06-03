#!/usr/bin/env python3
"""Stage 4B ablation comparison: 3D baseline vs risk-only vs centerline-only vs full."""

import json
import os
import sys
from pathlib import Path
from collections import defaultdict


def load_runs(base_dir: str) -> list[dict]:
    runs = []
    path = Path(os.path.expanduser(base_dir))
    if not path.exists():
        print(f"WARNING: {base_dir} not found", file=sys.stderr)
        return runs
    for d in sorted(path.iterdir()):
        if not d.is_dir():
            continue
        jp = d / "attempt_01" / "benchmark_results.json"
        if jp.exists():
            with open(jp) as f:
                runs.append(json.load(f))
    return runs


def mean(vals):
    return sum(vals) / len(vals) if vals else 0.0


def median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


def completion_pct(runs):
    if not runs:
        return 0.0, ""
    expl = sum(1 for r in runs if (
        r.get("terminal_state") == "COMPLETED"
        or r.get("run_status") == "COMPLETED"
        or r.get("stable_completion_detected") is True
        or r.get("completion_candidate_detected") is True
    ))
    harness = sum(1 for r in runs if (
        r.get("terminal_state") == "COMPLETED"
        or r.get("run_status") == "COMPLETED"
        or r.get("stable_completion_detected") is True
    ))
    note = f" ({expl - harness} explorer-completed)" if expl > harness else ""
    return expl / len(runs) * 100.0, note


def summarize(label, runs):
    n = len(runs)
    if n == 0:
        return {"label": label, "runs": 0}
    c_pct, c_note = completion_pct(runs)
    nav = [float(r.get("navigation_goal_success_rate", 0)) for r in runs]
    revisits = [float(r.get("revisit_rate", 0)) for r in runs]
    bins = [int(r.get("unique_goal_bins", 0)) for r in runs]
    probe_ok = sum(int(r.get("recovery_probe_succeeded", 0)) for r in runs)
    probe_total = sum(int(r.get("recovery_probe_dispatched", 0)) for r in runs)
    ttc = [int(r.get("completion_time_seconds", 0)) for r in runs if r.get("completion_time_seconds", 0) > 0]
    return {
        "label": label,
        "runs": n,
        "completion_pct": c_pct,
        "completion_note": c_note,
        "median_nav": median(nav),
        "mean_nav": mean(nav),
        "median_revisit": median(revisits),
        "mean_revisit": mean(revisits),
        "mean_bins": mean(bins),
        "probe_ok": probe_ok,
        "probe_total": probe_total,
        "median_ttc": median(ttc) if ttc else 0,
        "mean_ttc": mean(ttc) if ttc else 0,
    }


def main():
    base = os.path.expanduser("~/stage4b_benchmark")
    variants = {
        "Stage 3D baseline": load_runs(f"{base}/stage3d_baseline"),
        "4B risk-only": load_runs(f"{base}/risk_only"),
        "4B centerline-only": load_runs(f"{base}/centerline_only"),
        "4B full (risk+centerline)": load_runs(f"{base}/full"),
    }

    summaries = {k: summarize(k, v) for k, v in variants.items()}

    if all(s["runs"] == 0 for s in summaries.values()):
        print("No benchmark results found. Run benchmarks first:")
        print(f"  scripts/stage4b_comparison.py expects results in {base}/{{stage3d,risk_only,centerline_only,full}}/")
        return

    print()
    print("## Stage 4B: Tunnel-Aware Scoring — Ablation Comparison")
    print()
    print("| Variant | Runs | Completion | Mean TTC | Median TTC | Mean Revisit | Recovery Probes | Nav2 Success | Mean Unique Bins |")
    print("|---------|------|------------|----------|------------|--------------|-----------------|--------------|------------------|")

    for label in ["Stage 3D baseline", "4B risk-only", "4B centerline-only", "4B full (risk+centerline)"]:
        s = summaries[label]
        if s["runs"] == 0:
            print(f"| {label} | 0 | — | — | — | — | — | — | — |")
            continue
        probes = f"{s['probe_ok']}/{s['probe_total']}" if s['probe_total'] > 0 else "—"
        print(
            f"| {label} | {s['runs']} | {s['completion_pct']:.0f}%{s['completion_note']} "
            f"| {s['mean_ttc']:.0f}s | {s['median_ttc']:.0f}s "
            f"| {s['mean_revisit']:.1f}% | {probes} "
            f"| {s['mean_nav']:.1f}% | {s['mean_bins']:.1f} |"
        )

    print()

    # Highlight deltas
    d3 = summaries.get("Stage 3D baseline", {})
    d4 = summaries.get("4B full (risk+centerline)", {})
    if d3.get("runs", 0) > 0 and d4.get("runs", 0) > 0:
        print("### Key Deltas (3D baseline → 4B full)")
        print()
        for metric, fmt in [("mean_revisit", "Mean revisit: {:.1f}% → {:.1f}%"),
                             ("mean_bins", "Mean unique bins: {:.1f} → {:.1f}"),
                             ("mean_ttc", "Mean TTC: {:.0f}s → {:.0f}s")]:
            v3 = d3.get(metric, 0)
            v4 = d4.get(metric, 0)
            print(f"- " + fmt.format(v3, v4))
        print()


if __name__ == "__main__":
    main()
