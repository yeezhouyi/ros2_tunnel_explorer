#!/usr/bin/env python3
"""Regenerate the coverage plan from saved masks (one-command reproducibility).

Reproduces the recorded u9 plan pipeline from the audit masks NPZ:
  - radius_cells is chosen to reproduce the recorded `executable` count
    (plan_stats.json) -- for the b6_chain map that is 6266 @ radius 2;
  - runs coverage_planner.make_plan (which now emits world-smoothed caps
    where caps exist);
  - applies the collinear compression the chain used (250 pts / 69.95 m on
    the b6_chain map, bit-identical to the recorded cleaning_path.json);
  - writes poses json + a plan_stats-like json.

Usage:
  python3 scripts/regen_plan_from_masks.py \
      --masks /path/audit_masks.npz --outdir /path/out \
      [--lane-width-m 0.3] [--radius-cells 2] [--resolution 0.05]
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = str(Path(__file__).resolve().parents[1])
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from cleaning_mode.boustrophedon import GridSpec  # noqa: E402
from cleaning_mode.coverage_planner import make_plan  # noqa: E402
from cleaning_mode.obstacle_inflation import (  # noqa: E402
    apply_inflation_and_exclusions,
)
from cleaning_mode.path_smoother import smooth_path  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--masks", required=True, type=Path)
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--lane-width-m", type=float, default=0.30)
    ap.add_argument("--radius-cells", type=int, default=0,
                    help="0 = auto-match the masks' executable count")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--origin-x", type=float, default=0.0)
    ap.add_argument("--origin-y", type=float, default=0.0)
    args = ap.parse_args()

    z = np.load(args.masks)
    target = int(z["executable"].sum())
    rc = args.radius_cells
    if rc <= 0:
        for cand in (1, 2, 3, 4):
            _, ex = apply_inflation_and_exclusions(
                z["obstacle"], z["unknown"], z["known_free"], z["candidate"],
                radius_cells=cand)
            if int(ex.sum()) == target:
                rc = cand
                break
        if rc <= 0:
            rc = 2

    spec = GridSpec(resolution=args.resolution,
                    origin_x=args.origin_x, origin_y=args.origin_y)
    plan = make_plan(z["candidate"], z["known_free"], z["obstacle"],
                     z["unknown"], spec,
                     footprint_radius_m=rc * args.resolution,
                     lane_width_m=args.lane_width_m)
    if not plan.xy:
        sys.exit("plan empty")

    comp = smooth_path(plan.xy, collinear_threshold_m=0.01)
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(comp, comp[1:]))
    outdir = args.outdir
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / "cleaning_path.json").write_text(json.dumps(
        {"poses": [[float(a), float(b)] for a, b in comp]}))
    (outdir / "plan_stats.json").write_text(json.dumps({
        "masks": str(args.masks),
        "known_free_cells": int(z["known_free"].sum()),
        "executable_cells": int(z["executable"].sum()),
        "radius_cells": rc,
        "path_waypoints": len(comp),
        "path_length_m": round(length, 2),
        "planned_coverage": round(plan.planned_coverage, 4),
        "planner_reason": plan.reason,
    }, indent=1))
    print(f"regen: {len(comp)} pts, L={length:.2f} m, reason={plan.reason}, "
          f"coverage={plan.planned_coverage:.3f}")


if __name__ == "__main__":
    main()
