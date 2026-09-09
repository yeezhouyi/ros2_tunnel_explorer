#!/usr/bin/env python3
"""Residual-coverage identification + budgeted recovery plan (必做2, E-line).

Scope of this first version (fixed by the round ruling): STATIC served map
(cleaning_room_rect), FIXED tool-width assumption (r0.15 adopted = 0.30 m
lane intent; r0.10 reported as the conservative band), FIXED initial
localisation (odom origin = spawn (0,0), mask shift (0,0)).

Steps implemented here (offline, no simulation needed):
  1. rebuild the SERVED-MAP visited mask of a recorded run from its odom
     bag with the seal gauge (same mapping + disc rule as
     audit_b6_triple.visited_mask, shift (0,0));
  2. residual = executable & ~visited  (free cells the run never swept);
  3. decompose into connected domains; drop too-small domains (reported,
     not silently kept) -- a further geometric filter is that the recovery
     route is only built for domains that admit a single pass (see 4);
  4. budgeted recovery plan: for every kept domain take its LONGEST
     in-domain A* pass (residual slivers are one-pass strips; a lane-
     spaced boustrophedon is the wrong geometry for cells narrower than
     the 0.30 m lane), then chain the domain passes in greedy nearest-
     neighbour order over the executable free set; report the total
     extra distance and the budget cap;
  5. closure estimate: theoretical r015/r010 after a perfect execution of
     the recovery plan vs the recorded before value (execution-layer
     delta stays marked 待测 unless a resume sim is run).

Usage (ROS env sourced only for --odom-bag; --visited-npz skips rosbag):
  python3 scripts/plan_residual_recovery.py \
      --masks-npz chain_day68_evidence/rect_plan/audit_masks.npz \
      --odom-bag ~/chain_day68/run6/odom_bag \
      --outdir results/coverage_closure \
      [--radius 0.15] [--footprint-radius-m 0.10]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = str(Path(__file__).resolve().parents[1])
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from cleaning_mode.boustrophedon import (  # noqa: E402
    GridSpec,
    astar,
    decompose,
)


def load_odom_xy(bag_dir: str):
    """/odom samples from an mcap bag (same reader as audit_b6_triple)."""
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id="mcap"),
                ConverterOptions("", ""))
    xy = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == "/odom":
            m = deserialize_message(data, Odometry)
            xy.append((m.pose.pose.position.x, m.pose.pose.position.y))
    return xy


def visited_mask(xy, sx, sy, ox, oy, res, h, w, radius):
    rr = int(math.ceil(radius / res))
    r2 = radius ** 2
    visited = np.zeros((h, w), dtype=bool)
    for (x, y) in xy:
        wx, wy = x + sx, y + sy
        cx = int((wx - ox) / res)
        cy = int((wy - oy) / res)
        for dy in range(-rr, rr + 1):
            for dx in range(-rr, rr + 1):
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < h and 0 <= nx < w:
                    px = ox + (nx + 0.5) * res
                    py = oy + (ny + 0.5) * res
                    if (px - wx) ** 2 + (py - wy) ** 2 <= r2:
                        visited[ny, nx] = True
    return visited


def describe_domain(cells, res, ox, oy):
    ys, xs = np.nonzero(cells)
    area_m2 = round(float(cells.sum()) * res * res, 4)
    w_m = (xs.max() - xs.min() + 1) * res
    h_m = (ys.max() - ys.min() + 1) * res
    cx = ox + (xs.mean() + 0.5) * res
    cy = oy + (ys.mean() + 0.5) * res
    return {
        "cells": int(cells.sum()), "area_m2": area_m2,
        "bbox_m": [round(w_m, 3), round(h_m, 3)],
        "centre_m": [round(float(cx), 3), round(float(cy), 3)],
    }


def longest_in_domain_pass(mask):
    """Longest A* path between boundary points of a connected domain.

    A residual sliver is a one-pass strip, so the recovery pass is the
    longest walkable chord of the domain (a lane-spaced boustrophedon is
    meaningless when the domain is narrower than the lane).
    """
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return []
    cand = set()
    for x in (int(xs.min()), int(xs.max())):
        col = np.nonzero(mask[:, x])[0]
        if len(col):
            cand.add((x, int(col.min())))
            cand.add((x, int(col.max())))
    for y in (int(ys.min()), int(ys.max())):
        row = np.nonzero(mask[y, :])[0]
        if len(row):
            cand.add((int(row.min()), int(y)))
            cand.add((int(row.max()), int(y)))
    pts = list(cand)
    best: list = []
    for a in pts:
        for b in pts:
            if a == b:
                continue
            p = astar(a, b, mask)
            if len(p) > len(best):
                best = p
    return best


def world(gp, spec):
    return (spec.origin_x + (gp[0] + 0.5) * spec.resolution,
            spec.origin_y + (gp[1] + 0.5) * spec.resolution)


def path_length_m(xy) -> float:
    if len(xy) < 2:
        return 0.0
    return float(sum(math.hypot(xy[i + 1][0] - xy[i][0],
                                xy[i + 1][1] - xy[i][1])
                     for i in range(len(xy) - 1)))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--masks-npz", required=True, type=Path)
    ap.add_argument("--odom-bag", type=Path, default=None)
    ap.add_argument("--visited-npz", type=Path, default=None,
                    help="precomputed visited mask (skips rosbag read)")
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--radius", type=float, default=0.15)
    ap.add_argument("--sx", type=float, default=0.0,
                    help="odom->map shift x (seal gauge: 0.0; the npz"
                         " 'shift_x' field is stale generator metadata)")
    ap.add_argument("--sy", type=float, default=0.0)
    ap.add_argument("--run-id", default="recorded_run")
    ap.add_argument("--footprint-radius-m", type=float, default=0.10)
    ap.add_argument("--min-domain-cells", type=int, default=4)
    ap.add_argument("--budget-factor", type=float, default=1.5,
                    help="recovery distance budget cap as multiple of the "
                         "planned recovery path length")
    args = ap.parse_args()

    out = args.outdir
    out.mkdir(parents=True, exist_ok=True)
    z = np.load(args.masks_npz)
    known_free = z["known_free"]
    obstacle = z["obstacle"]
    unknown = z["unknown"]
    candidate = z["candidate"]
    executable = z["executable"]
    res = float(z["res"])
    ox, oy = float(z["ox"]), float(z["oy"])
    sx = args.sx                    # seal gauge (0,0); npz shift is stale
    sy = args.sy
    h, w = executable.shape
    cell_m2 = res * res
    exec_sum = int(executable.sum())

    # ---- 1) visited mask at the reported radius -------------------------
    if args.visited_npz is not None:
        v = np.load(args.visited_npz)
        visited = v["visited"].astype(bool)
        assert visited.shape == executable.shape, \
            "visited shape %s != masks %s" % (visited.shape,
                                              executable.shape)
    else:
        assert args.odom_bag is not None, "need --odom-bag or --visited-npz"
        xy = load_odom_xy(str(args.odom_bag))
        visited = visited_mask(xy, sx, sy, ox, oy, res, h, w, args.radius)
        np.savez(out / ("visited_%.2f.npz" % args.radius), visited=visited)

    vis_exec = int(np.logical_and(visited, executable).sum())
    residual = np.logical_and(executable, np.logical_not(visited))
    res_sum = int(residual.sum())
    if os.environ.get("PRR_DEBUG"):
        print("DBG sx=%.3f sy=%.3f r=%.3f visited_total=%d "
              "visited_exec=%d exec=%d residual=%d"
              % (sx, sy, args.radius, int(visited.sum()), vis_exec,
                 exec_sum, res_sum), file=sys.stderr)

    # ---- 2) connected domains + per-domain recovery pass ------------------
    spec = GridSpec(resolution=res, origin_x=ox, origin_y=oy)
    domains = decompose(residual)
    kept, dropped, passes = [], [], []
    for i, d in enumerate(domains):
        base = describe_domain(d, res, ox, oy)
        base["id"] = "R%02d" % i
        if int(d.sum()) < args.min_domain_cells:
            dropped.append({**base, "reason": "too_small"})
            continue
        gp = longest_in_domain_pass(d)
        if not gp:
            dropped.append({**base, "reason": "no_in_domain_pass"})
            continue
        xy = [world(g, spec) for g in gp]
        kept.append({**base, "mask": d, "pass_waypoints": len(gp),
                     "pass_length_m": round(path_length_m(xy), 3)})
        passes.append((kept[-1], gp))
    keep_mask = np.zeros_like(residual)
    for k in kept:
        keep_mask |= k["mask"]
    kept_sum = int(keep_mask.sum())
    drop_sum = res_sum - kept_sum

    # ---- 3) greedy nearest-neighbour chain over domain passes ---------------
    nav = executable                       # transfers stay inside exec free
    route_gp: list = []
    remaining = list(passes)
    cur = None
    while remaining:
        if cur is None:
            nxt_i = max(range(len(remaining)),
                        key=lambda i: len(remaining[i][1]))
        else:
            nxt_i = min(range(len(remaining)),
                        key=lambda i: _dist2(cur, remaining[i][1][0]))
        k, gp = remaining.pop(nxt_i)
        if cur is not None:
            conn = astar(cur, gp[0], nav)
            route_gp += conn
        route_gp += gp
        cur = gp[-1]
    route_xy = [world(g, spec) for g in route_gp]
    planned_m = round(path_length_m(route_xy), 3)
    budget_m = round(planned_m * args.budget_factor, 3)

    # theoretical closure: a perfect recovery sweeps every KEPT residual
    # cell (pass endpoints lie inside the domain and r0.15 >= sliver width)
    theo_after = round(float(vis_exec + kept_sum) / exec_sum, 4)
    theo_delta = round(float(kept_sum) / exec_sum, 4)

    # cost/effect honest ratio: extra route distance per extra m^2 of
    # covered residual; the user's warning about "endless loops for the
    # last slivers" is exactly what this number exposes.
    extra_area_m2 = round(kept_sum * cell_m2, 4)
    cost_per_m2 = round(planned_m / extra_area_m2, 2) if extra_area_m2 > 0 else 0.0
    main_plan_m = 60.75                          # seal: rect cleaning plan length
    route_overhead_ratio = round(planned_m / main_plan_m, 2)

    before = round(float(vis_exec) / exec_sum, 4)

    # ---- overlay PNG ------------------------------------------------------
    png_ok = False
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[executable] = (215, 215, 215)      # light grey = executable
        img[obstacle] = (60, 60, 60)            # dark grey = obstacle
        img[visited & executable] = (120, 200, 120)   # green = visited
        img[residual] = (210, 60, 60)           # red = residual (unvisited)
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.imshow(img, origin="lower",
                  extent=[ox, ox + w * res, oy, oy + h * res])
        ax.set_title("Residual coverage: %s (exec %d, visited r%.2f %d, "
                     "residual %d)" % (args.run_id, exec_sum, args.radius,
                                       vis_exec, res_sum))
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        if route_xy:
            rx = [p[0] for p in route_xy]
            ry = [p[1] for p in route_xy]
            ax.plot(rx, ry, "-", color=(0.0, 0.0, 0.85), lw=0.8,
                    alpha=0.9, label="recovery route")
            ax.legend(loc="upper right")
        png = out / "residual_overlay.png"
        fig.savefig(png, dpi=110, bbox_inches="tight")
        plt.close(fig)
        png_ok = True
    except Exception as exc:                    # pragma: no cover
        print("PNG skipped:", exc)

    # ---- JSON + report ---------------------------------------------------
    summary = {
        "run_id": args.run_id,
        "radius_m": args.radius,
        "gauge": "served map masks, shift (%.0f, %.0f)" % (sx, sy),
        "executable_cells": exec_sum,
        "visited_exec_cells": vis_exec,
        "coverage_before_r%.2f" % args.radius: before,
        "residual_cells": res_sum,
        "residual_area_m2": round(res_sum * cell_m2, 4),
        "domains_total": len(domains),
        "domains_kept": len(kept),
        "domains_dropped": len(dropped),
        "dropped_cells": drop_sum,
        "kept_cells": kept_sum,
        "recovery_plan": {
            "per_domain_pass_waypoints": sum(k.get("pass_waypoints", 0)
                                             for k in kept),
            "domains_with_pass": len(passes),
            "path_waypoints": len(route_xy),
            "path_length_m": planned_m,
            "distance_budget_m": budget_m,
            "budget_factor": args.budget_factor,
        },
        "theoretical_closure": {
            "coverage_after_perfect_recovery_r%.2f" % args.radius:
                theo_after,
            "delta_r%.2f" % args.radius: theo_delta,
            "note": "planning-layer estimate; execution-layer delta is "
                    "待测 until a resume/recovery sim run is executed",
        },
        "domains_kept_list": [{kk: vv for kk, vv in k.items()
                               if kk != "mask"} for k in kept],
        "domains_dropped_list": dropped,
        "png": "residual_overlay.png" if png_ok else None,
    }
    (out / "recovery_plan.json").write_text(
        json.dumps(summary, indent=1))
    (out / "recovery_path.json").write_text(json.dumps(
        {"xy": [[round(float(p[0]), 4), round(float(p[1]), 4)]
                for p in route_xy]}, indent=1))

    md = [
        "# Residual coverage + budgeted recovery plan (%s)" % args.run_id,
        "",
        "Gauge: served static map cleaning_room_rect masks (executable %d),"
        " shift (%.0f, %.0f), footprint radius r%.2f." % (
            exec_sum, sx, sy, args.radius),
        "",
        "| quantity | value |",
        "|---|---|",
        "| coverage before (r%.2f) | %.4f (%d/%d cells) |"
        % (args.radius, before, vis_exec, exec_sum),
        "| residual cells / area | %d / %.4f m^2 |"
        % (res_sum, res_sum * cell_m2),
        "| residual domains total / kept / dropped | %d / %d / %d |"
        % (len(domains), len(kept), len(dropped)),
        "| dropped cells (too small / no single pass) | %d |" % drop_sum,
        "| recovery route | %d waypoints, %.3f m |"
        % (len(route_xy), planned_m),
        "| distance budget (x%.2f) | %.3f m |"
        % (args.budget_factor, budget_m),
        "| theoretical coverage after perfect recovery (r%.2f) | %.4f"
        " (delta +%.4f) |" % (args.radius, theo_after, theo_delta),
        "",
        "Dropped domains (reported, not silently kept):",
        "",
    ]
    if dropped:
        for d in dropped:
            if "mask" in d:
                d = {kk: vv for kk, vv in d.items() if kk != "mask"}
            md.append("- id=%s cells=%d area=%.4f m^2 centre=%s reason=%s"
                      % (d.get("id", "?"), d["cells"], d["area_m2"],
                         d["centre_m"], d["reason"]))
    else:
        md.append("- (none)")
    budget_note = (
        "- Budget rule: stop when the extra distance exceeds the budget"
        " (x%.2f of the planned recovery length).  The sim side (budgeted"
        " resume execution + before/after audit) is a separate step; its"
        " delta stays 待测 until run." % args.budget_factor)
    md += [
        "",
        "Notes:",
        "- Recovery geometry: residual slivers are one-pass strips, so each"
        " kept domain gets its LONGEST in-domain pass; the passes are then",
        " chained in greedy nearest-neighbour order over the executable"
        " free set (transfers inside free cells only).",
        budget_note,
        "- Cost/effect (planning layer): recovery route %.3f m for"
        " %.4f m^2 of extra coverage = %.2f m/m^2, %.2fx the main plan"
        " length (%.2f m).  This is the 'endless loops for the last"
        " slivers' risk the round ruling warned about, now made visible."
        "  Execution-layer delta stays 待测 until a resume/recovery sim"
        " is run; the budget cap (%s, x%.2f) is the agreed stop rule." % (
            planned_m, extra_area_m2, cost_per_m2, route_overhead_ratio,
            main_plan_m, "%0.3f m" % budget_m, args.budget_factor),
        "",
    ]
    (out / "closure_report.md").write_text("\n".join(md) + "\n")
    print("coverage_before r%.2f: %.4f  residual: %d cells (%.4f m^2)"
          % (args.radius, before, res_sum, res_sum * cell_m2))
    print("recovery route: %d waypoints, %.3f m, budget %.3f m"
          % (len(route_xy), planned_m, budget_m))
    print("theoretical after: %.4f (delta +%.4f)" % (theo_after, theo_delta))
    print("written:", out / "closure_report.md")


def _dist2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


if __name__ == "__main__":
    main()
