#!/usr/bin/env python3
"""Coverage strategy comparison: boustrophedon vs standard-planner baselines.

P0 review item: put the boustrophedon(+A*) coverage planner head-to-head with
generic standard-planner baselines on the SAME fixed maps, SAME metrics.

Strategies
----------
boustrophedon_astar      repo planner (cell decomposition + A* connectors),
                         re-used unchanged from cleaning_benchmark.py
row_by_row               repo baseline (lane scan, no connector), idem
astar_greedy             greedy nearest-unvisited-cell walker; every leg is a
                         plain A* (4-connected, Manhattan heuristic) search
rrt_greedy               same greedy walker; every leg is an RRT (random-tree,
                         8-neighbour steering, collision-checked) instead of A*
hybrid_astar_greedy      heading-aware walker over motion primitives
                         (8 headings x {straight, left 45deg, right 45deg}),
                         A* escape search in (cell, heading) space when stuck

All greedy walkers visit EVERY executable cell; the lane planners rely on tool
width.  To compare fairly, every strategy is scored with a footprint-coverage
metric: an executable cell counts as covered when some path cell lies within
footprint_radius.  Reported per strategy x map: footprint coverage, path
length, revisit ratio (path cells revisited beyond the first visit), turn
count and planning time.

Usage
-----
  python3 scripts/coverage_strategy_compare.py --all --outdir DIR
  python3 scripts/coverage_strategy_compare.py --planner rrt_greedy       --map open_room --seed 0 --outdir DIR
"""
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root
sys.path.insert(0, str(Path(__file__).resolve().parent))      # scripts/

try:
    from cleaning_benchmark import (  # noqa: E402
        MAP_BUILDERS, boustrophedon_astar_plan,
        compute_metrics, spiral_or_existing_baseline_plan,
    )
except ImportError as e:
    raise SystemExit(
        f"cleaning_benchmark import failed: {e}"
        " -- run from the repo root or scripts/ directory")

INF = 10 ** 9


# ------------------------------------------------------------------
# shared mask helper (same semantics as cleaning_benchmark planners)
# ------------------------------------------------------------------
def _executable(masks: dict, footprint_radius_m: float) -> np.ndarray:
    from cleaning_mode.boustrophedon import inflate
    spec = masks["spec"]
    radius_cells = int(math.ceil(footprint_radius_m / spec.resolution))
    blocked = inflate(masks["obstacle"] | masks["unknown"] | ~masks["known_free"],
                      radius_cells)
    return masks["candidate"] & masks["known_free"] & ~blocked


def _wavefront_path(start: tuple[int, int], targets: np.ndarray,
                    walkable: np.ndarray) -> list[tuple[int, int]] | None:
    """A* / uniform-cost search from start to the nearest True cell of the targets mask over the walkable
    mask (4-connected).  Vectorised wavefront with
    parent tracking; returns the intermediate cell path (excluding start,
    including target) or None when unreachable."""
    h, w = walkable.shape
    dist = np.full((h, w), INF, dtype=np.int32)
    parent = np.full((h, w), -1, dtype=np.int64)  # index = y * w + x
    sy, sx = start
    if not walkable[sy, sx]:
        return None
    dist[sy, sx] = 0
    frontier = np.zeros((h, w), dtype=bool)
    frontier[sy, sx] = True
    found = False
    while frontier.any() and not found:
        nxt = np.zeros_like(frontier)
        ys, xs = np.nonzero(frontier)
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            ny, nx = ys + dy, xs + dx
            ok = (ny >= 0) & (ny < h) & (nx >= 0) & (nx < w)
            ny, nx = ny[ok], nx[ok]
            better = walkable[ny, nx] & (dist[ny, nx] > dist[ys[ok], xs[ok]] + 1)
            ny, nx = ny[better], nx[better]
            dist[ny, nx] = dist[ys[ok][better], xs[ok][better]] + 1
            parent[ny, nx] = ys[ok][better] * w + xs[ok][better]
            nxt[ny, nx] = True
        hit = nxt & targets
        if hit.any():
            ty, tx = next(zip(*np.nonzero(hit)))
            path = [(int(tx), int(ty))]
            cy, cx = ty, tx
            while (cy, cx) != (sy, sx):
                pi = parent[cy, cx]
                cy, cx = int(pi // w), int(pi % w)
                path.append((cx, cy))
            path.reverse()
            path.pop()          # drop the start cell copy
            return path
        frontier = nxt & (dist < INF) & ~(
            np.zeros_like(frontier))  # keep growing
        frontier &= ~hit
    return None


def _astar_path(start, goal, walkable) -> list[tuple[int, int]] | None:
    """Plain A* (4-connected, Manhattan heuristic) from start to goal."""
    targets = np.zeros_like(walkable)
    targets[goal[1], goal[0]] = True
    return _wavefront_path(start, targets, walkable)


# ------------------------------------------------------------------
# strategy 1: astar_greedy
# ------------------------------------------------------------------
def astar_greedy_plan(masks: dict, footprint_radius_m: float,
                      lane_width_m: float) -> dict:
    """Greedy coverage: repeatedly A* to the nearest not-yet-visited
    executable cell.  Visits every executable cell."""
    spec = masks["spec"]
    t0 = time.perf_counter()
    executable = _executable(masks, footprint_radius_m)
    visited = ~executable.copy()          # True = done
    ys, xs = np.nonzero(executable)
    cur = (int(xs[0]), int(ys[0]))
    path = [cur]
    visited[cur[1], cur[0]] = True
    h, w = executable.shape
    while (~visited & executable).any():
        # fast path: an adjacent unvisited cell needs no search
        stepped = False
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            ny, nx = cur[1] + dy, cur[0] + dx
            if 0 <= ny < h and 0 <= nx < w and executable[ny, nx]                     and not visited[ny, nx]:
                path.append((nx, ny))
                visited[ny, nx] = True
                cur = (nx, ny)
                stepped = True
                break
        if stepped:
            continue
        targets = executable & ~visited
        leg = _wavefront_path(cur, targets, executable)
        if leg is None:
            break                          # remaining cells unreachable
        for c in leg:
            path.append(c)
            visited[c[1], c[0]] = True
        cur = path[-1]
    t1 = time.perf_counter()
    return dict(planner="astar_greedy", grid_path=path,
                executable_cells=int(executable.sum()),
                executable_mask=executable,
                planning_time_ms=(t1 - t0) * 1000.0)


# ------------------------------------------------------------------
# strategy 2: rrt_greedy
# ------------------------------------------------------------------
def _rrt_leg(start, goal, blocked: np.ndarray, max_iter: int = 400,
             step: int = 2) -> list[tuple[int, int]] | None:
    """Small RRT from start to goal on the grid; 8-neighbour steering,
    collision check against the blocked mask.  Returns intermediate cells
    (excluding start, including goal) or None."""
    h, w = blocked.shape
    gy, gx = goal[1], goal[0]
    nodes = [(start[1], start[0])]
    parents = [-1]
    lo_y = max(0, min(start[1], gy) - 6)
    hi_y = min(h - 1, max(start[1], gy) + 6)
    lo_x = max(0, min(start[0], gx) - 6)
    hi_x = min(w - 1, max(start[0], gx) + 6)
    for _ in range(max_iter):
        if np.random.random() < 0.1:
            sy, sx = gy, gx               # goal bias
        else:
            sy = int(np.random.randint(lo_y, hi_y + 1))
            sx = int(np.random.randint(lo_x, hi_x + 1))
        if blocked[sy, sx]:
            continue
        arr = np.array(nodes)
        d2 = (arr[:, 0] - sy) ** 2 + (arr[:, 1] - sx) ** 2
        ni = int(np.argmin(d2))
        ny, nx = arr[ni]
        # steer one step of length <= step towards (sy, sx)
        dy, dx = sy - ny, sx - nx
        dist = max(1, int(round(math.hypot(dy, dx))))
        step_y = int(round(dy / dist * min(step, dist)))
        step_x = int(round(dx / dist * min(step, dist)))
        cy, cx = int(ny + step_y), int(nx + step_x)
        if cy < 0 or cy >= h or cx < 0 or cx >= w or blocked[cy, cx]:
            continue
        nodes.append((cy, cx))
        parents.append(ni)
        if (cy, cx) == (gy, gx):
            path = []
            i = len(nodes) - 1
            while i > 0:
                py, px = nodes[i]
                path.append((px, py))
                i = parents[i]
            path.reverse()
            return path
    return None


def rrt_greedy_plan(masks: dict, footprint_radius_m: float,
                    lane_width_m: float) -> dict:
    """Greedy coverage whose legs are RRT paths instead of A* paths.
    Standard sampling-based planner used as a generic baseline."""
    spec = masks["spec"]
    t0 = time.perf_counter()
    executable = _executable(masks, footprint_radius_m)
    radius_cells = int(math.ceil(footprint_radius_m / spec.resolution))
    blocked = ~executable
    # inflate the blocked mask for RRT collision checking (clearance)
    from cleaning_mode.boustrophedon import inflate
    blocked = inflate(blocked, 1)
    visited = ~executable.copy()
    ys, xs = np.nonzero(executable)
    cur = (int(xs[0]), int(ys[0]))
    path = [cur]
    visited[cur[1], cur[0]] = True
    while (~visited & executable).any():
        targets = executable & ~visited
        tys, txs = np.nonzero(targets)
        d2 = (tys - cur[1]) ** 2 + (txs - cur[0]) ** 2
        i = int(np.argmin(d2))
        goal = (int(txs[i]), int(tys[i]))
        if math.hypot(goal[0] - cur[0], goal[1] - cur[1]) < 2.0:
            leg = [goal]                  # adjacent: straight step
        else:
            leg = _rrt_leg(cur, goal, blocked)
            if leg is None:               # RRT failure -> fall back to A*
                leg = _astar_path(cur, goal, executable)
            if leg is None:
                break
        for c in leg:
            path.append(c)
            visited[c[1], c[0]] = True
        cur = path[-1]
    t1 = time.perf_counter()
    return dict(planner="rrt_greedy", grid_path=path,
                executable_cells=int(executable.sum()),
                executable_mask=executable,
                planning_time_ms=(t1 - t0) * 1000.0)


# ------------------------------------------------------------------
# strategy 3: hybrid_astar_greedy
# ------------------------------------------------------------------
HYBRID_HEADINGS = 8


def _hybrid_escape(cur, heading, targets, walkable):
    """A* escape search in (cell, heading) space.  Primitives: one-cell
    moves {straight, left 45, right 45} plus IN-PLACE rotation (+-45deg,
    cost 0.4) -- the platform is a differential drive, so rotating without
    translating is executable.  Returns (cells, final_heading) or None;
    cells exclude the start and may repeat the start cell when the first
    action is an in-place turn."""
    import heapq
    h, w = walkable.shape
    start = (cur[1], cur[0], heading)
    dist = {start: 0}
    parent = {}
    open_list = [(0.0, start)]
    tys, txs = np.nonzero(targets)
    has_t = tys.size > 0
    while open_list:
        g, (cy, cx, ch) = heapq.heappop(open_list)
        if has_t and targets[cy, cx]:
            path = []
            key = (cy, cx, ch)
            while key in parent:
                py, px, _ = key
                path.append((px, py))
                key = parent[key]
            path.reverse()
            return path, ch
        for dh in (-1, 0, 1):
            nh = (ch + dh) % HYBRID_HEADINGS
            # in-place rotation (differential drive): cost 0.4 per 45deg
            ng_turn = g + 0.4
            key_turn = (cy, cx, nh)
            if key_turn not in dist or ng_turn < dist[key_turn]:
                dist[key_turn] = ng_turn
                parent[key_turn] = (cy, cx, ch)
                best = float(np.min((tys - cy) ** 2 + (txs - cx) ** 2)) \
                    if has_t else 0.0
                heapq.heappush(open_list,
                               (ng_turn + math.sqrt(best), key_turn))
            dy, dx = ((1, 0), (1, 1), (0, 1), (-1, 1),
                      (-1, 0), (-1, -1), (0, -1), (1, -1))[nh]
            ny, nx = cy + dy, cx + dx
            if not (0 <= ny < h and 0 <= nx < w) or not walkable[ny, nx]:
                continue
            ng = g + 1.0 + (abs(dh) * 0.5)
            key = (ny, nx, nh)
            if key not in dist or ng < dist[key]:
                dist[key] = ng
                parent[key] = (cy, cx, ch)
                best = float(np.min((tys - ny) ** 2 + (txs - nx) ** 2)) \
                    if has_t else 0.0
                heapq.heappush(open_list, (ng + math.sqrt(best), key))
    return None


def hybrid_astar_greedy_plan(masks: dict, footprint_radius_m: float,
                             lane_width_m: float) -> dict:
    """Heading-aware greedy coverage over kinodynamic-style motion
    primitives ({straight, +-45deg} per cell); when the primitive set is
    stuck, an A* escape search in (cell, heading) space finds the next
    reachable unvisited cell.  Visits every executable cell."""
    t0 = time.perf_counter()
    executable = _executable(masks, footprint_radius_m)
    visited = ~executable.copy()
    ys, xs = np.nonzero(executable)
    cur = (int(xs[0]), int(ys[0]))
    heading = 2                       # +x direction
    path = [cur]
    visited[cur[1], cur[0]] = True
    moves = ((1, 0), (1, 1), (0, 1), (-1, 1),
             (-1, 0), (-1, -1), (0, -1), (1, -1))
    while (~visited & executable).any():
        stepped = False
        for dh in (0, -1, 1):         # prefer straight, then gentle turns
            nh = (heading + dh) % HYBRID_HEADINGS
            ny, nx = cur[1] + moves[nh][0], cur[0] + moves[nh][1]
            if 0 <= ny < executable.shape[0] and 0 <= nx < executable.shape[1]                     and executable[ny, nx]:
                path.append((nx, ny))
                visited[ny, nx] = True
                cur, heading = (nx, ny), nh
                stepped = True
                break
        if stepped:
            continue
        targets = executable & ~visited
        leg_h = _hybrid_escape(cur, heading, targets, executable)
        if not leg_h:
            break
        leg, head_out = leg_h
        for c in leg:
            path.append(c)
            visited[c[1], c[0]] = True
        cur = path[-1]
        heading = head_out            # keep the real final heading
    t1 = time.perf_counter()
    return dict(planner="hybrid_astar_greedy", grid_path=path,
                executable_cells=int(executable.sum()),
                executable_mask=executable,
                planning_time_ms=(t1 - t0) * 1000.0)


NEW_PLANNERS = {
    "astar_greedy": astar_greedy_plan,
    "rrt_greedy": rrt_greedy_plan,
    "hybrid_astar_greedy": hybrid_astar_greedy_plan,
}
LEGACY_PLANNERS = {
    "boustrophedon_astar": boustrophedon_astar_plan,
    "row_by_row": spiral_or_existing_baseline_plan,
}
ALL_PLANNERS = {**LEGACY_PLANNERS, **NEW_PLANNERS}


# ------------------------------------------------------------------
# extra metrics: footprint coverage + revisit ratio (fair across
# lane-width planners and exhaustive walkers)
# ------------------------------------------------------------------
def footprint_coverage(path, executable: np.ndarray, radius_cells: int) -> float:
    """Fraction of executable cells within radius_cells of some path cell
    (tool-footprint semantics, identical for every strategy)."""
    if not path:
        return 0.0
    h, w = executable.shape
    path_mask = np.zeros((h, w), dtype=bool)
    for x, y in path:
        if 0 <= y < h and 0 <= x < w:
            path_mask[y, x] = True
    cov = np.zeros((h, w), dtype=bool)
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dy * dy + dx * dx > radius_cells * radius_cells:
                continue
            sy = slice(max(0, -dy), h - max(0, dy))
            sx = slice(max(0, -dx), w - max(0, dx))
            ty = slice(max(0, dy), h - max(0, -dy))
            tx = slice(max(0, dx), w - max(0, -dx))
            cov[ty, tx] |= path_mask[sy, sx]
    n = int(executable.sum())
    return float((cov & executable).sum() / n) if n else 0.0


def revisit_ratio(path) -> float:
    """Share of path steps that land on an already-visited cell."""
    if len(path) < 2:
        return 0.0
    seen = set()
    revisits = 0
    for c in path:
        if c in seen:
            revisits += 1
        seen.add(c)
    return revisits / max(1, len(path))


def run_compare(planner: str, map_name: str, seed: int, spec,
                footprint_radius_m: float, lane_width_m: float):
    masks = MAP_BUILDERS[map_name](spec)
    np.random.seed(seed)
    plan = ALL_PLANNERS[planner](masks, footprint_radius_m, lane_width_m)
    m = compute_metrics(plan, masks)
    m.map_name = map_name
    m.seed = seed
    spec_res = masks["spec"].resolution
    radius_cells = int(math.ceil(footprint_radius_m / spec_res))
    m.extra["footprint_coverage"] = round(
        footprint_coverage(plan["grid_path"], plan["executable_mask"],
                           radius_cells), 4)
    m.extra["revisit_ratio"] = round(revisit_ratio(plan["grid_path"]), 4)
    m.extra["path_points"] = len(plan["grid_path"])
    return m


# ------------------------------------------------------------------
# driver
# ------------------------------------------------------------------
def main() -> None:
    p = argparse.ArgumentParser(description="coverage strategy comparison")
    p.add_argument("--planner", choices=list(ALL_PLANNERS))
    p.add_argument("--map", choices=list(MAP_BUILDERS))
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--all", action="store_true",
                   help="run the full matrix: 5 planners x 3 maps x 5 seeds")
    p.add_argument("--outdir", required=True)
    p.add_argument("--footprint-radius-m", type=float, default=0.18)
    p.add_argument("--lane-width-m", type=float, default=0.30)
    p.add_argument("--resolution-m", type=float, default=0.10)
    args = p.parse_args()

    from cleaning_mode.boustrophedon import GridSpec
    spec = GridSpec(resolution=args.resolution_m, origin_x=0.0, origin_y=0.0)
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)

    if args.all:
        jobs = [(pl, mp, sd)
                for pl in ALL_PLANNERS
                for mp in MAP_BUILDERS
                for sd in range(5)]
    else:
        if not (args.planner and args.map):
            raise SystemExit("--planner and --map required without --all")
        jobs = [(args.planner, args.map, args.seed)]

    for pl, mp, sd in jobs:
        tag = pl + "_" + mp + "_seed" + str(sd)
        try:
            m = run_compare(pl, mp, sd, spec, args.footprint_radius_m,
                            args.lane_width_m)
        except Exception as e:                      # noqa: BLE001
            print("[" + tag + "] EXCEPTION " + str(e), flush=True)
            (out / tag).mkdir(parents=True, exist_ok=True)
            (out / tag / "error.txt").write_text(str(e), encoding="utf-8")
            continue
        d = out / tag
        d.mkdir(parents=True, exist_ok=True)
        (d / "metrics.json").write_text(json.dumps(asdict(m), indent=2),
                                        encoding="utf-8")
        print("[" + tag + "] fpcov=%.3f len=%.1fm rev=%.3f t=%.0fms reason=%s"
              % (m.extra["footprint_coverage"], m.path_length_m,
                 m.extra["revisit_ratio"], m.planning_time_ms,
                 m.termination_reason), flush=True)

    if not args.all:
        return

    # ---- aggregate ----
    import statistics
    rows = {}
    for pl in ALL_PLANNERS:
        for mp in MAP_BUILDERS:
            sel = []
            for sd in range(5):
                f = out / (pl + "_" + mp + "_seed" + str(sd)) / "metrics.json"
                if f.exists():
                    sel.append(json.loads(f.read_text(encoding="utf-8")))
            if not sel:
                continue
            ok = sum(1 for r in sel if r["termination_reason"] == "OK")
            rows[pl + "|" + mp] = dict(
                n=len(sel), ok=ok,
                fpcov=statistics.median(r["extra"]["footprint_coverage"] for r in sel),
                length=statistics.median(r["path_length_m"] for r in sel),
                revisit=statistics.median(r["extra"]["revisit_ratio"] for r in sel),
                turns=statistics.median(r["turn_count"] for r in sel),
                tms=statistics.median(r["planning_time_ms"] for r in sel),
            )
    (out / "aggregate.json").write_text(
        json.dumps(rows, indent=1), encoding="utf-8")

    md = ["# Coverage strategy comparison (same maps, same metrics)", "",
          "footprint coverage: executable cells within tool radius of the "
          "path; medians over 5 seeds per map.", ""]
    hdr = ("| planner | map | n | OK | footprint_cov | path_len_m | revisit "
           "| turns | planning_ms |\n"
           "|---|---|---|---|---|---|---|---|---|\n")
    md.append(hdr)
    for key, v in sorted(rows.items()):
        pl, mp = key.split("|")
        md.append("| %s | %s | %d | %d | %.3f | %.1f | %.3f | %.0f | %.1f |"
                  % (pl, mp, v["n"], v["ok"], v["fpcov"], v["length"],
                     v["revisit"], v["turns"], v["tms"]))
    (out / "strategy_compare.md").write_text("\n".join(md) + "\n",
                                             encoding="utf-8")
    print("aggregate written:", out / "strategy_compare.md")


if __name__ == "__main__":
    main()
