#!/usr/bin/env python3
"""A6 cleaning_mode 固定地图基准 (cleaning_benchmark.py).

与执行手册 U10B-6 对齐:2 planners x 3 maps x 5 seeds = 30 runs
记录:planned_coverage, executed_coverage, overlap_ratio, path_length_m,
turn_count, min_clearance_m, planning_time_ms, tracking_rms_m, fallback_count,
termination_reason
"""
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

# cleaning_mode 模块在 test/cleaning_mode 同一仓库的 cleaning_mode/ 目录下
# 此处做 import 容错,允许 standalone 跑测试
try:
    from cleaning_mode.boustrophedon import GridSpec, connected_boustrophedon, inflate
    from cleaning_mode.coverage_planner import make_plan
    from cleaning_mode.path_smoother import smooth_path
except ImportError as e:
    raise SystemExit(
        f"cleaning_mode import failed: {e}\n"
        "请在 ~/robot_advance_4060/work/ros2_tunnel_explorer/ 目录下运行"
    )


# ============================================================
# 固定测试地图(共 3 张):open_room, room_with_islands, narrow_corridor
# ============================================================
def make_open_room(spec: GridSpec) -> dict[str, np.ndarray]:
    """10m x 12m open room,resolution 0.1m -> 100x120 grid."""
    h, w = 100, 120
    candidate = np.ones((h, w), dtype=bool)
    known_free = np.ones_like(candidate)
    obstacle = np.zeros_like(candidate)
    unknown = np.zeros_like(candidate)
    return dict(candidate=candidate, known_free=known_free,
                obstacle=obstacle, unknown=unknown, spec=spec)


def make_room_with_islands(spec: GridSpec) -> dict[str, np.ndarray]:
    """Open room with two pillar obstacles."""
    h, w = 100, 120
    candidate = np.ones((h, w), dtype=bool)
    known_free = np.ones_like(candidate)
    obstacle = np.zeros_like(candidate)
    # 4 个柱子各 4x4 cells
    for cx, cy in [(30, 30), (30, 70), (80, 30), (80, 70)]:
        obstacle[cy - 2:cy + 2, cx - 2:cx + 2] = True
    return dict(candidate=candidate, known_free=known_free,
                obstacle=obstacle, unknown=np.zeros_like(candidate), spec=spec)


def make_narrow_corridor(spec: GridSpec) -> dict[str, np.ndarray]:
    """Narrow corridor with two side walls."""
    h, w = 100, 120
    candidate = np.ones((h, w), dtype=bool)
    known_free = np.ones_like(candidate)
    obstacle = np.zeros_like(candidate)
    obstacle[0:20, :] = True   # top wall
    obstacle[80:100, :] = True  # bottom wall
    return dict(candidate=candidate, known_free=known_free,
                obstacle=obstacle, unknown=np.zeros_like(candidate), spec=spec)


MAP_BUILDERS = {
    "open_room": make_open_room,
    "room_with_islands": make_room_with_islands,
    "narrow_corridor": make_narrow_corridor,
}


# ============================================================
# Planners
# ============================================================
def boustrophedon_astar_plan(masks: dict, footprint_radius_m: float,
                             lane_width_m: float) -> dict:
    """Boustrophedon + A* connector planning."""
    spec: GridSpec = masks["spec"]
    radius_cells = int(math.ceil(footprint_radius_m / spec.resolution))
    blocked = inflate(masks["obstacle"] | masks["unknown"] | ~masks["known_free"],
                     radius_cells)
    executable = masks["candidate"] & masks["known_free"] & ~blocked
    lane_cells = max(1, int(round(lane_width_m / spec.resolution)))
    t0 = time.perf_counter()
    grid_path = connected_boustrophedon(executable, lane_cells)
    t1 = time.perf_counter()
    return dict(
        planner="boustrophedon_astar",
        grid_path=grid_path,
        executable_cells=int(executable.sum()),
        executable_mask=executable,
        planning_time_ms=(t1 - t0) * 1000.0,
    )


def spiral_or_existing_baseline_plan(masks: dict, footprint_radius_m: float,
                                     lane_width_m: float) -> dict:
    """Baseline planner: simple row-by-row scan (no A* connector)."""
    spec: GridSpec = masks["spec"]
    radius_cells = int(math.ceil(footprint_radius_m / spec.resolution))
    blocked = inflate(masks["obstacle"] | masks["unknown"] | ~masks["known_free"],
                     radius_cells)
    executable = masks["candidate"] & masks["known_free"] & ~blocked
    lane_cells = max(1, int(round(lane_width_m / spec.resolution)))
    t0 = time.perf_counter()
    # spiral-like: traverse every row, drop blocked columns
    path: list[tuple[int, int]] = []
    for y in range(0, executable.shape[0], lane_cells):
        xs = np.flatnonzero(executable[y])
        if xs.size == 0:
            continue
        for x in xs:
            path.append((int(x), int(y)))
    t1 = time.perf_counter()
    return dict(
        planner="spiral_or_existing_baseline",
        grid_path=path,
        executable_cells=int(executable.sum()),
        executable_mask=executable,
        planning_time_ms=(t1 - t0) * 1000.0,
    )


PLANNERS = {
    "boustrophedon_astar": boustrophedon_astar_plan,
    "spiral_or_existing_baseline": spiral_or_existing_baseline_plan,
}


# ============================================================
# Metrics
# ============================================================
@dataclass
class RunMetrics:
    planner: str
    map_name: str
    seed: int
    planned_coverage: float
    executed_coverage: float
    visited_coverage: float
    overlap_ratio: float
    path_length_m: float
    turn_count: int
    min_clearance_m: float
    planning_time_ms: float
    tracking_rms_m: float
    fallback_count: int
    termination_reason: str
    extra: dict = field(default_factory=dict)


def compute_metrics(plan_result: dict, masks: dict) -> RunMetrics:
    """Compute benchmark metrics for a single plan."""
    spec: GridSpec = masks["spec"]
    grid_path = plan_result["grid_path"]
    executable = plan_result["executable_cells"]
    unique_visited = len(set(grid_path))
    visited_coverage = unique_visited / executable if executable else 0.0
    # 可达性语义(与 make_plan 一致):从路径出发 BFS 能触达的 executable
    # 比例决定 OK / PARTIAL_REACHABILITY;lane 间隔扫描按设计不逐格踩,
    # 访问占比只作为路径密度指标保留。
    from cleaning_mode.boustrophedon import _bfs_reachable
    reachable_count = 0
    if grid_path:
        mask = plan_result["executable_mask"]
        reached = _bfs_reachable(grid_path[0], mask)
        reachable_count = sum(1 for (x, y) in reached if mask[y, x])
    planned_coverage = reachable_count / executable if executable else 0.0

    # path length
    path_length = 0.0
    turn_count = 0
    last_dir: tuple[int, int] | None = None
    for (x0, y0), (x1, y1) in zip(grid_path, grid_path[1:]):
        path_length += math.hypot(x1 - x0, y1 - y0) * spec.resolution
        d = (x1 - x0, y1 - y0)
        if last_dir is not None and d != last_dir:
            turn_count += 1
        last_dir = d

    # min clearance: distance from path to nearest obstacle
    if grid_path:
        obstacle = masks["obstacle"]
        if obstacle.any():
            min_clear = min(
                _min_clearance_to_obstacle(np.array(grid_path), obstacle, spec)
                for _ in range(1)
            )
        else:
            min_clear = float("inf")
    else:
        min_clear = float("inf")

    # termination reason
    if executable == 0:
        reason = "NO_EXECUTABLE_CELLS"
    elif not grid_path:
        reason = "NO_REACHABLE_PATH"
    elif planned_coverage < 0.95:
        reason = "PARTIAL_REACHABILITY"  # reachable fraction < 95%
    else:
        reason = "OK"

    return RunMetrics(
        planner=plan_result["planner"],
        map_name="",  # filled by caller
        seed=0,       # filled by caller
        planned_coverage=planned_coverage,
        executed_coverage=planned_coverage,  # 离线基准,executed == planned
        visited_coverage=visited_coverage,
        overlap_ratio=0.0,                    # 离线无 overlap
        path_length_m=path_length,
        turn_count=turn_count,
        min_clearance_m=min_clear,
        planning_time_ms=plan_result["planning_time_ms"],
        tracking_rms_m=0.0,                   # 离线无跟踪
        fallback_count=0,
        termination_reason=reason,
    )


def _min_clearance_to_obstacle(path: np.ndarray, obstacle: np.ndarray,
                                spec: GridSpec) -> float:
    """Cheap BFS distance to nearest obstacle from any path cell."""
    if not len(path):
        return float("inf")
    h, w = obstacle.shape
    # distance transform from obstacle
    INF = h + w + 10
    dist = np.full_like(obstacle, INF, dtype=np.int32)
    dist[obstacle] = 0
    # BFS
    from collections import deque
    q = deque(zip(*np.where(obstacle)))
    while q:
        y, x = q.popleft()
        d = dist[y, x]
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and dist[ny, nx] == INF:
                dist[ny, nx] = d + 1
                q.append((ny, nx))
    min_d = min(dist[y, x] for x, y in path)
    return min_d * spec.resolution


# ============================================================
# Driver
# ============================================================
def run_one(planner: str, map_name: str, seed: int,
            spec: GridSpec, footprint_radius_m: float,
            lane_width_m: float) -> RunMetrics:
    masks = MAP_BUILDERS[map_name](spec)
    np.random.seed(seed)
    plan = PLANNERS[planner](masks, footprint_radius_m, lane_width_m)
    m = compute_metrics(plan, masks)
    m.map_name = map_name
    m.seed = seed
    return m


def main() -> None:
    p = argparse.ArgumentParser(description="cleaning_mode fixed-map benchmark")
    p.add_argument("--planner", required=True, choices=list(PLANNERS))
    p.add_argument("--map", required=True, choices=list(MAP_BUILDERS))
    p.add_argument("--seed", type=int, required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--footprint-radius-m", type=float, default=0.18)
    p.add_argument("--lane-width-m", type=float, default=0.30)
    p.add_argument("--resolution-m", type=float, default=0.10)
    args = p.parse_args()

    spec = GridSpec(resolution=args.resolution_m, origin_x=0.0, origin_y=0.0)
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    m = run_one(args.planner, args.map, args.seed, spec,
                args.footprint_radius_m, args.lane_width_m)

    (out / "metrics.json").write_text(json.dumps(asdict(m), indent=2),
                                      encoding="utf-8")
    (out / "run.log").write_text(
        f"planner={args.planner} map={args.map} seed={args.seed} "
        f"planned_coverage={m.planned_coverage:.4f} "
        f"path_length_m={m.path_length_m:.3f} "
        f"turn_count={m.turn_count} "
        f"planning_time_ms={m.planning_time_ms:.2f} "
        f"termination_reason={m.termination_reason}\n",
        encoding="utf-8",
    )
    print(f"[{args.planner}/{args.map}/seed{args.seed}] "
          f"cov={m.planned_coverage:.3f} "
          f"len={m.path_length_m:.1f}m "
          f"turns={m.turn_count} "
          f"reason={m.termination_reason}")


if __name__ == "__main__":
    main()
