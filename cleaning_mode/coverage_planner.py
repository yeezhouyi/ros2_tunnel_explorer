"""覆盖规划器(主入口).

契约:
- 输入 candidate/known_free/obstacle/unknown 掩膜 + GridSpec + 几何参数
- 输出 CleaningPlan {xy, candidate_cells, executable_cells,
                     visited_unique_cells, planned_coverage,
                     path_length_m, reason}
- reason 取值:
    OK                        - 全部可行单元均被覆盖
    NO_EXECUTABLE_CELLS       - 没有任何可行单元
    NO_REACHABLE_PATH         - 路径生成失败(无解)
    PARTIAL_REACHABILITY      - 覆盖率 < 95%
"""
from __future__ import annotations

from dataclasses import dataclass
from math import ceil
import math

import numpy as np

from .boustrophedon import (
    GridSpec,
    _bfs_reachable,
    connected_boustrophedon,
    grid_to_world,
    path_length_m,
)
from .obstacle_inflation import apply_inflation_and_exclusions


@dataclass(frozen=True)
class CleaningPlan:
    """覆盖式清洁规划结果."""

    xy: list[tuple[float, float]]
    candidate_cells: int
    executable_cells: int
    visited_unique_cells: int
    planned_coverage: float
    path_length_m: float
    reason: str


def _apply_world_caps(xy, grid_path, arcs, spec):
    """Replace grid-staircase U-cap spans with continuous semicircular arcs.

    The planner's U-caps are int-rounded cell samples of a small-radius
    semicircle; on the world polyline that is a Manhattan staircase whose
    corners (R~0.03 m) make any smooth tracker reference kinematically
    infeasible.  Each captured cap (from ``connected_boustrophedon(arcs=)``)
    spans ``[start, start+n-1]`` in both ``grid_path`` and ``xy``; we drop
    the staircase interior and splice the analytic arc through the removed
    midpoint, keeping both endpoints.
    """
    if not arcs:
        return xy
    res = spec.resolution
    ox = spec.origin_x
    oy = spec.origin_y
    xy = list(xy)

    def wcell(c):
        return (ox + (c[0] + 0.5) * res, oy + (c[1] + 0.5) * res)

    for m in sorted(arcs, key=lambda a: a["start"], reverse=True):
        i = m["start"]
        n = m["n"]
        if i < 0 or i + n - 1 >= len(xy):
            continue
        e1, e2 = i, i + n - 1
        x1, y1 = xy[e1]
        x2, y2 = xy[e2]
        p1, p2 = m["p1"], m["p2"]
        ccx = (p1[0] + p2[0]) / 2.0
        ccy = (p1[1] + p2[1]) / 2.0
        cwx = ox + (ccx + 0.5) * res
        cwy = oy + (ccy + 0.5) * res
        r = math.hypot(x2 - x1, y2 - y1) / 2.0
        if r <= 1e-9:
            continue
        a1 = math.atan2(y1 - cwy, x1 - cwx)
        a2 = math.atan2(y2 - cwy, x2 - cwx)
        # pick the semicircle side that passes near the removed midpoint
        mid = wcell(grid_path[(e1 + e2) // 2])
        am = math.atan2(mid[1] - cwy, mid[0] - cwx)
        cc = (a2 - a1 + 2.0 * math.pi) % (2.0 * math.pi)
        d = (am - a1 + math.pi) % (2.0 * math.pi) - math.pi
        total = cc if d >= 0.0 else -(2.0 * math.pi - cc)
        steps = max(4, int(round(math.pi * r / res)))
        pts = []
        for k in range(1, steps):
            ang = a1 + total * k / steps
            pts.append((cwx + r * math.cos(ang), cwy + r * math.sin(ang)))
        xy[e1 + 1:e2] = pts
    return xy


def make_plan(
    candidate: np.ndarray,
    known_free: np.ndarray,
    obstacle: np.ndarray,
    unknown: np.ndarray,
    spec: GridSpec,
    footprint_radius_m: float,
    lane_width_m: float,
) -> CleaningPlan:
    """生成覆盖路径.

    Args:
        candidate: 候选覆盖区(布尔 2-D)
        known_free: 已知自由区(布尔 2-D)
        obstacle: 已知障碍(布尔 2-D)
        unknown: 未知区(布尔 2-D)
        spec: 栅格元数据
        footprint_radius_m: 机器人足迹半径(m)
        lane_width_m: 行间距(m)

    Returns:
        CleaningPlan
    """
    masks = [np.asarray(m, dtype=bool) for m in
             (candidate, known_free, obstacle, unknown)]
    if len({m.shape for m in masks}) != 1 or masks[0].ndim != 2:
        raise ValueError("all masks must be 2-D and have identical shapes")
    if spec.resolution <= 0 or footprint_radius_m < 0 or lane_width_m <= 0:
        raise ValueError("invalid metric configuration")

    candidate, known_free, obstacle, unknown = masks
    radius_cells = int(ceil(footprint_radius_m / spec.resolution))
    blocked, executable = apply_inflation_and_exclusions(
        obstacle, unknown, known_free, candidate, radius_cells
    )

    lane_cells = max(1, int(round(lane_width_m / spec.resolution)))
    denominator = int(executable.sum())

    if denominator == 0:
        return CleaningPlan(
            xy=[],
            candidate_cells=int(candidate.sum()),
            executable_cells=0,
            visited_unique_cells=0,
            planned_coverage=0.0,
            path_length_m=0.0,
            reason="NO_EXECUTABLE_CELLS",
        )

    try:
        _arcs: list = []
        grid_path = connected_boustrophedon(executable, lane_cells,
                                            collision_check=True, arcs=_arcs)
    except ValueError:
        grid_path = []
        _arcs = []

    if not grid_path:
        return CleaningPlan(
            xy=[],
            candidate_cells=int(candidate.sum()),
            executable_cells=denominator,
            visited_unique_cells=0,
            planned_coverage=0.0,
            path_length_m=0.0,
            reason="NO_REACHABLE_PATH",
        )

    unique_visited = len(set(grid_path))
    # planned_coverage = 可达性:从路径出发 BFS 能到达的 executable 比例。
    # lane 间隔扫描按设计不逐格踩(覆盖率对全分母恒 < 1),所以 OK/失败
    # 的判据是"规划是否触达全部可行单元",而不是"路径点数占比"。
    reached = _bfs_reachable(grid_path[0], executable)
    reachable_count = sum(1 for (x, y) in reached if executable[y, x])
    coverage = reachable_count / denominator if denominator else 0.0
    xy = grid_to_world(grid_path, spec)
    xy = _apply_world_caps(xy, grid_path, _arcs, spec)
    length = path_length_m(xy)

    if coverage < 0.95:
        reason = "PARTIAL_REACHABILITY"
    else:
        reason = "OK"

    return CleaningPlan(
        xy=xy,
        candidate_cells=int(candidate.sum()),
        executable_cells=denominator,
        visited_unique_cells=unique_visited,
        planned_coverage=coverage,
        path_length_m=length,
        reason=reason,
    )
