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
        grid_path = connected_boustrophedon(executable, lane_cells,
                                            collision_check=True)
    except ValueError:
        grid_path = []

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
