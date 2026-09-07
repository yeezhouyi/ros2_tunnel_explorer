"""Boustrophedon 分解与蛇形覆盖路径.

关键设计点(基于交接文档修复):
1. decompose() 把可行栅格按连通性切分;同一行内被障碍断开的 run,即便属于同一上行 cell,
   也会被切分为独立 cell(避免跨过死区连接)。
2. sweep_segments() 在每个 cell 内做蛇形扫描,行间距 spacing_cells。
3. connected_boustrophedon() 用 A* 跨 cell 连接;若连接线穿障,抛 ValueError。
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from heapq import heappop, heappush
from math import hypot
from typing import Iterator

import math

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

GridPoint = tuple[int, int]  # x, y (col, row)


@dataclass(frozen=True)
class GridSpec:
    """栅格地图的元数据."""

    resolution: float
    origin_x: float
    origin_y: float


# ============================================================
# 邻接与连通性
# ============================================================
def _neighbors(point: GridPoint, free: np.ndarray) -> Iterator[GridPoint]:
    x, y = point
    height, width = free.shape
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nx, ny = x + dx, y + dy
        if 0 <= nx < width and 0 <= ny < height and free[ny, nx]:
            yield nx, ny


def inflate(blocked: np.ndarray, radius_cells: int) -> np.ndarray:
    """障碍物欧氏半径膨胀(与 obstacle_inflation 保持一致)."""
    blocked = np.asarray(blocked, dtype=bool)
    if blocked.ndim != 2:
        raise ValueError("blocked must be a 2-D mask")
    if radius_cells < 0:
        raise ValueError("radius_cells must be non-negative")
    if radius_cells == 0:
        return blocked.copy()
    size = 2 * radius_cells + 1
    # pad with False: outside the map is NOT an obstacle, so dilation must
    # not grow inward from the border (test contract: inflate never marks
    # border cells blocked when the obstacle is interior)
    padded = np.pad(blocked, radius_cells, mode="constant", constant_values=False)
    windows = sliding_window_view(padded, (size, size))
    return windows.any(axis=(-2, -1))


def _bfs_reachable(start: GridPoint, free: np.ndarray) -> set[GridPoint]:
    """BFS 从 start 出发,返回所有可达 free 单元."""
    visited: set[GridPoint] = {start}
    q: deque[GridPoint] = deque([start])
    while q:
        cur = q.popleft()
        for nxt in _neighbors(cur, free):
            if nxt not in visited:
                visited.add(nxt)
                q.append(nxt)
    return visited


def decompose(free: np.ndarray) -> list[np.ndarray]:
    """把 free 栅格按连通性切分.

    Returns:
        一个布尔掩膜的列表,每个掩膜代表一个 cell.
    """
    free = np.asarray(free, dtype=bool)
    if free.ndim != 2:
        raise ValueError("free must be a 2-D mask")
    h, w = free.shape
    visited = np.zeros_like(free, dtype=bool)
    cells: list[np.ndarray] = []
    for y in range(h):
        for x in range(w):
            if free[y, x] and not visited[y, x]:
                start: GridPoint = (x, y)
                reachable = _bfs_reachable(start, free)
                cell_mask = np.zeros_like(free, dtype=bool)
                for cx, cy in reachable:
                    cell_mask[cy, cx] = True
                visited |= cell_mask
                cells.append(cell_mask)
    return cells


# ============================================================
# A*
# ============================================================
def astar(start: GridPoint, goal: GridPoint,
          free: np.ndarray) -> list[GridPoint]:
    """A* on 4-connected grid. 若任一端点不在 free,返回 []. 若不可达,返回 [].

    注意:本函数不会检查路径是否穿过障碍(只穿过 free 节点),但也不会越界。
    """
    if start == goal:
        return [start]
    if not free[start[1], start[0]] or not free[goal[1], goal[0]]:
        return []
    queue: list[tuple[float, float, GridPoint]] = []
    heappush(queue, (0.0, 0.0, start))
    parent: dict[GridPoint, GridPoint] = {}
    cost: dict[GridPoint, float] = {start: 0.0}
    while queue:
        _, current_cost, current = heappop(queue)
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
            path.reverse()
            return path
        if current_cost > cost.get(current, float("inf")):
            continue
        for nxt in _neighbors(current, free):
            new_cost = current_cost + 1.0
            if new_cost >= cost.get(nxt, float("inf")):
                continue
            cost[nxt] = new_cost
            parent[nxt] = current
            heuristic = abs(goal[0] - nxt[0]) + abs(goal[1] - nxt[1])
            heappush(queue, (new_cost + heuristic, new_cost, nxt))
    return []


# ============================================================
# 蛇形扫描
# ============================================================
def sweep_segments(cell: np.ndarray,
                   spacing_cells: int) -> list[list[GridPoint]]:
    """在一个 cell 内做蛇形行扫,每 spacing_cells 一行.

    Returns:
        段列表,每段是一条连续 cell 列表(单行内的 free 单元).
    """
    if cell.ndim != 2:
        raise ValueError("cell must be a 2-D mask")
    if spacing_cells < 1:
        raise ValueError("spacing_cells must be positive")
    segments: list[list[GridPoint]] = []
    reverse = False
    # the U-turn arc needs turn_r cells of headroom at the turning end:
    # trim the far end of each run (the arc disc re-covers the corner)
    turn_r = int(math.ceil(spacing_cells / 2.0))
    for y in range(0, cell.shape[0], spacing_cells):
        xs = np.flatnonzero(cell[y])
        if xs.size == 0:
            continue
        # 切分连续 run(同行内被障碍断开)
        split_indices = np.flatnonzero(np.diff(xs) > 1) + 1
        for run in np.split(xs, split_indices):
            lo, hi = int(run[0]), int(run[-1])
            if hi - lo < turn_r:
                continue  # too short to turn within
            if reverse:
                lo = lo + turn_r      # travelling -x: trim low end
            else:
                hi = hi - turn_r      # travelling +x: trim high end
            ordered = list(range(hi, lo - 1, -1)) if reverse \
                else list(range(lo, hi + 1))
            segments.append([(int(x), int(y)) for x in ordered])
            reverse = not reverse
    return segments


# ============================================================
# 跨 cell 蛇形连接
# ============================================================

def _band_uturn(path: list[GridPoint], next_start: GridPoint,
                free: np.ndarray, spacing_cells: int) -> list[GridPoint] | None:
    """Band-transition U-turn maneuver (U9): straight overshoot along the
current travel direction, then a 180-degree semicircular arc of radius
ceil(spacing/2) landing on the next band row, reversing the heading.
Works for aligned serpentine pairs AND furniture-fragmented lanes (the
arc radius derives from the row gap, not from column alignment).
Returns the maneuver points (starting at path[-1]) or None when the arc
region leaves the free mask (caller falls back to A*)."""
    if len(path) < 2:
        return None
    p1 = path[-1]
    dy = next_start[1] - p1[1]
    if abs(dy) < spacing_cells:
        return None  # next lane not a band below
    prev = path[-2]
    tsign = 1 if p1[0] >= prev[0] else -1
    nsign = 1 if dy > 0 else -1
    r = int(math.ceil(abs(dy) / 2.0))
    cx = p1[0] + tsign * r
    cy = p1[1] + nsign * r
    a1 = math.atan2(p1[1] - cy, p1[0] - cx)
    a2 = math.atan2(next_start[1] - cx * 0 - cy + nsign * r - nsign * r,
                    0.0) if False else (math.pi / 2 if nsign > 0 else -math.pi / 2)
    def arc_pts(sign: int) -> list[GridPoint]:
        span = (a2 - a1) % (2 * math.pi) if sign > 0 else (a1 - a2) % (2 * math.pi)
        n = max(12, int(math.ceil(span * r / 0.5)))
        pts = []
        for t in range(n + 1):
            a = a1 + sign * span * t / n
            x = int(round(cx + r * math.cos(a)))
            y = int(round(cy + r * math.sin(a)))
            pts.append((x, y))
        return pts
    for sign in ((-1 if nsign > 0 else 1), (1 if nsign > 0 else -1)):
        pts = arc_pts(sign)
        dedup = [pts[0]]
        for q in pts[1:]:
            if q != dedup[-1]:
                dedup.append(q)
        if all(0 <= y < free.shape[0] and 0 <= x < free.shape[1]
               and free[y, x] for x, y in dedup):
            return dedup
    return None



def _semicircle_arc(path: list[GridPoint], next_start: GridPoint,
                    free: np.ndarray) -> list[GridPoint] | None:
    """Semicircular U-turn arc between the trimmed lane ends.

    The sweep trims each lane by turn_r at the turning end, so the two
    ends are 2*turn_r apart vertically: the connecting arc has centre at
    the midpoint, radius = half the row gap, bulging toward the previous
    travel direction.  Returns the arc points (starting at path[-1]) or
    None when the arc leaves the free mask (A* fallback).
    """
    if len(path) < 2:
        return None
    p1 = path[-1]
    p2 = next_start
    dy = p2[1] - p1[1]
    if abs(dy) < 2 or abs(p2[0] - p1[0]) > 2 * abs(dy):
        return None
    cx = (p1[0] + p2[0]) / 2.0
    cy = (p1[1] + p2[1]) / 2.0
    r = abs(dy) / 2.0
    a1 = math.atan2(p1[1] - cy, p1[0] - cx)
    a2 = math.atan2(p2[1] - cy, p2[0] - cx)
    prev = path[-2]
    tsign = 1 if p1[0] >= prev[0] else -1
    if tsign > 0:
        span = (a2 - a1) % (2 * math.pi)
        sgn = 1
    else:
        span = (a1 - a2) % (2 * math.pi)
        sgn = -1
    n = max(12, int(math.ceil(span * r / 0.5)))
    pts = []
    for t in range(n + 1):
        a = a1 + sgn * span * t / n
        pts.append((int(round(cx + r * math.cos(a))),
                    int(round(cy + r * math.sin(a)))))
    dedup = [pts[0]]
    for q in pts[1:]:
        if q != dedup[-1]:
            dedup.append(q)
    if all(0 <= y < free.shape[0] and 0 <= x < free.shape[1]
           and free[y, x] for x, y in dedup):
        return dedup
    return None


def connected_boustrophedon(
    free: np.ndarray,
    spacing_cells: int,
    *,
    collision_check: bool = True,
) -> list[GridPoint]:
    """跨 cell 蛇形连接:每个 cell 内做蛇形,cell 之间用 A* 连接.

    Args:
        free: 2-D 布尔掩膜
        spacing_cells: 行间距
        collision_check: True 时,若 A* 连接线穿障则抛 ValueError(规划器契约)

    Raises:
        ValueError: 当 collision_check=True 且 A* 路径上有点不在 free 中。
    """
    free = np.asarray(free, dtype=bool)
    if free.ndim != 2:
        raise ValueError("free must be a 2-D mask")
    if spacing_cells < 1:
        raise ValueError("spacing_cells must be positive")

    if not free.any():
        return []

    cells = decompose(free)
    if not cells:
        return []

    # 收集所有段的列表(每段都附属于自己的 cell)
    cell_segments: list[list[list[GridPoint]]] = []
    for c in cells:
        segs = sweep_segments(c, spacing_cells)
        if segs:
            cell_segments.append(segs)

    if not cell_segments:
        return []

    # 拼接段:同一 cell 内段与段通过 A* 跨 cell 连接
    path: list[GridPoint] = []
    first = True
    for segs in cell_segments:
        if first:
            path.extend(segs[0])
            first = False
            rest = segs[1:]
        else:
            rest = segs

        for seg in rest:
            # ── U-turn cap (U9/B6 fix): adjacent serpentine lanes are
            # connected by a semicircular cap bulging in the travel
            # direction, so the reference heading turns continuously
            # instead of jumping 180° at a lateral hop (which stalls the
            # forward-only MPC at the first lane end).
            cap = _semicircle_arc(path, seg[0], free)
            if cap is not None:
                path.extend(cap[1:])  # cap[0] == path[-1]
                if path[-1] != seg[0]:
                    bridge = astar(path[-1], seg[0], free)
                    if not bridge:
                        if collision_check:
                            raise ValueError(
                                'no bridge from arc landing '
                                + str(path[-1]) + ' to ' + str(seg[0]))
                    else:
                        path.extend(bridge[1:])
                if path[-1] == seg[0]:
                    path.extend(seg[1:])
                else:
                    path.extend(seg)
                continue
            connector = astar(path[-1], seg[0], free)
            if not connector:
                if collision_check:
                    raise ValueError(
                        'no connector between lane fragments: '
                        + str(path[-1]) + ' -> ' + str(seg[0]))
                continue
            path.extend(connector[1:])
            if path[-1] == seg[0]:
                path.extend(seg[1:])
            else:
                path.extend(seg)
            continue
            # 验证连接线不穿障
            if collision_check:
                for x, y in connector:
                    if not free[y, x]:
                        raise ValueError(
                            f"connector passes through obstacle at ({x},{y})"
                        )
            # 拼接(去掉连接点本身)
            if path[-1] == connector[0]:
                path.extend(connector[1:])
            else:
                path.extend(connector)
            # 拼接段(去掉首点)
            if path[-1] == seg[0]:
                path.extend(seg[1:])
            else:
                path.extend(seg)
    return path


# ============================================================
# 坐标转换
# ============================================================
def grid_to_world(path: list[GridPoint],
                  spec: GridSpec) -> list[tuple[float, float]]:
    """栅格坐标 -> 世界坐标(取单元中心)."""
    return [
        (
            spec.origin_x + (x + 0.5) * spec.resolution,
            spec.origin_y + (y + 0.5) * spec.resolution,
        )
        for x, y in path
    ]


def path_length_m(xy: list[tuple[float, float]]) -> float:
    """世界路径总长度(m)."""
    return sum(
        hypot(bx - ax, by - ay)
        for (ax, ay), (bx, by) in zip(xy, xy[1:])
    )
