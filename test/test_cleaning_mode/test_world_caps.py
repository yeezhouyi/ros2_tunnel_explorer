"""Deterministic world-cap smoothing tests (E-line).

``coverage_planner._apply_world_caps`` replaces grid-staircase U-caps with
continuous world semicircular arcs (the planner records every inserted cap
via ``connected_boustrophedon(arcs=)``).  These tests pin that the spliced
world path is drivable: no |kappa| in the staircase regime (~31.4, R 0.03 m)
-- curvature stays bounded by the cap radius (R ~ lane/2 = 0.15 m,
|kappa| ~ 6.7) -- while endpoints are preserved.
"""
import math

import numpy as np
import pytest

from cleaning_mode.boustrophedon import GridSpec
from cleaning_mode.coverage_planner import _apply_world_caps


def _max_kappa(xy):
    xs = np.array([p[0] for p in xy])
    ys = np.array([p[1] for p in xy])
    n = len(xs)
    yaw = np.zeros(n)
    for i in range(n):
        j0, j1 = max(0, i - 1), min(n - 1, i + 1)
        yaw[i] = math.atan2(ys[j1] - ys[j0], xs[j1] - xs[j0])
    k = 0.0
    for i in range(1, n - 1):
        d_prev = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        d_next = math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i])
        arc = 0.5 * (d_prev + d_next)
        if arc > 1e-9:
            k = max(k, abs((yaw[i + 1] - yaw[i - 1] + math.pi)
                           % (2 * math.pi) - math.pi) / (2.0 * arc))
    return float(k)


def test_apply_world_caps_replaces_staircase():
    spec = GridSpec(resolution=0.05, origin_x=0.0, origin_y=0.0)
    p1 = (30, 5)
    p2 = (30, 11)
    # explicit Manhattan "cap" with 90-degree corners (grid staircase)
    cap = ([(30, y) for y in range(5, 9)] +
           [(x, 8) for x in (31, 32)] +
           [(32, y) for y in range(9, 12)] +
           [(31, 11), p2])
    # full grid path: straight row end into p1, the staircase cap up to p2,
    # then the next row continues LEFT (cap top tangent is -x)
    prefix = [(29, 5), p1]
    row2 = [(29, 11), (28, 11), (27, 11)]
    grid_path = prefix + cap[1:] + row2
    start = len(prefix) - 1            # index of p1 in grid_path
    xy = [(spec.origin_x + (c[0] + 0.5) * spec.resolution,
           spec.origin_y + (c[1] + 0.5) * spec.resolution)
          for c in grid_path]
    raw_k = _max_kappa(xy)
    assert raw_k > 12.0, "fixture must actually be a staircase"

    arcs = [dict(start=start, n=len(cap), p1=p1, p2=p2)]
    out = _apply_world_caps(xy, grid_path, arcs, spec)

    # endpoints preserved, interior now smooth
    assert out[start] == xy[start]
    assert out[-1] == xy[-1]
    smoothed = _max_kappa(out)
    assert smoothed < 9.0, f"staircase not removed (max kappa {smoothed})"
    # and there IS still turning geometry (the semicircle), not a straight cut
    assert smoothed > 3.0
