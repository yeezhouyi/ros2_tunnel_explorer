# Copyright 2026 zhouyi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the pure coverage metrics (R4/R23 mirror of the C++ core)."""

import math

from benchmark_tools.coverage_metrics import (
    compute_metrics,
    masks_all_free,
)


def _band_exempt(width: int, height: int, x_lo: int, x_hi: int):
    masks = masks_all_free(width, height)
    for row in range(height):
        for col in range(width):
            if x_lo <= col < x_hi:
                idx = row * width + col
                masks.exempt[idx] = 1
                masks.exempt_cause[idx] = 1  # UNREACHABLE
    return masks


def test_gross_effective_exempt_denominators():
    # 40x30 map; exempt band columns [0, 12) -> exempt ratio 0.3.
    masks = _band_exempt(40, 30, 0, 12)
    n = 40 * 30
    visit = [0] * n
    # Cover only the exempt band.
    for row in range(30):
        for col in range(12):
            visit[row * 40 + col] = 1

    m = compute_metrics(
        masks.intended_target, masks.exempt, visit, cell_area_m2=0.05 * 0.05
    )
    assert m.exempt_ratio == 12 / 40
    assert m.effective_coverage == 0.0
    assert 0.0 < m.gross_coverage < 0.5

    # Cover only the effective region (columns [12, 40)) -> effective 1,
    # gross 0.7 (the exempt band stays unvisited and keeps lowering gross).
    visit2 = [0] * n
    for row in range(30):
        for col in range(12, 40):
            visit2[row * 40 + col] = 1
    m2 = compute_metrics(
        masks.intended_target, masks.exempt, visit2, cell_area_m2=0.05 * 0.05
    )
    assert m2.effective_coverage == 1.0
    assert m2.gross_coverage == 28 / 40
    assert m2.exempt_ratio == 12 / 40


def test_repeat_ratio_counts_only_effective_cells():
    masks = masks_all_free(10, 10)
    n = 100
    visit = [1] * n
    # Revisit every other cell -> repeat 0.5 over T\E.
    for i in range(0, n, 2):
        visit[i] = 2
    m = compute_metrics(
        masks.intended_target, masks.exempt, visit, cell_area_m2=0.01
    )
    assert m.repeat_ratio == 0.5
    assert m.unique_swept_area_m2 == 1.0  # all 100 cells once


def test_empty_target_is_vacuous():
    masks = masks_all_free(4, 4)
    masks.intended_target = [0] * 16
    m = compute_metrics(
        masks.intended_target, masks.exempt, [0] * 16, cell_area_m2=1.0
    )
    assert m.gross_coverage == 1.0
    assert m.effective_coverage == 1.0
    assert m.exempt_ratio == 0.0
    assert m.intended_area_m2 == 0.0


def test_disc_sweep_matches_expected_area():
    """Sweep radius 0.25 over length 1.4 -> ~0.7 m^2 band."""
    # Reuse 40x30 free map at 0.05.
    masks = masks_all_free(40, 30)
    cell = 0.05
    # Mark cells whose centre is within 0.25 m of *any point* of the segment
    # x in [0.3, 1.7] at y = 0.75.
    visit = [0] * (40 * 30)
    for row in range(30):
        for col in range(40):
            wx = (col + 0.5) * cell
            wy = (row + 0.5) * cell
            if 0.3 - 1e-9 <= wx <= 1.7 + 1e-9:
                d = abs(wy - 0.75)
            else:
                dx = min(abs(wx - 0.3), abs(wx - 1.7))
                d = math.hypot(dx, wy - 0.75)
            if d <= 0.25 + 1e-9:
                visit[row * 40 + col] = 1
    m = compute_metrics(
        masks.intended_target, masks.exempt, visit, cell_area_m2=cell * cell
    )
    # Swept set = 0.5 x 1.4 rectangle plus two end half-discs (radius 0.25).
    expected = 0.5 * 1.4 + math.pi * 0.25 * 0.25
    assert abs(m.unique_swept_area_m2 - expected) <= 0.03 * expected
