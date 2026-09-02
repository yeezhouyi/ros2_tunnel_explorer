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
"""Coverage metrics — pure-Python mirror of tunnel_coverage_planner.

The C++ CoverageTracker::metrics() and this module must agree on the
definitions of gross / effective / exempt / repeat coverage (R4, R23).
Kept dependency-free so it can be unit-tested with plain pytest and reused by
rosbag analysis and the benchmark aggregator.

Definitions (denominators explicitly tracked, R19):
  T  = intended_target cells
  E  = exempt cells (subset of T)
  V  = cells visited at least once by the real trajectory sweep
  gross     = |T ∩ V| / |T|
  effective = |(T \\ E) ∩ V| / |T \\ E|
  exempt    = |E| / |T|
  repeat    = |{c in T \\ E : visits(c) > 1}| / |T \\ E|
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence


@dataclass
class CoverageMetrics:
    """Plain-number mirror of tunnel_coverage_planner::CoverageMetrics."""

    gross_coverage: float = 0.0
    effective_coverage: float = 0.0
    exempt_ratio: float = 0.0
    repeat_ratio: float = 0.0
    unique_swept_area_m2: float = 0.0
    total_swept_area_m2: float = 0.0
    path_length_m: float = 0.0
    intended_area_m2: float = 0.0
    effective_area_m2: float = 0.0
    exempt_area_m2: float = 0.0


def compute_metrics(
    intended_target: Sequence[int],
    exempt: Sequence[int],
    visit_counts: Sequence[int],
    cell_area_m2: float,
    path_length_m: float = 0.0,
    total_swept_area_m2: float = 0.0,
) -> CoverageMetrics:
    """Compute coverage metrics from parallel per-cell vectors."""
    assert len(intended_target) == len(exempt) == len(visit_counts)
    t_total = t_effective = e_total = 0
    covered_gross = covered_effective = 0
    repeat_effective = 0
    swept_unique_effective = 0

    for target, ex, visits in zip(intended_target, exempt, visit_counts):
        if not target:
            continue
        t_total += 1
        if visits > 0:
            covered_gross += 1
        if not ex:
            t_effective += 1
            if visits > 0:
                covered_effective += 1
                swept_unique_effective += 1
                if visits > 1:
                    repeat_effective += 1
        else:
            e_total += 1

    m = CoverageMetrics()
    m.intended_area_m2 = t_total * cell_area_m2
    m.effective_area_m2 = t_effective * cell_area_m2
    m.exempt_area_m2 = e_total * cell_area_m2
    m.gross_coverage = covered_gross / t_total if t_total else 1.0
    m.effective_coverage = (
        covered_effective / t_effective if t_effective else 1.0
    )
    m.exempt_ratio = e_total / t_total if t_total else 0.0
    m.repeat_ratio = (
        repeat_effective / t_effective if t_effective else 0.0
    )
    m.unique_swept_area_m2 = swept_unique_effective * cell_area_m2
    m.total_swept_area_m2 = total_swept_area_m2
    m.path_length_m = path_length_m
    return m


@dataclass
class MaskSet:
    """Row-major masks sized width*height (mirror CoverageMasks)."""

    width: int
    height: int
    intended_target: List[int] = field(default_factory=list)
    navigable_center: List[int] = field(default_factory=list)
    reachable_cleanable: List[int] = field(default_factory=list)
    exempt: List[int] = field(default_factory=list)
    exempt_cause: List[int] = field(default_factory=list)


def masks_all_free(width: int, height: int) -> MaskSet:
    n = width * height
    return MaskSet(
        width=width,
        height=height,
        intended_target=[1] * n,
        navigable_center=[1] * n,
        reachable_cleanable=[1] * n,
        exempt=[0] * n,
        exempt_cause=[0] * n,
    )
