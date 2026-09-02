// Copyright 2026 zhouyi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tunnel_coverage_planner/coverage_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace tunnel_coverage_planner
{

namespace
{

/// Squared distance between two world points.
double dist2(
  const tunnel_map_core::Point2D & a, const tunnel_map_core::Point2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

}  // namespace

CoverageTracker::CoverageTracker(
  const tunnel_map_core::GridGeometry & geometry,
  const CoverageMasks & masks,
  double sweep_radius_m)
: geometry_(geometry),
  masks_(masks),
  sweep_radius_m_(sweep_radius_m),
  max_step_m_(0.5 * geometry.cellSize())
{
  if (!(sweep_radius_m_ > 0.0)) {
    throw std::invalid_argument("CoverageTracker — sweep_radius_m must be > 0");
  }
  if (!masks_.consistent()) {
    throw std::invalid_argument("CoverageTracker — inconsistent masks");
  }
  counts_.assign(masks_.intended_target.size(), 0);
  mark_buf_.assign(masks_.intended_target.size(), 0);
}

std::size_t CoverageTracker::stampDisc(const tunnel_map_core::Point2D & centre)
{
  const double cell = geometry_.cellSize();
  const int k = static_cast<int>(std::ceil(sweep_radius_m_ / cell));
  const double r2 = sweep_radius_m_ * sweep_radius_m_ + 1e-12;

  tunnel_map_core::GridCell centre_cell;
  if (!geometry_.worldToGridCell(centre, centre_cell)) {
    return 0;  // off-map sample: nothing to mark
  }
  const std::size_t w = geometry_.width();
  const std::size_t h = geometry_.height();
  std::size_t newly_marked = 0;

  for (int dr = -k; dr <= k; ++dr) {
    for (int dc = -k; dc <= k; ++dc) {
      const auto row = static_cast<std::int64_t>(centre_cell.row) + dr;
      const auto col = static_cast<std::int64_t>(centre_cell.col) + dc;
      if (row < 0 || col < 0 ||
        static_cast<std::size_t>(row) >= h ||
        static_cast<std::size_t>(col) >= w)
      {
        continue;
      }
      // Cell membership of the disc is decided by comparing cell centres.
      const auto centre_world = geometry_.gridToWorld(
        static_cast<int>(row), static_cast<int>(col));
      if (dist2(centre, centre_world) <= r2) {
        auto & m = mark_buf_[geometry_.index(
          static_cast<int>(row), static_cast<int>(col))];
        if (m == 0) {
          m = 1;
          ++newly_marked;
        }
      }
    }
  }
  return newly_marked;
}

void CoverageTracker::addSweepSegment(
  const tunnel_map_core::Point2D & a, const tunnel_map_core::Point2D & b)
{
  const double cell_area = geometry_.cellSize() * geometry_.cellSize();
  const double len = std::sqrt(dist2(a, b));

  // Clear the per-pass mark buffer.
  std::fill(mark_buf_.begin(), mark_buf_.end(), 0);

  if (len <= 0.0) {
    const std::size_t marked = stampDisc(a);
    total_swept_area_m2_ += static_cast<double>(marked) * cell_area;
    commitPass();
    return;
  }
  // Interpolate at steps no larger than half a cell so that sparse odometry
  // does not create uncovered holes between samples.
  const int n = std::max(1, static_cast<int>(std::ceil(len / max_step_m_)));
  std::size_t union_marked = 0;
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    tunnel_map_core::Point2D p;
    p.x = a.x + t * (b.x - a.x);
    p.y = a.y + t * (b.y - a.y);
    union_marked += stampDisc(p);
  }
  total_swept_area_m2_ += static_cast<double>(union_marked) * cell_area;
  commitPass();
  path_length_m_ += len;
}

void CoverageTracker::addToolPose(const ToolPose & pose)
{
  addSweepSegment(pose.position, pose.position);
}

void CoverageTracker::commitPass()
{
  const auto n = counts_.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (mark_buf_[i] != 0) {
      counts_[i] = counts_[i] == 0 ? 1 : counts_[i] + 1;
    }
  }
}

std::size_t CoverageTracker::uniqueCoveredCells() const
{
  std::size_t n = 0;
  for (const auto c : counts_) {
    if (c > 0) {
      ++n;
    }
  }
  return n;
}

bool CoverageTracker::effectivelyComplete() const
{
  return metrics().effective_coverage >= 1.0 - 1e-9;
}

CoverageMetrics CoverageTracker::metrics() const
{
  CoverageMetrics m;
  const auto n = counts_.size();
  std::size_t t_total = 0;
  std::size_t t_effective = 0;
  std::size_t e_total = 0;
  std::size_t covered_gross = 0;      // T ∩ V
  std::size_t covered_effective = 0;  // (T \ E) ∩ V
  std::size_t repeat_effective = 0;   // {c in T \ E : visits > 1}
  std::size_t swept_unique_effective = 0;

  for (std::size_t i = 0; i < n; ++i) {
    const bool target = masks_.intended_target[i] != 0;
    const bool exempt = masks_.exempt[i] != 0;
    const bool visited = counts_[i] > 0;
    if (!target) {
      continue;
    }
    ++t_total;
    if (visited) {
      ++covered_gross;
    }
    if (!exempt) {
      ++t_effective;
      if (visited) {
        ++covered_effective;
        ++swept_unique_effective;
        if (counts_[i] > 1) {
          ++repeat_effective;
        }
      }
    } else {
      ++e_total;
    }
  }

  const double cell_area = geometry_.cellSize() * geometry_.cellSize();
  m.intended_area_m2 = static_cast<double>(t_total) * cell_area;
  m.effective_area_m2 = static_cast<double>(t_effective) * cell_area;
  m.exempt_area_m2 = static_cast<double>(e_total) * cell_area;

  m.gross_coverage = t_total > 0 ?
    static_cast<double>(covered_gross) / static_cast<double>(t_total) : 1.0;
  m.effective_coverage = t_effective > 0 ?
    static_cast<double>(covered_effective) / static_cast<double>(t_effective) : 1.0;
  m.exempt_ratio = t_total > 0 ?
    static_cast<double>(e_total) / static_cast<double>(t_total) : 0.0;
  m.repeat_ratio = t_effective > 0 ?
    static_cast<double>(repeat_effective) / static_cast<double>(t_effective) : 0.0;
  m.unique_swept_area_m2 =
    static_cast<double>(swept_unique_effective) * cell_area;
  m.total_swept_area_m2 = total_swept_area_m2_;
  m.path_length_m = path_length_m_;

  return m;
}

void CoverageTracker::reset()
{
  std::fill(counts_.begin(), counts_.end(), 0);
  path_length_m_ = 0.0;
  total_swept_area_m2_ = 0.0;
}

void CoverageTracker::restore(
  const std::vector<std::int32_t> & counts, double path_length_m)
{
  if (counts.size() != counts_.size()) {
    throw std::invalid_argument(
      "CoverageTracker::restore — visit-count vector size mismatch");
  }
  counts_ = counts;
  path_length_m_ = std::max(0.0, path_length_m);
  // The exact per-stamp sum is not recoverable from counts alone; the
  // restored report uses unique swept area as a lower bound.
  total_swept_area_m2_ = static_cast<double>(uniqueCoveredCells()) *
    geometry_.cellSize() * geometry_.cellSize();
}

}  // namespace tunnel_coverage_planner
