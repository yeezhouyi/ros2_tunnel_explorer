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

#include "tunnel_coverage_planner/scanline_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "tunnel_map_core/map_digest.hpp"

namespace tunnel_coverage_planner
{

namespace
{

double dist(double x0, double y0, double x1, double y1)
{
  const double dx = x0 - x1;
  const double dy = y0 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

double wrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a <= -M_PI) {a += 2.0 * M_PI;}
  return a;
}

}  // namespace

double CoverageSegment::lengthM() const
{
  return dist(start_x, start_y, end_x, end_y);
}

ScanlinePlanner::ScanlinePlanner(
  const tunnel_map_core::GridGeometry & geometry,
  const CoverageMasks & masks,
  const RobotCleaningGeometry & robot,
  ScanlinePlannerConfig config)
: geometry_(geometry),
  masks_(masks),
  robot_(robot),
  config_(config)
{
  if (!robot_.valid()) {
    throw std::invalid_argument("ScanlinePlanner — " + robot_.validationError());
  }
  if (!masks_.consistent()) {
    throw std::invalid_argument("ScanlinePlanner — inconsistent masks");
  }
  if (!(config_.overlap_eta >= 0.0 && config_.overlap_eta < 1.0)) {
    throw std::invalid_argument(
      "ScanlinePlanner — overlap_eta must be in [0, 1)");
  }
  if (!(config_.loc_err_p95_m >= 0.0 && config_.track_err_p95_m >= 0.0)) {
    throw std::invalid_argument(
      "ScanlinePlanner — error budgets must be >= 0");
  }
  if (!(config_.min_segment_length_m > 0.0)) {
    throw std::invalid_argument("ScanlinePlanner — min_segment_length_m > 0");
  }
  if (!(config_.endpoint_inset_m >= 0.0)) {
    throw std::invalid_argument("ScanlinePlanner — endpoint_inset_m >= 0");
  }
}

double ScanlinePlanner::spacingM() const
{
  const double w = robot_.cleaning_width_m;
  const double d_strip = w * (1.0 - config_.overlap_eta);
  const double d_safe =
    w - 2.0 * (config_.loc_err_p95_m + config_.track_err_p95_m);
  return std::min(d_strip, std::max(d_safe, 0.02));
}

std::vector<double> ScanlinePlanner::rowOffsets(
  double lo, double hi, double r_tool, double spacing)
{
  std::vector<double> out;
  if (hi - lo <= 0.0) {
    return out;
  }
  double y = lo + r_tool;
  while (y <= hi - r_tool + 1e-9) {
    out.push_back(y);
    y += spacing;
  }
  if (!out.empty() && out.back() < hi - r_tool - 1e-9) {
    // Anchor the final strip w/2 from the far edge so the tool reaches it.
    out.push_back(hi - r_tool);
  }
  return out;
}

CoveragePlan ScanlinePlanner::buildSweep(double axis, bool & ok) const
{
  ok = false;
  CoveragePlan plan;
  const double cell = geometry_.cellSize();
  const double w = geometry_.width();
  const double h = geometry_.height();
  const auto & reach = masks_.reachable_cleanable;
  const auto & nav = masks_.navigable_center;

  // Bounds over the reachable-cleanable mask.
  int min_row = -1, max_row = -1, min_col = -1, max_col = -1;
  for (std::size_t row = 0; row < h; ++row) {
    for (std::size_t col = 0; col < w; ++col) {
      if (reach[row * w + col] != 0) {
        min_row = min_row < 0 ? static_cast<int>(row) : std::min(min_row, static_cast<int>(row));
        max_row = std::max(max_row, static_cast<int>(row));
        min_col = min_col < 0 ? static_cast<int>(col) : std::min(min_col, static_cast<int>(col));
        max_col = std::max(max_col, static_cast<int>(col));
      }
    }
  }
  if (min_row < 0) {
    return plan;  // nothing cleanable
  }

  const double r_tool = robot_.toolSweepRadiusM();
  const double spacing = spacingM();
  const double inset = config_.endpoint_inset_m;
  const double min_len = config_.min_segment_length_m;

  // World band edges (outer edges of the first/last free cells).
  const double lo_world_y = geometry_.gridToWorld(min_row, 0).y - 0.5 * cell;
  const double hi_world_y = geometry_.gridToWorld(max_row, 0).y + 0.5 * cell;
  const double lo_world_x = geometry_.gridToWorld(0, min_col).x - 0.5 * cell;
  const double hi_world_x = geometry_.gridToWorld(0, max_col).x + 0.5 * cell;

  const bool horizontal = std::abs(axis) < 1e-9;
  const std::vector<double> offsets = horizontal ?
    rowOffsets(lo_world_y, hi_world_y, r_tool, spacing) :
    rowOffsets(lo_world_x, hi_world_x, r_tool, spacing);

  // Record the sweep identity on the returned plan (used by tests and by the
  // executor when dispatching segment types).
  plan.direction_rad = axis;
  plan.spacing_m = spacing;

  std::vector<CoverageSegment> segments;
  std::size_t work_count = 0;
  std::size_t t_index = 0;
  bool have_last = false;
  double last_x = 0.0, last_y = 0.0;

  auto emit = [&](CoverageSegment s) {
      if (have_last) {
        CoverageSegment t;
        t.id = "room0-t" + std::to_string(t_index++);
        t.cell_id = "room0";
        t.type = SegmentType::TRANSITION;
        t.start_x = last_x;
        t.start_y = last_y;
        t.end_x = s.start_x;
        t.end_y = s.start_y;
        segments.push_back(t);
      }
      segments.push_back(s);
      last_x = s.end_x;
      last_y = s.end_y;
      have_last = true;
      if (s.type == SegmentType::WORK) {
        ++work_count;
      }
    };

  for (std::size_t k = 0; k < offsets.size(); ++k) {
    const double line = offsets[k];
    const bool forward = (k % 2 == 0);
    const double dir_sign = forward ? 1.0 : -1.0;
    const double yaw = wrapAngle(dir_sign > 0 ? axis : axis + M_PI);

    if (horizontal) {
      // Line at fixed world y: runs of navigable cells along +x.
      const double y = line;
      tunnel_map_core::GridPoint gp = geometry_.worldToGrid({0.0, y});
      const int r = static_cast<int>(std::lround(gp.row - 0.5));
      if (r < 0 || static_cast<std::size_t>(r) >= h) {
        continue;
      }
      // Collect runs of navigable cells on this row.
      std::vector<std::pair<int, int>> runs;
      int c0 = -1;
      for (std::size_t col = 0; col < w; ++col) {
        const bool in_nav = nav[static_cast<std::size_t>(r) * w + col] != 0;
        if (in_nav && c0 < 0) {
          c0 = static_cast<int>(col);
        }
        if (!in_nav && c0 >= 0) {
          runs.emplace_back(c0, static_cast<int>(col) - 1);
          c0 = -1;
        }
      }
      if (c0 >= 0) {
        runs.emplace_back(c0, static_cast<int>(w) - 1);
      }
      if (forward) {
        std::sort(runs.begin(), runs.end());
      } else {
        std::sort(runs.begin(), runs.end(), std::greater<std::pair<int, int>>());
      }
      for (std::size_t j = 0; j < runs.size(); ++j) {
        const int cl = runs[j].first;
        const int cr = runs[j].second;
        const double xl = geometry_.gridToWorld(r, cl).x;
        const double xr = geometry_.gridToWorld(r, cr).x;
        const double sx = forward ? xl + inset : xr - inset;
        const double ex = forward ? xr - inset : xl + inset;
        const double len = std::abs(ex - sx);
        if (len < min_len) {
          continue;
        }
        CoverageSegment s;
        s.id = "room0-w" + std::to_string(k) + "-" + std::to_string(j);
        s.cell_id = "room0";
        s.type = SegmentType::WORK;
        s.start_x = sx;
        s.start_y = y;
        s.start_yaw = yaw;
        s.end_x = ex;
        s.end_y = y;
        s.end_yaw = yaw;
        emit(s);
      }
    } else {
      // Vertical sweep: lines at fixed world x, runs of navigable cells
      // along +y.
      const double x = line;
      tunnel_map_core::GridPoint gp = geometry_.worldToGrid({x, 0.0});
      const int c = static_cast<int>(std::lround(gp.col - 0.5));
      if (c < 0 || static_cast<std::size_t>(c) >= w) {
        continue;
      }
      std::vector<std::pair<int, int>> runs;
      int r0 = -1;
      for (std::size_t row = 0; row < h; ++row) {
        const bool in_nav = nav[row * w + static_cast<std::size_t>(c)] != 0;
        if (in_nav && r0 < 0) {
          r0 = static_cast<int>(row);
        }
        if (!in_nav && r0 >= 0) {
          runs.emplace_back(r0, static_cast<int>(row) - 1);
          r0 = -1;
        }
      }
      if (r0 >= 0) {
        runs.emplace_back(r0, static_cast<int>(h) - 1);
      }
      if (forward) {
        std::sort(runs.begin(), runs.end());
      } else {
        std::sort(runs.begin(), runs.end(), std::greater<std::pair<int, int>>());
      }
      for (std::size_t j = 0; j < runs.size(); ++j) {
        const int rl = runs[j].first;
        const int rr = runs[j].second;
        const double yl = geometry_.gridToWorld(rl, c).y;
        const double yr = geometry_.gridToWorld(rr, c).y;
        const double sy = forward ? yl + inset : yr - inset;
        const double ey = forward ? yr - inset : yl + inset;
        const double len = std::abs(ey - sy);
        if (len < min_len) {
          continue;
        }
        CoverageSegment s;
        s.id = "room0-w" + std::to_string(k) + "-" + std::to_string(j);
        s.cell_id = "room0";
        s.type = SegmentType::WORK;
        s.start_x = x;
        s.start_y = sy;
        s.start_yaw = yaw;
        s.end_x = x;
        s.end_y = ey;
        s.end_yaw = yaw;
        emit(s);
      }
    }
  }

  if (work_count == 0) {
    return plan;
  }
  plan.segments = std::move(segments);
  plan.work_count = work_count;
  ok = true;
  return plan;
}

std::string ScanlinePlanner::serialize(const CoveragePlan & plan) const
{
  std::ostringstream os;
  os << "v1|" << std::setprecision(17)
     << plan.direction_rad << '|' << plan.spacing_m << '\n';
  for (const auto & s : plan.segments) {
    os << s.id << '|'
       << static_cast<int>(s.type) << '|'
       << s.start_x << '|' << s.start_y << '|' << s.start_yaw << '|'
       << s.end_x << '|' << s.end_y << '|' << s.end_yaw << '\n';
  }
  return os.str();
}

CoveragePlan ScanlinePlanner::plan()
{
  bool ok_h = false;
  bool ok_v = false;
  CoveragePlan h = buildSweep(0.0, ok_h);
  CoveragePlan v = buildSweep(M_PI_2, ok_v);

  auto cost = [](const CoveragePlan & p) -> double {
    // MVP selection criterion: total path length (work + transition).
    // Direction-choice ablation (turn/segment weights) is tracked in the
    // config for the U6+ extension of J(theta).
      double c = 0.0;
      for (const auto & s : p.segments) {
        c += s.lengthM();
      }
      return c;
    };

  CoveragePlan chosen;
  bool have_chosen = false;
  if (ok_h && ok_v) {
    // Deterministic selection: prefer the shorter total path; tie-break by
    // fewer work segments, then by smaller angle (horizontal first).
    const double ch = cost(h);
    const double cv = cost(v);
    if (ch < cv - 1e-9) {
      chosen = h;
    } else if (cv < ch - 1e-9) {
      chosen = v;
    } else if (h.work_count <= v.work_count) {
      chosen = h;
    } else {
      chosen = v;
    }
    have_chosen = true;
  } else if (ok_h) {
    chosen = h;
    have_chosen = true;
  } else if (ok_v) {
    chosen = v;
    have_chosen = true;
  }

  if (have_chosen) {
    chosen.plan_id = tunnel_map_core::fnv1a64Hex(serialize(chosen));
  }
  return chosen;
}

}  // namespace tunnel_coverage_planner
