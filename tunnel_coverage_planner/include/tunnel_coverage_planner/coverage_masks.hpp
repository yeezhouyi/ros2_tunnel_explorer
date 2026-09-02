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

#ifndef TUNNEL_COVERAGE_PLANNER__COVERAGE_MASKS_HPP_
#define TUNNEL_COVERAGE_PLANNER__COVERAGE_MASKS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "tunnel_map_core/grid_geometry.hpp"
#include "tunnel_map_core/grid_map.hpp"

namespace tunnel_coverage_planner
{

/// Reasons a target cell is exempt from cleaning (auditable, R23).
enum class ExemptCause : std::uint8_t
{
  NONE = 0,       ///< cell is a cleaning target
  UNREACHABLE = 1,  ///< not reachable from the seed via navigable space
  ISLAND = 2,     ///< physically cleanable but below the min-area gate
  UNKNOWN = 3     ///< non-free/uncertain cell (never a target)
};

/// The four semantic masks of the cleaning track (R1, R19).
///
/// All masks share one GridGeometry and are row-major vectors of 0/1 with
/// `size() == width * height`.
struct CoverageMasks
{
  /// Cells the task intends to clean (the gross-coverage denominator T).
  std::vector<std::uint8_t> intended_target;

  /// Cells whose centre the chassis can occupy (free space eroded by the
  /// chassis clearance radius).
  std::vector<std::uint8_t> navigable_center;

  /// Target cells physically reachable by the centred cleaning footprint:
  /// navigable component(s) connected to the seed, dilated by the tool sweep
  /// radius, intersected with the intended target (effective denominator
  /// T \ E).
  std::vector<std::uint8_t> reachable_cleanable;

  /// Target cells that will not be cleaned and why.  Stored parallel to the
  /// other masks; exempt_cause[row * width + col] is meaningful only when
  /// exempt[..] == 1.
  std::vector<std::uint8_t> exempt;
  std::vector<std::uint8_t> exempt_cause;  // ExemptCause values

  /// Underlying map geometry (same for every mask).
  tunnel_map_core::GridGeometry geometry;

  /// Validate that every mask has the expected dimensions.
  bool consistent() const
  {
    const auto n = geometry.width() * geometry.height();
    return intended_target.size() == n && navigable_center.size() == n &&
           reachable_cleanable.size() == n && exempt.size() == n &&
           exempt_cause.size() == n;
  }

  /// Number of true cells in a mask.
  static std::size_t count(const std::vector<std::uint8_t> & mask)
  {
    std::size_t c = 0;
    for (const auto v : mask) {
      c += (v != 0) ? 1u : 0u;
    }
    return c;
  }

  /// Area in m^2 of a mask.
  double areaM2(const std::vector<std::uint8_t> & mask) const
  {
    return static_cast<double>(count(mask)) *
           geometry.cellSize() * geometry.cellSize();
  }
};

}  // namespace tunnel_coverage_planner

#endif  // TUNNEL_COVERAGE_PLANNER__COVERAGE_MASKS_HPP_
