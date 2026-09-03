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

#ifndef TUNNEL_COVERAGE_PLANNER__BOUSTROPHEDON_DECOMPOSER_HPP_
#define TUNNEL_COVERAGE_PLANNER__BOUSTROPHEDON_DECOMPOSER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "tunnel_coverage_planner/coverage_masks.hpp"
#include "tunnel_map_core/grid_geometry.hpp"

namespace tunnel_coverage_planner
{

/// One Boustrophedon decomposition cell (R6).
///
/// A cell is a maximal y-band (rows r0..r1) in which every sweep line
/// (map row) intersects the region in a single connected run, so a plain
/// boustrophedon row sweep covers it without further splits.
struct CoverageCell
{
  /// Stable id ("c0", "c1", ... in creation order).
  std::string id;
  /// Row-major mask of the cells of this region (size = w*h).
  std::vector<std::uint8_t> mask;
  /// Indices of neighbouring cells (touch at split/merge event rows).
  std::vector<int> neighbors;
  /// Bounding box over true mask cells.
  int row0 = -1;
  int row1 = -1;
  int col0 = -1;
  int col1 = -1;
  std::size_t cellCount() const;

  /// Geometric centroid (map frame) of the cell mask.
  tunnel_map_core::Point2D centroid(const tunnel_map_core::GridGeometry & g) const;
};

/// Discrete Boustrophedon cell decomposition (pure algorithm, no ROS).
///
/// Sweeps the map rows top-to-bottom and tracks the connected runs of the
/// reachable-cleanable mask on each row.  Events:
///   - one previous run splits into several new runs  -> previous cell ends,
///     a new cell starts per new run (e.g. sweeping past a pillar);
///   - several previous runs merge into one new run   -> those cells end and
///     one merged cell starts (e.g. past the pillar end);
///   - one run continues (1:1 overlap)                -> same cell.
/// Neighbour edges are added between cells that touch at an event row, which
/// is what the cell-orderer uses to route between regions (R6).
class BoustrophedonDecomposer
{
public:
  /// @param geometry Frozen grid geometry.
  /// @param masks    Coverage masks (only reachable_cleanable is consumed).
  /// @throws std::invalid_argument if masks are inconsistent.
  BoustrophedonDecomposer(
    const tunnel_map_core::GridGeometry & geometry,
    const CoverageMasks & masks);

  /// Decompose the reachable-cleanable region.
  /// @return Cells in deterministic creation order (row-major scan).
  std::vector<CoverageCell> decompose() const;

private:
  tunnel_map_core::GridGeometry geometry_;
  CoverageMasks masks_;

  struct Run
  {
    int c0 = 0;
    int c1 = 0;
    int label = -1;
  };

  /// True-cell intervals of one row.
  std::vector<Run> rowRuns(int row) const;
};

}  // namespace tunnel_coverage_planner

#endif  // TUNNEL_COVERAGE_PLANNER__BOUSTROPHEDON_DECOMPOSER_HPP_
