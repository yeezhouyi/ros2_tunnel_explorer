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

#ifndef TUNNEL_COVERAGE_PLANNER__SEGMENT_ORDERER_HPP_
#define TUNNEL_COVERAGE_PLANNER__SEGMENT_ORDERER_HPP_

#include <cstddef>
#include <vector>

namespace tunnel_coverage_planner
{

/// Deterministic nearest-neighbour ordering of coverage cells (R6, R18).
///
/// Pure algorithm: consumes a precomputed NxN (asymmetric allowed) cost
/// matrix produced by the integration layer (e.g. Nav2 ComputePathToPose
/// costs).  Never calls ROS.  Ties are broken by smaller cell index so the
/// output is deterministic.
class SegmentOrderer
{
public:
  /// @param cost  NxN matrix; cost[i][j] = travel cost cell i -> cell j.
  /// @param start Index of the first cell (robot entry cell).
  /// @return Ordered indices of all N cells, starting with @p start.
  /// @throws std::invalid_argument on empty/non-square/invalid matrix or a
  ///         start index out of range.
  static std::vector<int> order(
    const std::vector<std::vector<double>> & cost, int start);
};

}  // namespace tunnel_coverage_planner

#endif  // TUNNEL_COVERAGE_PLANNER__SEGMENT_ORDERER_HPP_
