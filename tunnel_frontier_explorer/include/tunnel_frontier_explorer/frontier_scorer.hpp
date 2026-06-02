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

#ifndef TUNNEL_FRONTIER_EXPLORER__FRONTIER_SCORER_HPP_
#define TUNNEL_FRONTIER_EXPLORER__FRONTIER_SCORER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "tunnel_frontier_explorer/frontier_cluster.hpp"
#include "tunnel_frontier_explorer/frontier_detector.hpp"
#include "tunnel_frontier_explorer/frontier_visit_history.hpp"
#include "tunnel_frontier_explorer/tunnel_geometry_grid.hpp"

namespace tunnel_frontier_explorer
{

/// Configuration for FrontierScorer.
struct FrontierScorerConfig
{
  /// Radius (metres) around each candidate goal to count unknown cells.
  double information_gain_radius_meters = 0.75;

  /// Radius (metres) for counting previous visits near a candidate.
  double revisit_radius_meters = 0.50;

  /// Maximum revisit count used in clamping (must be > 0).
  std::size_t max_revisit_count = 3;

  /// Weight applied to normalised information gain (>= 0).
  double weight_information_gain = 1.0;

  /// Weight applied to normalised distance (>= 0).  Distance reduces score.
  double weight_distance = 1.0;

  /// Weight applied to normalised revisit penalty (>= 0).  Revisit reduces score.
  double weight_revisit = 1.5;

  // Stage 4B: tunnel geometry features
  double weight_centerline_alignment = 0.3;
  double weight_wall_risk = 0.5;
  double geometry_sampling_radius_meters = 0.25;
};

/// Scoring result for a single candidate goal.
struct ScoredGoal
{
  /// The original cluster with its representative_world.
  FrontierCluster cluster;

  /// Final composite score (higher = better).
  double score = 0.0;

  /// Raw count of unknown cells within the information gain radius.
  std::size_t raw_information_gain = 0;

  /// log(1 + raw_information_gain).
  double transformed_information_gain = 0.0;

  /// Normalised to [0, 1] across the candidate set.
  double normalized_information_gain = 0.0;

  /// Distance from robot to representative_world (metres).
  double goal_distance_meters = 0.0;

  /// Normalised to [0, 1] across the candidate set.
  double normalized_distance = 0.0;

  /// Raw number of previous visits within revisit_radius_meters.
  std::size_t raw_revisit_count = 0;

  /// raw_revisit_count clamped to max_revisit_count.
  std::size_t clamped_revisit_count = 0;

  /// Normalised to [0, 1] using max_revisit_count as denominator.
  double normalized_revisit_penalty = 0.0;

  // Stage 4B: tunnel geometry features (only set when using scoreAndRankTunnelAware)
  double normalized_centerline_alignment = 0.0;
  double normalized_wall_risk = 0.0;
  Point2D scoring_point_world = {0.0, 0.0};
};

/// Scores candidate frontier goals by information gain, distance, and revisit
/// penalty, then ranks them for selection.
///
/// Formula (per candidate set):
///   score = w_gain * norm_gain - w_dist * norm_dist - w_revisit * norm_revisit
///
/// Zero ROS dependencies.  Operates on FrontierCluster, GridMap, and
/// FrontierVisitHistory.
class FrontierScorer
{
public:
  /// @throws std::invalid_argument if radii <= 0, max_revisit_count == 0,
  ///         or any weight < 0.
  explicit FrontierScorer(FrontierScorerConfig config);

  /// Default constructor uses FrontierScorerConfig{} defaults (all valid).
  FrontierScorer() = default;

  /// Score and rank all candidate clusters (Stage 2/3 baseline).
  ///
  /// Information gain is computed using a **circular** radius (bounding box
  /// is used only for iteration speed).  Unknown cells are those with
  /// value == -1.  Scores are deterministic via a tie-break chain:
  ///   score desc → distance asc → gain desc → row asc → col asc.
  ///
  /// @param candidates  Clusters that passed min-distance + blacklist checks.
  /// @param map         Occupancy grid (must be valid()).
  /// @param robot       Robot position in world coordinates.
  /// @param history     Visit history for revisit penalty.
  /// @return Scored and rank-sorted candidates (highest score first).
  ///         Empty if @p candidates is empty.
  std::vector<ScoredGoal> scoreAndRank(
    const std::vector<FrontierCluster> & candidates,
    const GridMap & map,
    const Point2D & robot,
    const FrontierVisitHistory & history) const;

  /// Stage 4B: tunnel-aware scoring with centerline alignment and wall risk.
  ///
  /// Extends the baseline formula:
  ///   score = w_gain * gain - w_dist * dist - w_revisit * revisit
  ///         + w_centerline * centerline - w_wall_risk * risk
  ///
  /// The scoring point is projected 0.4 m toward the robot from the
  /// frontier representative, then sampled against the tunnel geometry
  /// grids.  This avoids penalising frontiers whose representative sits
  /// on the free/unknown boundary (which naturally has high wall risk).
  ///
  /// If @p tunnel_geometry is not valid(), falls back to the baseline
  /// scoreAndRank().
  std::vector<ScoredGoal> scoreAndRankTunnelAware(
    const std::vector<FrontierCluster> & candidates,
    const GridMap & map,
    const Point2D & robot,
    const FrontierVisitHistory & history,
    const TunnelGeometryGrid & tunnel_geometry) const;

private:
  FrontierScorerConfig config_;

  static constexpr double kScoreEpsilon = 1e-9;

  /// Count unique unknown cells (value == -1) within a circular radius around
  /// a world point.  Bounding-box fast-path with circular inclusion check.
  static std::size_t countUnknownCellsInRadius(
    const GridMap & map,
    const Point2D & center_world,
    double radius_meters);

  /// Validate config and throw on illegal values.
  static void validateConfig(const FrontierScorerConfig & cfg);
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__FRONTIER_SCORER_HPP_
