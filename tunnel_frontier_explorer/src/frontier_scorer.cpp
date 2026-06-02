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

#include "tunnel_frontier_explorer/frontier_scorer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace tunnel_frontier_explorer
{

// ── validateConfig ──────────────────────────────────────────────────────────

void FrontierScorer::validateConfig(const FrontierScorerConfig & cfg)
{
  if (cfg.information_gain_radius_meters <= 0.0) {
    throw std::invalid_argument(
      "FrontierScorer: information_gain_radius_meters must be > 0, got " +
      std::to_string(cfg.information_gain_radius_meters));
  }
  if (cfg.revisit_radius_meters <= 0.0) {
    throw std::invalid_argument(
      "FrontierScorer: revisit_radius_meters must be > 0, got " +
      std::to_string(cfg.revisit_radius_meters));
  }
  if (cfg.max_revisit_count == 0) {
    throw std::invalid_argument(
      "FrontierScorer: max_revisit_count must be > 0, got 0");
  }
  if (cfg.weight_information_gain < 0.0) {
    throw std::invalid_argument(
      "FrontierScorer: weight_information_gain must be >= 0, got " +
      std::to_string(cfg.weight_information_gain));
  }
  if (cfg.weight_distance < 0.0) {
    throw std::invalid_argument(
      "FrontierScorer: weight_distance must be >= 0, got " +
      std::to_string(cfg.weight_distance));
  }
  if (cfg.weight_revisit < 0.0) {
    throw std::invalid_argument(
      "FrontierScorer: weight_revisit must be >= 0, got " +
      std::to_string(cfg.weight_revisit));
  }
}

// ── Constructor ─────────────────────────────────────────────────────────────

FrontierScorer::FrontierScorer(FrontierScorerConfig config)
: config_(config)
{
  validateConfig(config_);
}

// ── countUnknownCellsInRadius ───────────────────────────────────────────────

std::size_t FrontierScorer::countUnknownCellsInRadius(
  const GridMap & map,
  const Point2D & center_world,
  double radius_meters)
{
  // Convert centre to grid coordinates.
  const double cx = (center_world.x - map.origin_x) / map.resolution - 0.5;
  const double cy = (center_world.y - map.origin_y) / map.resolution - 0.5;
  const int centre_col = static_cast<int>(std::round(cx));
  const int centre_row = static_cast<int>(std::round(cy));

  const int radius_cells =
    static_cast<int>(std::ceil(radius_meters / map.resolution));
  const double radius_sq = radius_meters * radius_meters;

  std::size_t count = 0;

  // Bounding-box iteration with circular check.
  const int min_row = std::max(0, centre_row - radius_cells);
  const int max_row = std::min(
    static_cast<int>(map.height) - 1, centre_row + radius_cells);
  const int min_col = std::max(0, centre_col - radius_cells);
  const int max_col = std::min(
    static_cast<int>(map.width) - 1, centre_col + radius_cells);

  for (int r = min_row; r <= max_row; ++r) {
    for (int c = min_col; c <= max_col; ++c) {
      // Cell centre in world coordinates.
      const double wx = map.origin_x + (static_cast<double>(c) + 0.5) * map.resolution;
      const double wy = map.origin_y + (static_cast<double>(r) + 0.5) * map.resolution;
      const double dx = wx - center_world.x;
      const double dy = wy - center_world.y;

      // Circular radius check.
      if (dx * dx + dy * dy > radius_sq) {
        continue;
      }

      const std::size_t idx = static_cast<std::size_t>(r) * map.width +
                              static_cast<std::size_t>(c);
      if (map.data[idx] == -1) {
        ++count;
      }
    }
  }

  return count;
}

// ── scoreAndRank ────────────────────────────────────────────────────────────

std::vector<ScoredGoal> FrontierScorer::scoreAndRank(
  const std::vector<FrontierCluster> & candidates,
  const GridMap & map,
  const Point2D & robot,
  const FrontierVisitHistory & history) const
{
  if (candidates.empty()) {
    return {};
  }

  const std::size_t n = candidates.size();

  // ── Phase 1: Compute raw metrics ──────────────────────────────────────
  std::vector<ScoredGoal> scored(n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto & c = candidates[i];

    scored[i].cluster = c;
    scored[i].goal_distance_meters = c.goal_distance_to_robot;

    const double dx = c.representative_world.x - robot.x;
    const double dy = c.representative_world.y - robot.y;
    // Use actual computed distance (should match goal_distance_to_robot).
    scored[i].goal_distance_meters = std::sqrt(dx * dx + dy * dy);

    // Information gain.
    scored[i].raw_information_gain = countUnknownCellsInRadius(
      map, c.representative_world, config_.information_gain_radius_meters);
    scored[i].transformed_information_gain =
      std::log1p(static_cast<double>(scored[i].raw_information_gain));

    // Revisit penalty.
    scored[i].raw_revisit_count = history.countVisitsNear(
      c.representative_world, config_.revisit_radius_meters);
    scored[i].clamped_revisit_count = std::min(
      scored[i].raw_revisit_count, config_.max_revisit_count);
    scored[i].normalized_revisit_penalty =
      static_cast<double>(scored[i].clamped_revisit_count) /
      static_cast<double>(config_.max_revisit_count);
  }

  // ── Phase 2: Find min/max for normalisation ───────────────────────────
  double min_gain = std::numeric_limits<double>::max();
  double max_gain = std::numeric_limits<double>::lowest();
  double min_dist = std::numeric_limits<double>::max();
  double max_dist = std::numeric_limits<double>::lowest();

  for (const auto & sg : scored) {
    min_gain = std::min(min_gain, sg.transformed_information_gain);
    max_gain = std::max(max_gain, sg.transformed_information_gain);
    min_dist = std::min(min_dist, sg.goal_distance_meters);
    max_dist = std::max(max_dist, sg.goal_distance_meters);
  }

  const double gain_range = max_gain - min_gain;
  const double dist_range = max_dist - min_dist;

  // ── Phase 3: Normalise and compute final score ────────────────────────
  for (auto & sg : scored) {
    // Normalised gain  [0, 1].
    if (gain_range > kScoreEpsilon) {
      sg.normalized_information_gain =
        (sg.transformed_information_gain - min_gain) / gain_range;
    } else {
      sg.normalized_information_gain = 0.0;
    }

    // Normalised distance  [0, 1].
    if (dist_range > kScoreEpsilon) {
      sg.normalized_distance =
        (sg.goal_distance_meters - min_dist) / dist_range;
    } else {
      sg.normalized_distance = 0.0;
    }

    // Final score.
    sg.score =
      config_.weight_information_gain * sg.normalized_information_gain -
      config_.weight_distance * sg.normalized_distance -
      config_.weight_revisit * sg.normalized_revisit_penalty;
  }

  // ── Phase 4: Sort by score descending with deterministic tie-break ────
  std::sort(scored.begin(), scored.end(),
    [](const ScoredGoal & a, const ScoredGoal & b) {
      // 1. Score descending.
      if (std::abs(a.score - b.score) > kScoreEpsilon) {
        return a.score > b.score;
      }

      // 2. Distance ascending (closer = better).
      if (std::abs(a.goal_distance_meters - b.goal_distance_meters) >
          kScoreEpsilon) {
        return a.goal_distance_meters < b.goal_distance_meters;
      }

      // 3. Information gain descending.
      if (a.raw_information_gain != b.raw_information_gain) {
        return a.raw_information_gain > b.raw_information_gain;
      }

      // 4. Representative row ascending.
      if (a.cluster.representative_cell.row !=
          b.cluster.representative_cell.row) {
        return a.cluster.representative_cell.row <
               b.cluster.representative_cell.row;
      }

      // 5. Representative col ascending.
      return a.cluster.representative_cell.col <
             b.cluster.representative_cell.col;
    });

  return scored;
}

}  // namespace tunnel_frontier_explorer
