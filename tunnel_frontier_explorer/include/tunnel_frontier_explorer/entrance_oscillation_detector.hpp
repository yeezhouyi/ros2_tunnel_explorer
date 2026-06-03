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

#ifndef TUNNEL_FRONTIER_EXPLORER__ENTRANCE_OSCILLATION_DETECTOR_HPP_
#define TUNNEL_FRONTIER_EXPLORER__ENTRANCE_OSCILLATION_DETECTOR_HPP_

#include <deque>
#include <set>
#include <string>

#include "tunnel_frontier_explorer/frontier_cluster.hpp"

namespace tunnel_frontier_explorer
{

/// A single goal dispatch event recorded by the detector.
struct GoalDispatchEvent
{
  Point2D goal;
  Point2D robot;
  double stamp_seconds = 0.0;
  int selected_bin = 0;
  double revisit_ratio = 0.0;
  int unique_bins = 0;
};

/// Result of oscillation evaluation.
struct OscillationStatus
{
  bool oscillating = false;
  int event_count = 0;
  double spatial_radius_m = 0.0;
  int repeated_goal_count = 0;
  int stagnant_bin_count = 0;
  std::string reason;
};

/// Configuration for the entrance oscillillation detector.
struct EntranceOscillationConfig
{
  int window_goals = 6;
  double radius_m = 2.0;
  int min_repeated_goals = 4;
  int max_unique_bins = 2;
  double min_revisit_ratio = 0.35;
  int min_goals_to_check = 4;
  // Type B: alternating-pair detection
  bool detect_alternating_pair = true;
  double pair_cluster_radius_m = 0.75;
  double pair_max_spatial_radius_m = 1.5;
  int pair_min_cluster_count = 2;
  double pair_min_alternation_score = 0.5;
};

/// Result of alternating-pair oscillillation detection (Type B).
struct AlternatingPairStatus
{
  bool detected = false;
  int cluster_count = 0;
  double alternation_score = 0.0;
  Point2D cluster_a_center;
  Point2D cluster_b_center;
  std::string reason;
};

/// Detects entrance-area oscillation in goal dispatch patterns.
///
/// Tracks a sliding window of recent goal dispatch events and checks
/// whether goals are clustered in a small area with low exploration progress.
/// Pure C++ class with no ROS dependencies.
class EntranceOscillationDetector
{
public:
  EntranceOscillationDetector() = default;
  explicit EntranceOscillationDetector(EntranceOscillationConfig config);

  /// Record a goal dispatch event. Automatically trims the window.
  void recordGoal(const GoalDispatchEvent & event);

  /// Evaluate the current window for oscillation.
  /// Increments event_count if oscillation is detected.
  OscillationStatus evaluate();

  /// Clear all recorded events and reset counters.
  void reset();

  /// Number of events currently in the sliding window.
  std::size_t windowSize() const;

  /// Total number of oscillation events detected since construction/reset.
  std::size_t eventCount() const;

  /// Count unique selected_bin values in the current window.
  /// Useful for populating GoalDispatchEvent.unique_bins before recording.
  int getCurrentUniqueBins() const;

  /// Centroid of goal positions in the current window.
  /// Returns {0,0} if window is empty.
  Point2D getOscillationCenter() const;

  /// Detect Type B oscillillation: alternating pair of nearby goals.
  /// Returns AlternatingPairStatus with detection result.
  AlternatingPairStatus detectAlternatingPair() const;

private:
  EntranceOscillationConfig config_;
  std::deque<GoalDispatchEvent> events_;
  int total_oscillation_events_ = 0;
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__ENTRANCE_OSCILLATION_DETECTOR_HPP_
