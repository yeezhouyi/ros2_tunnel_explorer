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

#ifndef TUNNEL_COVERAGE_EXECUTOR__COVERAGE_TASK_CORE_HPP_
#define TUNNEL_COVERAGE_EXECUTOR__COVERAGE_TASK_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "tunnel_coverage_executor/checkpoint_store.hpp"
#include "tunnel_coverage_planner/scanline_planner.hpp"

namespace tunnel_coverage_executor
{

/// Long-lived disposition of one plan segment (R10).
enum SegmentDisposition : std::int32_t
{
  DISP_PENDING = 0,
  DISP_COVERED = 1,
  DISP_BLOCKED_TEMP = 2,
  DISP_EXEMPT = 3,
  DISP_FAILED = 4
};

/// Terminal result values shared with ExecuteCoverage.action (R21).
enum TerminalResult : std::int32_t
{
  RESULT_SUCCEEDED_FULL = 0,
  RESULT_SUCCEEDED_WITH_EXEMPTIONS = 1,
  RESULT_PARTIAL_FAILED = 2,
  RESULT_CANCELLED = 3,
  RESULT_MAP_CHANGED = 4,
  RESULT_STOP_FAILED = 5
};

/// Per-segment execution bookkeeping and terminal aggregation.
///
/// Pure C++ (no ROS).  The ROS node owns the Nav2 child-goal lifecycle and
/// calls into this core to record outcomes; the core decides *dispositions*
/// and *terminal class*, keeping the two orthogonal (R10, R21).
class CoverageTaskCore
{
public:
  using Segment = tunnel_coverage_planner::CoverageSegment;

  struct Options
  {
    /// Attempts granted to one segment before it is marked FAILED.
    int max_attempts_per_segment = 2;
  };

  /// @throws std::invalid_argument if segments are empty.
  CoverageTaskCore(const std::vector<Segment> & segments, Options options);

  /// Number of plan segments.
  std::size_t size() const {return segments_.size();}

  /// Current disposition of segment @p index.
  std::int32_t disposition(std::size_t index) const;

  /// Attempt count already spent on segment @p index.
  int attempts(std::size_t index) const;

  /// True while a PENDING or BLOCKED_TEMP segment remains.
  bool hasWorkRemaining() const;

  /// Next index to execute (first PENDING, then BLOCKED_TEMP, else none).
  int nextPendingIndex() const;

  /// Record one outcome (attempts are consumed by the node before calling).
  void markCovered(std::size_t index);
  void markFailed(std::size_t index);
  void markExempt(std::size_t index);
  void markBlockedTemp(std::size_t index);

  std::size_t countPending() const;
  std::size_t countCovered() const;
  std::size_t countBlockedTemp() const;
  std::size_t countExempt() const;
  std::size_t countFailed() const;

  /// Terminal class given current dispositions (no cancelled/map-change path).
  std::int32_t terminalResult() const;

  /// Human-readable terminal class.
  static std::string terminalName(std::int32_t result);

  /// Copy dispositions from a validated checkpoint into a new plan.
  ///
  /// The checkpoint and the new plan must carry the *same* segment ids
  /// (same plan_id); otherwise std::invalid_argument is thrown.  Exempt and
  /// covered dispositions are inherited; PENDING stays pending (R22).
  void applyCheckpointDispositions(const CheckpointData & checkpoint);

private:
  std::vector<Segment> segments_;
  std::vector<std::int32_t> dispositions_;
  std::vector<int> attempts_;
  int max_attempts_;
};

}  // namespace tunnel_coverage_executor

#endif  // TUNNEL_COVERAGE_EXECUTOR__COVERAGE_TASK_CORE_HPP_
