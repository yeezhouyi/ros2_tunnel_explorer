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

#ifndef TUNNEL_COVERAGE_EXECUTOR__CHECKPOINT_STORE_HPP_
#define TUNNEL_COVERAGE_EXECUTOR__CHECKPOINT_STORE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace tunnel_coverage_executor
{

/// Versioned checkpoint payload (R22).
///
/// A checkpoint bundles the CoverageGrid (visit counts), the task-input and
/// plan identities, the segment dispositions and the resume position.  Only
/// the *discrete* state travels across restarts; segment ids are semantic
/// strings so residual replanning never migrates state by array index.
struct CheckpointData
{
  std::uint32_t version = 1;
  std::string task_input_id;
  std::string plan_id;
  std::string map_digest;
  int32_t resume_segment_index = 0;
  /// Segment ids of the plan the checkpoint belongs to.
  std::vector<std::string> segment_ids;
  /// Per-segment disposition (0 PENDING, 1 COVERED, 2 BLOCKED_TEMP,
  /// 3 EXEMPT, 4 FAILED).
  std::vector<std::int32_t> dispositions;
  /// Per-cell visit counts of the CoverageGrid (row-major, one per cell).
  std::vector<std::int32_t> visit_counts;
  double path_length_m = 0.0;
  double task_duration_s = 0.0;
};

/// Result of a load/validate attempt.
enum class CheckpointLoadStatus
{
  OK,           ///< loaded and identity-validated
  NOT_FOUND,    ///< file missing
  CORRUPT,      ///< unreadable / truncated / version mismatch
  MISMATCH      ///< task_input_id or plan_id differ from the current task
};

/// Plain-file, atomic checkpoint store (no ROS).
///
/// File format v1: newline-delimited header + length-prefixed rows, written
/// to `<path>.tmp` and atomically renamed over `<path>` so a crash never
/// leaves a half-written checkpoint (R12, R21).
class CheckpointStore
{
public:
  static constexpr const char * kMagic = "TUNNEL_COV_CP";

  /// Write atomically.  Throws std::runtime_error on I/O failure.
  static void save(const std::string & path, const CheckpointData & data);

  /// Load and identity-validate against the expected task ids.
  static CheckpointLoadStatus loadAndValidate(
    const std::string & path,
    const std::string & expected_task_input_id,
    const std::string & expected_plan_id,
    CheckpointData & data);
};

}  // namespace tunnel_coverage_executor

#endif  // TUNNEL_COVERAGE_EXECUTOR__CHECKPOINT_STORE_HPP_
