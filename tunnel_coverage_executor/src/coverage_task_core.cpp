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

#include "tunnel_coverage_executor/coverage_task_core.hpp"

#include <algorithm>
#include <stdexcept>

namespace tunnel_coverage_executor
{

CoverageTaskCore::CoverageTaskCore(
  const std::vector<Segment> & segments, Options options)
: segments_(segments),
  dispositions_(segments.size(), DISP_PENDING),
  attempts_(segments.size(), 0),
  max_attempts_(options.max_attempts_per_segment)
{
  if (segments_.empty()) {
    throw std::invalid_argument(
      "CoverageTaskCore — cannot build a task from an empty plan");
  }
  if (max_attempts_ < 1) {
    throw std::invalid_argument(
      "CoverageTaskCore — max_attempts_per_segment must be >= 1");
  }
}

std::int32_t CoverageTaskCore::disposition(std::size_t index) const
{
  return dispositions_.at(index);
}

int CoverageTaskCore::attempts(std::size_t index) const
{
  return attempts_.at(index);
}

bool CoverageTaskCore::hasWorkRemaining() const
{
  return nextPendingIndex() >= 0;
}

int CoverageTaskCore::nextPendingIndex() const
{
  for (std::size_t i = 0; i < dispositions_.size(); ++i) {
    if (dispositions_[i] == DISP_PENDING) {
      return static_cast<int>(i);
    }
  }
  for (std::size_t i = 0; i < dispositions_.size(); ++i) {
    if (dispositions_[i] == DISP_BLOCKED_TEMP) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void CoverageTaskCore::markCovered(std::size_t index)
{
  dispositions_.at(index) = DISP_COVERED;
}

void CoverageTaskCore::markFailed(std::size_t index)
{
  dispositions_.at(index) = DISP_FAILED;
}

void CoverageTaskCore::markExempt(std::size_t index)
{
  dispositions_.at(index) = DISP_EXEMPT;
}

void CoverageTaskCore::markBlockedTemp(std::size_t index)
{
  dispositions_.at(index) = DISP_BLOCKED_TEMP;
}

std::size_t CoverageTaskCore::countPending() const
{
  return static_cast<std::size_t>(std::count(
    dispositions_.begin(), dispositions_.end(), DISP_PENDING));
}

std::size_t CoverageTaskCore::countCovered() const
{
  return static_cast<std::size_t>(std::count(
    dispositions_.begin(), dispositions_.end(), DISP_COVERED));
}

std::size_t CoverageTaskCore::countBlockedTemp() const
{
  return static_cast<std::size_t>(std::count(
    dispositions_.begin(), dispositions_.end(), DISP_BLOCKED_TEMP));
}

std::size_t CoverageTaskCore::countExempt() const
{
  return static_cast<std::size_t>(std::count(
    dispositions_.begin(), dispositions_.end(), DISP_EXEMPT));
}

std::size_t CoverageTaskCore::countFailed() const
{
  return static_cast<std::size_t>(std::count(
    dispositions_.begin(), dispositions_.end(), DISP_FAILED));
}

std::int32_t CoverageTaskCore::terminalResult() const
{
  if (countFailed() == 0 && countPending() == 0 && countBlockedTemp() == 0) {
    return countExempt() > 0 ?
      RESULT_SUCCEEDED_WITH_EXEMPTIONS : RESULT_SUCCEEDED_FULL;
  }
  return RESULT_PARTIAL_FAILED;
}

std::string CoverageTaskCore::terminalName(std::int32_t result)
{
  switch (result) {
    case RESULT_SUCCEEDED_FULL:
      return "SUCCEEDED_FULL";
    case RESULT_SUCCEEDED_WITH_EXEMPTIONS:
      return "SUCCEEDED_WITH_EXEMPTIONS";
    case RESULT_PARTIAL_FAILED:
      return "PARTIAL_FAILED";
    case RESULT_CANCELLED:
      return "CANCELLED";
    case RESULT_MAP_CHANGED:
      return "MAP_CHANGED";
    case RESULT_STOP_FAILED:
      return "STOP_FAILED";
    default:
      return "UNKNOWN";
  }
}

void CoverageTaskCore::applyCheckpointDispositions(
  const CheckpointData & checkpoint)
{
  if (checkpoint.segment_ids.size() != segments_.size()) {
    throw std::invalid_argument(
      "CoverageTaskCore — checkpoint segment count differs from plan");
  }
  for (std::size_t i = 0; i < segments_.size(); ++i) {
    if (checkpoint.segment_ids[i] != segments_[i].id) {
      throw std::invalid_argument(
        "CoverageTaskCore — checkpoint segment ids do not match the plan "
        "(residual replanning must not migrate state by index)");
    }
    const auto d = checkpoint.dispositions[i];
    if (d == DISP_COVERED) {
      dispositions_[i] = DISP_COVERED;
    } else if (d == DISP_EXEMPT) {
      dispositions_[i] = DISP_EXEMPT;
    } else if (d == DISP_FAILED) {
      // A previously failed segment may be retried after a fresh start.
      dispositions_[i] = DISP_PENDING;
    }
    // PENDING / BLOCKED_TEMP stay pending.
  }
}

}  // namespace tunnel_coverage_executor
