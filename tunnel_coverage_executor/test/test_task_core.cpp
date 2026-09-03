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

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "tunnel_coverage_executor/checkpoint_store.hpp"
#include "tunnel_coverage_executor/coverage_task_core.hpp"
#include "tunnel_coverage_planner/scanline_planner.hpp"

namespace tunnel_coverage_executor
{
namespace
{

using tunnel_coverage_planner::CoverageSegment;
using tunnel_coverage_planner::SegmentType;

std::vector<CoverageSegment> makeSegments(std::size_t n)
{
  std::vector<CoverageSegment> segs;
  for (std::size_t i = 0; i < n; ++i) {
    CoverageSegment s;
    s.id = "w" + std::to_string(i);
    s.cell_id = "room0";
    s.type = SegmentType::WORK;
    s.start_x = static_cast<double>(i);
    s.end_x = static_cast<double>(i) + 1.0;
    segs.push_back(s);
  }
  return segs;
}

TEST(CoverageTaskCoreTest, SequentialDispositions)
{
  auto segs = makeSegments(4);
  CoverageTaskCore core(segs, CoverageTaskCore::Options{});

  EXPECT_TRUE(core.hasWorkRemaining());
  EXPECT_EQ(core.nextPendingIndex(), 0);
  EXPECT_EQ(core.disposition(0), DISP_PENDING);

  core.markCovered(0);
  core.markCovered(1);
  core.markFailed(2);
  core.markExempt(3);

  EXPECT_FALSE(core.hasWorkRemaining());
  EXPECT_EQ(core.countCovered(), 2u);
  EXPECT_EQ(core.countFailed(), 1u);
  EXPECT_EQ(core.countExempt(), 1u);
  EXPECT_EQ(core.countPending(), 0u);

  // Failed segments force PARTIAL_FAILED even when everything else is done.
  EXPECT_EQ(core.terminalResult(), RESULT_PARTIAL_FAILED);
}

TEST(CoverageTaskCoreTest, TerminalClassification)
{
  auto segs = makeSegments(2);
  CoverageTaskCore core(segs, CoverageTaskCore::Options{});
  core.markCovered(0);
  core.markCovered(1);
  EXPECT_EQ(core.terminalResult(), RESULT_SUCCEEDED_FULL);

  CoverageTaskCore core2(segs, CoverageTaskCore::Options{});
  core2.markCovered(0);
  core2.markExempt(1);
  EXPECT_EQ(core2.terminalResult(), RESULT_SUCCEEDED_WITH_EXEMPTIONS);

  CoverageTaskCore core3(segs, CoverageTaskCore::Options{});
  core3.markFailed(0);
  core3.markExempt(1);
  EXPECT_EQ(core3.terminalResult(), RESULT_PARTIAL_FAILED);

  EXPECT_EQ(CoverageTaskCore::terminalName(RESULT_SUCCEEDED_FULL),
    "SUCCEEDED_FULL");
  EXPECT_EQ(CoverageTaskCore::terminalName(RESULT_MAP_CHANGED), "MAP_CHANGED");
}

TEST(CoverageTaskCoreTest, BlockedTempIsRevisitedAfterPending)
{
  auto segs = makeSegments(3);
  CoverageTaskCore core(segs, CoverageTaskCore::Options{});
  core.markBlockedTemp(1);  // segment 1 temporarily blocked
  // Pending first, then blocked.
  EXPECT_EQ(core.nextPendingIndex(), 0);
  core.markCovered(0);
  EXPECT_EQ(core.nextPendingIndex(), 2);
  core.markCovered(2);
  // Blocked segment comes back last.
  EXPECT_EQ(core.nextPendingIndex(), 1);
  EXPECT_TRUE(core.hasWorkRemaining());
}

TEST(CoverageTaskCoreTest, RejectsEmptyPlan)
{
  bool threw = false;
  try {
    CoverageTaskCore core(
      std::vector<CoverageSegment>{}, CoverageTaskCore::Options{});
    (void)core;
  } catch (const std::invalid_argument &) {
    threw = true;
  } catch (...) {
    // wrong exception type
  }
  EXPECT_TRUE(threw);
}

TEST(CoverageTaskCoreTest, CheckpointDispositionsApplyById)
{
  auto segs = makeSegments(3);
  CoverageTaskCore core(segs, CoverageTaskCore::Options{});

  CheckpointData cp;
  cp.plan_id = "plan_x";
  cp.segment_ids = {"w0", "w1", "w2"};
  cp.dispositions = {DISP_COVERED, DISP_PENDING, DISP_EXEMPT};

  core.applyCheckpointDispositions(cp);
  EXPECT_EQ(core.disposition(0), DISP_COVERED);
  EXPECT_EQ(core.disposition(1), DISP_PENDING);
  EXPECT_EQ(core.disposition(2), DISP_EXEMPT);
  EXPECT_EQ(core.countCovered(), 1u);
  EXPECT_EQ(core.countPending(), 1u);
}

TEST(CoverageTaskCoreTest, CheckpointIdMismatchThrows)
{
  auto segs = makeSegments(2);
  CoverageTaskCore core(segs, CoverageTaskCore::Options{});

  // Same count but different ids -> refuse (R22: no index-based migration).
  CheckpointData cp;
  cp.segment_ids = {"other0", "other1"};
  cp.dispositions = {DISP_COVERED, DISP_COVERED};
  {
    bool threw = false;
    try {
      core.applyCheckpointDispositions(cp);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    EXPECT_TRUE(threw);
  }

  // Different count -> refuse.
  CheckpointData cp2;
  cp2.segment_ids = {"w0"};
  cp2.dispositions = {DISP_COVERED};
  {
    bool threw = false;
    try {
      core.applyCheckpointDispositions(cp2);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    EXPECT_TRUE(threw);
  }
}

}  // namespace
}  // namespace tunnel_coverage_executor
