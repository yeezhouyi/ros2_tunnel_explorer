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

#include "tunnel_coverage_executor/coverage_task_core.hpp"

namespace tunnel_coverage_executor
{
namespace
{

using CoverageTaskCore = tunnel_coverage_executor::CoverageTaskCore;

TEST(CoverageGate, BarClearedKeepsFullSuccess)
{
  EXPECT_EQ(CoverageTaskCore::resolveCoverageGate(
      RESULT_SUCCEEDED_FULL, 0.98, 0.97), RESULT_SUCCEEDED_FULL);
}

TEST(CoverageGate, BarClearedKeepsExemptionSuccess)
{
  EXPECT_EQ(CoverageTaskCore::resolveCoverageGate(
      RESULT_SUCCEEDED_WITH_EXEMPTIONS, 0.98, 0.97),
    RESULT_SUCCEEDED_WITH_EXEMPTIONS);
}

TEST(CoverageGate, BarMissedWithExemptionsIsPartialFailure)
{
  // Review case: effective 50% < required 97% with exempt_ratio 10% must NOT
  // be re-branded SUCCEEDED_WITH_EXEMPTIONS.  Effective coverage already
  // excluded the exempt area, so exemptions cannot excuse the miss.
  EXPECT_EQ(CoverageTaskCore::resolveCoverageGate(
      RESULT_SUCCEEDED_WITH_EXEMPTIONS, 0.50, 0.97), RESULT_PARTIAL_FAILED);
}

TEST(CoverageGate, BarMissedFullSuccessIsPartialFailure)
{
  EXPECT_EQ(CoverageTaskCore::resolveCoverageGate(
      RESULT_SUCCEEDED_FULL, 0.50, 0.97), RESULT_PARTIAL_FAILED);
}

TEST(CoverageGate, NonSuccessTerminalsPassThrough)
{
  EXPECT_EQ(CoverageTaskCore::resolveCoverageGate(
      RESULT_CANCELLED, 0.50, 0.97), RESULT_CANCELLED);
  EXPECT_EQ(CoverageTaskCore::resolveCoverageGate(
      RESULT_STOP_FAILED, 0.98, 0.97), RESULT_STOP_FAILED);
  EXPECT_EQ(CoverageTaskCore::resolveCoverageGate(
      RESULT_MAP_CHANGED, 0.98, 0.97), RESULT_MAP_CHANGED);
  EXPECT_EQ(CoverageTaskCore::resolveCoverageGate(
      RESULT_PARTIAL_FAILED, 0.98, 0.97), RESULT_PARTIAL_FAILED);
}

TEST(CoverageGate, EpsilonBoundary)
{
  // effective == min within the 1e-9 tolerance keeps the success class.
  EXPECT_EQ(CoverageTaskCore::resolveCoverageGate(
      RESULT_SUCCEEDED_FULL, 0.97 - 1e-10, 0.97), RESULT_SUCCEEDED_FULL);
  // clearly below the bar (outside the tolerance) collapses to partial.
  EXPECT_EQ(CoverageTaskCore::resolveCoverageGate(
      RESULT_SUCCEEDED_WITH_EXEMPTIONS, 0.97 - 1e-4, 0.97),
    RESULT_PARTIAL_FAILED);
}

}  // namespace
}  // namespace tunnel_coverage_executor
