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

#include <optional>

#include "tunnel_coverage_executor/stop_confirm.hpp"

namespace tunnel_coverage_executor
{
namespace
{

using std::nullopt;

StopSample sample(double x, double y, double yaw)
{
  StopSample s;
  s.x = x;
  s.y = y;
  s.yaw = yaw;
  return s;
}

TEST(StopConfirm, FirstValidSampleIsUnknownNotStationary)
{
  // No previous sample => cannot confirm anything yet (unknown).
  EXPECT_EQ(classifyStopSample(nullopt, sample(1.0, 2.0, 0.1), 0.01, 0.03), -1);
}

TEST(StopConfirm, StationaryWithinBothTranslationAndRotationBounds)
{
  const auto prev = sample(0.0, 0.0, 0.0);
  EXPECT_EQ(classifyStopSample(prev, sample(0.002, 0.001, 0.01), 0.01, 0.03), 0);
}

TEST(StopConfirm, RotationAloneCountsAsMotion)
{
  // Zero translation but a yaw step beyond the bound => moving.
  // (The pre-fix logic checked translation only and would have missed this.)
  const auto prev = sample(0.0, 0.0, 0.0);
  EXPECT_EQ(classifyStopSample(prev, sample(0.0, 0.0, 0.10), 0.01, 0.03), 1);
}

TEST(StopConfirm, TranslationAloneCountsAsMotion)
{
  const auto prev = sample(0.0, 0.0, 0.0);
  EXPECT_EQ(classifyStopSample(prev, sample(0.02, 0.0, 0.0), 0.01, 0.03), 1);
}

TEST(StopConfirm, LargeYawDifferenceIsWrapped)
{
  // 2.9 -> -2.9 raw delta -5.8 rad wraps to ~+0.483 rad => moving.
  const auto prev = sample(0.0, 0.0, 2.9);
  const auto cur = sample(0.0, 0.0, -2.9);
  EXPECT_EQ(classifyStopSample(prev, cur, 0.01, 0.03), 1);
}

TEST(StopConfirm, WrappedSmallYawDifferenceIsStationary)
{
  // 3.1 -> -3.1 raw delta -6.2 rad wraps to ~+0.083? No: wraps to 0.083 < 0.03?
  // -6.2 + 2*pi = +0.0832 rad.  With bound 0.2 this is stationary.
  const auto prev = sample(0.0, 0.0, 3.1);
  const auto cur = sample(0.0, 0.0, -3.1);
  EXPECT_EQ(classifyStopSample(prev, cur, 0.01, 0.2), 0);
}

}  // namespace
}  // namespace tunnel_coverage_executor
