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

StopSample sample(double x, double y, double yaw, double stamp_s)
{
  StopSample s;
  s.x = x;
  s.y = y;
  s.yaw = yaw;
  s.stamp_s = stamp_s;
  return s;
}

// Per-second thresholds used throughout: 0.05 m/s, 0.15 rad/s defaults match
// the node's stop_velocity_threshold_mps / stop_yaw_threshold_radps.
constexpr double kSpeed = 0.05;      // m/s
constexpr double kYawRate = 0.15;    // rad/s

TEST(StopConfirm, FirstValidSampleIsUnknownNotStationary)
{
  // No previous sample => cannot confirm anything yet (unknown).
  EXPECT_EQ(classifyStopSampleTimed(nullopt, sample(1.0, 2.0, 0.1, 1.0),
    kSpeed, kYawRate), -1);
}

TEST(StopConfirm, RepeatedIdenticalSampleIsNotANewObservation)
{
  // Same localization AND same stamp read again (frozen TF / same message):
  // dt == 0 => unknown, NEVER stationary.  Re-reading one localization must
  // not accumulate into a stop confirmation.
  const auto prev = sample(0.0, 0.0, 0.0, 1.000);
  const auto cur = sample(0.0, 0.0, 0.0, 1.000);  // same stamp, same pose
  EXPECT_EQ(classifyStopSampleTimed(prev, cur, kSpeed, kYawRate), -1);
}

TEST(StopConfirm, SamePoseAdvancingStampIsStationary)
{
  // Robot is actually still, but TF keeps publishing new stamps:
  // dt > 0 and motion within bounds => stationary (valid confirmation path).
  const auto prev = sample(0.0, 0.0, 0.0, 1.000);
  const auto cur = sample(0.0, 0.0, 0.0, 1.500);  // new stamp, no motion
  EXPECT_EQ(classifyStopSampleTimed(prev, cur, kSpeed, kYawRate), 0);
}

TEST(StopConfirm, StationaryWithinBoundsOverRealInterval)
{
  // 0.01 m translation over a 0.5 s interval; bound = 0.05 m/s * 0.5 s.
  const auto prev = sample(0.0, 0.0, 0.0, 0.0);
  const auto cur = sample(0.008, 0.006, 0.02, 0.5);
  EXPECT_EQ(classifyStopSampleTimed(prev, cur, kSpeed, kYawRate), 0);
}

TEST(StopConfirm, RotationAloneCountsAsMotion)
{
  // Zero translation but a yaw step beyond the per-step bound => moving.
  const auto prev = sample(0.0, 0.0, 0.0, 0.0);
  const auto cur = sample(0.0, 0.0, 0.10, 0.5);  // 0.2 rad/s > 0.15
  EXPECT_EQ(classifyStopSampleTimed(prev, cur, kSpeed, kYawRate), 1);
}

TEST(StopConfirm, TranslationAloneCountsAsMotion)
{
  const auto prev = sample(0.0, 0.0, 0.0, 0.0);
  const auto cur = sample(0.03, 0.0, 0.0, 0.5);  // 0.06 m/s > 0.05
  EXPECT_EQ(classifyStopSampleTimed(prev, cur, kSpeed, kYawRate), 1);
}

TEST(StopConfirm, SlowDriftIsStationaryFastDriftIsMotionSameDisplacement)
{
  // The SAME displacement must be judged by the real interval: 0.02 m over
  // 1.0 s (0.02 m/s < 0.05) is stationary; 0.02 m over 0.2 s (0.10 m/s) is
  // motion.  A fixed "0.2 s conversion" would label both the same.
  const auto prev = sample(0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(classifyStopSampleTimed(prev, sample(0.02, 0.0, 0.0, 1.0),
    kSpeed, kYawRate), 0);
  EXPECT_EQ(classifyStopSampleTimed(prev, sample(0.02, 0.0, 0.0, 0.2),
    kSpeed, kYawRate), 1);
}

TEST(StopConfirm, GapTooLargeIsUnknown)
{
  // > max_gap_s (default 2.0) between samples: not one integration step.
  const auto prev = sample(0.0, 0.0, 0.0, 0.0);
  const auto cur = sample(0.001, 0.0, 0.0, 3.0);
  EXPECT_EQ(classifyStopSampleTimed(prev, cur, kSpeed, kYawRate), -1);
}

TEST(StopConfirm, BackwardsStampIsUnknown)
{
  // Non-monotonic stamp (TF regression): unknown, never stationary.
  const auto prev = sample(0.0, 0.0, 0.0, 2.0);
  const auto cur = sample(0.0, 0.0, 0.0, 1.0);
  EXPECT_EQ(classifyStopSampleTimed(prev, cur, kSpeed, kYawRate), -1);
}

TEST(StopConfirm, LargeYawDifferenceIsWrapped)
{
  // 2.9 -> -2.9 raw delta -5.8 rad wraps to ~+0.483 rad => moving.
  const auto prev = sample(0.0, 0.0, 2.9, 0.0);
  const auto cur = sample(0.0, 0.0, -2.9, 1.0);
  EXPECT_EQ(classifyStopSampleTimed(prev, cur, kSpeed, 0.2), 1);
}

TEST(StopConfirm, WrappedSmallYawDifferenceIsStationary)
{
  // 3.1 -> -3.1 raw delta -6.2 rad wraps to ~+0.083 rad < 0.2 rad/s * 1 s.
  const auto prev = sample(0.0, 0.0, 3.1, 0.0);
  const auto cur = sample(0.0, 0.0, -3.1, 1.0);
  EXPECT_EQ(classifyStopSampleTimed(prev, cur, kSpeed, 0.2), 0);
}

}  // namespace
}  // namespace tunnel_coverage_executor
