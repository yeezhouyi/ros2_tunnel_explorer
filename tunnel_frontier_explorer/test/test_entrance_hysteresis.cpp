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

#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>

#include "tunnel_frontier_explorer/entrance_hysteresis.hpp"

namespace tunnel_frontier_explorer
{

struct Pt
{
  double x;
  double y;
};

using Clock = std::chrono::steady_clock;

TEST(EntranceHysteresisTest, RejectsInsideCooldownAndRadius)
{
  const auto now = Clock::now();
  const Pt last{1.0, 1.0};
  const Pt cand{1.2, 1.1};  // 0.22 m from last
  EXPECT_TRUE(entranceHysteresisRejects(
    true, now, std::optional<Pt>{last}, now - std::chrono::seconds(5),
    cand, 20.0, 1.0));
}

TEST(EntranceHysteresisTest, ReleasesAfterCooldownExpiry)
{
  const auto now = Clock::now();
  const Pt last{1.0, 1.0};
  const Pt cand{1.2, 1.1};
  EXPECT_FALSE(entranceHysteresisRejects(
    true, now, std::optional<Pt>{last}, now - std::chrono::seconds(25),
    cand, 20.0, 1.0));
}

TEST(EntranceHysteresisTest, ReleasesOutsideRadius)
{
  const auto now = Clock::now();
  const Pt last{1.0, 1.0};
  const Pt cand{4.0, 4.0};  // far away
  EXPECT_FALSE(entranceHysteresisRejects(
    true, now, std::optional<Pt>{last}, now - std::chrono::seconds(5),
    cand, 20.0, 1.0));
}

TEST(EntranceHysteresisTest, DisabledMeansZeroBehaviourChange)
{
  const auto now = Clock::now();
  const Pt last{1.0, 1.0};
  const Pt cand{1.05, 1.05};  // dead centre of the previous goal
  EXPECT_FALSE(entranceHysteresisRejects(
    false, now, std::optional<Pt>{last}, now - std::chrono::seconds(1),
    cand, 20.0, 1.0));
}

TEST(EntranceHysteresisTest, NoPreviousGoalPasses)
{
  const auto now = Clock::now();
  const Pt cand{1.0, 1.0};
  const std::optional<Pt> none;
  EXPECT_FALSE(entranceHysteresisRejects(
    true, now, none, now, cand, 20.0, 1.0));
}

}  // namespace tunnel_frontier_explorer
