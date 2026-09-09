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

#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

#include "tunnel_coverage_planner/segment_orderer.hpp"

namespace tunnel_coverage_planner
{
namespace
{

std::vector<std::vector<double>> lineCosts(std::size_t n)
{
  std::vector<std::vector<double>> c(n, std::vector<double>(n, 0.0));
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      c[i][j] = i < j ? static_cast<double>(j - i) :
        static_cast<double>(i - j);
    }
  }
  return c;
}

template<typename Fn>
bool throwsInvalidArgument(Fn && fn)
{
  try {
    fn();
    return false;
  } catch (const std::invalid_argument &) {
    return true;
  } catch (...) {
    return false;
  }
}

TEST(SegmentOrdererTest, NearestNeighbourAlongALine)
{
  // 4 cells on a line: 0-1-2-3, start 0 -> 0,1,2,3.
  const auto c = lineCosts(4);
  EXPECT_EQ(SegmentOrderer::order(c, 0), (std::vector<int>{0, 1, 2, 3}));
  // Start at 2 -> 2,1,3? distances: from 2: 1(1),3(1) tie -> smaller index 1
  // first, then 0(1) or 3(2)? from 1: 0(1),3(2) -> 0 then 3.
  EXPECT_EQ(SegmentOrderer::order(c, 2), (std::vector<int>{2, 1, 0, 3}));
}

TEST(SegmentOrdererTest, TieBreakIsDeterministicAndAsymmetricCosts)
{
  std::vector<std::vector<double>> c(3, std::vector<double>(3, 1.0));
  // start 0: 1 and 2 tie at 1.0 -> smaller index first.
  EXPECT_EQ(SegmentOrderer::order(c, 0), (std::vector<int>{0, 1, 2}));

  // Asymmetric: 0->2 cheap, 2->1 cheap.
  c[0][2] = 0.1;
  c[2][1] = 0.1;
  c[0][1] = 5.0;
  c[2][0] = 5.0;
  c[1][2] = 5.0;
  c[1][0] = 5.0;
  EXPECT_EQ(SegmentOrderer::order(c, 0), (std::vector<int>{0, 2, 1}));
}

TEST(SegmentOrdererTest, VisitsEveryCellExactlyOnce)
{
  const auto c = lineCosts(5);
  const auto order = SegmentOrderer::order(c, 3);
  ASSERT_EQ(order.size(), 5u);
  std::set<int> seen(order.begin(), order.end());
  EXPECT_EQ(seen.size(), 5u);
}

TEST(SegmentOrdererTest, RejectsInvalidInput)
{
  EXPECT_TRUE(throwsInvalidArgument([]() {
      SegmentOrderer::order({}, 0);
  }));
  EXPECT_TRUE(throwsInvalidArgument([]() {
      const auto c = lineCosts(3);
      std::vector<std::vector<double>> ragged = c;
      ragged[0].pop_back();
      SegmentOrderer::order(ragged, 0);
  }));
  EXPECT_TRUE(throwsInvalidArgument([]() {
      auto c = lineCosts(3);
      c[0][1] = -1.0;
      SegmentOrderer::order(c, 0);
  }));
  EXPECT_TRUE(throwsInvalidArgument([]() {
      auto c = lineCosts(3);
      c[1][2] = std::numeric_limits<double>::infinity();
      SegmentOrderer::order(c, 0);
  }));
  EXPECT_TRUE(throwsInvalidArgument([]() {
      const auto c = lineCosts(3);
      SegmentOrderer::order(c, 3);
  }));
}

}  // namespace
}  // namespace tunnel_coverage_planner
