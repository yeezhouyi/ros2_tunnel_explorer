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

#include <cstddef>

#include <gtest/gtest.h>

#include "tunnel_frontier_explorer/frontier_visit_history.hpp"

namespace tunnel_frontier_explorer
{

TEST(FrontierVisitHistoryTest, emptyHistory)
{
  FrontierVisitHistory h;
  Point2D p{1.0, 2.0};
  EXPECT_EQ(h.countVisitsNear(p, 1.0), 0u);
  EXPECT_EQ(h.size(), 0u);
}

TEST(FrontierVisitHistoryTest, singleRecord)
{
  FrontierVisitHistory h;
  Point2D p{1.0, 2.0};
  h.recordAcceptedGoal(p);
  EXPECT_EQ(h.size(), 1u);
  EXPECT_EQ(h.countVisitsNear(p, 1.0), 1u);
}

TEST(FrontierVisitHistoryTest, withinRadius)
{
  FrontierVisitHistory h;
  Point2D recorded{1.0, 2.0};
  h.recordAcceptedGoal(recorded);

  Point2D nearby{1.3, 2.1};  // ~0.36 m away
  EXPECT_EQ(h.countVisitsNear(nearby, 0.5), 1u);
}

TEST(FrontierVisitHistoryTest, outsideRadius)
{
  FrontierVisitHistory h;
  Point2D recorded{0.0, 0.0};
  h.recordAcceptedGoal(recorded);

  Point2D far{2.0, 0.0};  // 2 m away
  EXPECT_EQ(h.countVisitsNear(far, 1.0), 0u);
}

TEST(FrontierVisitHistoryTest, exactBoundary)
{
  FrontierVisitHistory h;
  Point2D recorded{0.0, 0.0};
  h.recordAcceptedGoal(recorded);

  Point2D boundary{1.0, 0.0};  // exactly 1 m away
  EXPECT_EQ(h.countVisitsNear(boundary, 1.0), 1u);
}

TEST(FrontierVisitHistoryTest, multipleRecordsSameArea)
{
  FrontierVisitHistory h;
  Point2D a{0.0, 0.0};
  Point2D b{0.2, 0.1};
  h.recordAcceptedGoal(a);
  h.recordAcceptedGoal(b);

  Point2D query{0.1, 0.0};  // near both
  EXPECT_EQ(h.countVisitsNear(query, 0.5), 2u);
}

TEST(FrontierVisitHistoryTest, multipleRecordsDifferentArea)
{
  FrontierVisitHistory h;
  Point2D a{0.0, 0.0};
  Point2D b{10.0, 10.0};
  h.recordAcceptedGoal(a);
  h.recordAcceptedGoal(b);

  Point2D query{0.1, 0.0};
  EXPECT_EQ(h.countVisitsNear(query, 0.5), 1u);  // only a is near
}

TEST(FrontierVisitHistoryTest, clearResets)
{
  FrontierVisitHistory h;
  h.recordAcceptedGoal({1.0, 2.0});
  EXPECT_EQ(h.size(), 1u);

  h.clear();
  EXPECT_EQ(h.size(), 0u);
  EXPECT_EQ(h.countVisitsNear({1.0, 2.0}, 0.5), 0u);
}

TEST(FrontierVisitHistoryTest, multipleRecordsCountedCorrectly)
{
  FrontierVisitHistory h;
  Point2D p{5.0, 5.0};
  for (int i = 0; i < 5; ++i) {
    h.recordAcceptedGoal(p);
  }
  EXPECT_EQ(h.size(), 5u);
  EXPECT_EQ(h.countVisitsNear(p, 1.0), 5u);
}

}  // namespace tunnel_frontier_explorer
