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

#include "tunnel_frontier_explorer/frontier_blacklist.hpp"

namespace tfe = tunnel_frontier_explorer;

// ── Helpers ──────────────────────────────────────────────────────────────

/// Convenience: create a time point offset from a base.
auto t0 = std::chrono::steady_clock::time_point{};  // epoch
auto tp(int seconds) {return t0 + std::chrono::seconds(seconds);}

// ── 1. Empty blacklist ───────────────────────────────────────────────────

TEST(FrontierBlacklistTest, emptyBlacklist)
{
  tfe::FrontierBlacklist bl;
  EXPECT_EQ(bl.size(), 0u);
  EXPECT_FALSE(bl.contains({0, 0}, tp(0)));
}

// ── 2. Single entry within radius ───────────────────────────────────────

TEST(FrontierBlacklistTest, singleEntryWithinRadius)
{
  tfe::FrontierBlacklist bl;
  bl.add({0, 0}, tp(0), 1.0, std::chrono::seconds(10));
  EXPECT_GE(bl.size(), 1u);
  // Exactly at centre → blocked
  EXPECT_TRUE(bl.contains({0, 0}, tp(0)));
  // At (0.5, 0) → within radius → blocked
  EXPECT_TRUE(bl.contains({0.5, 0}, tp(0)));
  // At (0, 0.5) → within radius → blocked
  EXPECT_TRUE(bl.contains({0, 0.5}, tp(0)));
}

// ── 3. Multiple entries ──────────────────────────────────────────────────

TEST(FrontierBlacklistTest, multipleEntries)
{
  tfe::FrontierBlacklist bl;
  bl.add({0, 0}, tp(0), 1.0, std::chrono::seconds(10));
  bl.add({5, 5}, tp(0), 1.0, std::chrono::seconds(10));
  bl.add({10, 10}, tp(0), 1.0, std::chrono::seconds(10));
  EXPECT_TRUE(bl.contains({0, 0}, tp(0)));
  EXPECT_TRUE(bl.contains({5, 5}, tp(0)));
  EXPECT_TRUE(bl.contains({10, 10}, tp(0)));
  // Between entries but outside radius → not blocked
  EXPECT_FALSE(bl.contains({2.5, 2.5}, tp(0)));
}

// ── 4. Expired entry ─────────────────────────────────────────────────────

TEST(FrontierBlacklistTest, expiredEntry)
{
  tfe::FrontierBlacklist bl;
  // timeout=0 means it expires immediately
  bl.add({0, 0}, tp(0), 1.0, std::chrono::seconds(0));
  EXPECT_FALSE(bl.contains({0, 0}, tp(0)));
}

// ── 5. Radius edge boundary ──────────────────────────────────────────────

TEST(FrontierBlacklistTest, radiusEdgeBoundary)
{
  tfe::FrontierBlacklist bl;
  bl.add({0, 0}, tp(0), 1.0, std::chrono::seconds(10));
  // Exactly at radius boundary → blocked
  EXPECT_TRUE(bl.contains({1.0, 0}, tp(0)));
  EXPECT_TRUE(bl.contains({0, 1.0}, tp(0)));
  // Just beyond radius → not blocked
  EXPECT_FALSE(bl.contains({1.001, 0}, tp(0)));
  EXPECT_FALSE(bl.contains({0, 1.001}, tp(0)));
}

// ── 6. Far centroid ──────────────────────────────────────────────────────

TEST(FrontierBlacklistTest, farCentroidNotBlacklisted)
{
  tfe::FrontierBlacklist bl;
  bl.add({0, 0}, tp(0), 1.0, std::chrono::seconds(10));
  EXPECT_FALSE(bl.contains({10, 10}, tp(0)));
  EXPECT_FALSE(bl.contains({-10, -10}, tp(0)));
}

// ── 7. Cleanup removes expired entries ───────────────────────────────────

TEST(FrontierBlacklistTest, cleanupRemovesExpired)
{
  tfe::FrontierBlacklist bl;
  bl.add({0, 0}, tp(0), 1.0, std::chrono::seconds(0));     // expires at tp(0)
  bl.add({5, 5}, tp(0), 1.0, std::chrono::seconds(10));    // expires at tp(10)
  EXPECT_EQ(bl.size(), 2u);
  bl.cleanup(tp(5));  // now=tp(5): first expired, second still valid
  EXPECT_EQ(bl.size(), 1u);
  EXPECT_TRUE(bl.contains({5, 5}, tp(5)));
  EXPECT_FALSE(bl.contains({0, 0}, tp(5)));
}

// ── 8. Re-add after cleanup works ────────────────────────────────────────

TEST(FrontierBlacklistTest, readdAfterCleanup)
{
  tfe::FrontierBlacklist bl;
  bl.add({0, 0}, tp(0), 1.0, std::chrono::seconds(0));  // expires immediately
  bl.cleanup(tp(1));
  EXPECT_EQ(bl.size(), 0u);
  // Re-add the same position with a valid timeout.
  bl.add({0, 0}, tp(1), 1.0, std::chrono::seconds(10));
  EXPECT_TRUE(bl.contains({0, 0}, tp(1)));
}

// ── 9. Time boundary: not expired at exact expiry time ───────────────────

TEST(FrontierBlacklistTest, exactExpiryTime)
{
  tfe::FrontierBlacklist bl;
  // Added at t=0, expires in 10 seconds → expires_at = t=10
  bl.add({0, 0}, tp(0), 1.0, std::chrono::seconds(10));
  // At t=10 exactly: now >= expires_at → expired, should be false
  EXPECT_FALSE(bl.contains({0, 0}, tp(10)));
  // At t=9: still valid
  EXPECT_TRUE(bl.contains({0, 0}, tp(9)));
}
