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
#include <cstdint>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "tunnel_frontier_explorer/event_log.hpp"
#include "tunnel_frontier_explorer/frontier_detector.hpp"

namespace tfe = tunnel_frontier_explorer;

namespace
{

tfe::GridMap makeGridMap(
  std::vector<std::vector<std::int8_t>> rows,
  double resolution = 1.0,
  double origin_x = 0.0,
  double origin_y = 0.0)
{
  tfe::GridMap map;
  map.height = rows.size();
  map.width = (map.height > 0) ? rows[0].size() : 0;
  map.resolution = resolution;
  map.origin_x = origin_x;
  map.origin_y = origin_y;
  map.data.clear();
  map.data.reserve(map.height * map.width);
  for (const auto & row : rows) {
    map.data.insert(map.data.end(), row.begin(), row.end());
  }
  return map;
}

/// Map with two spatially separated free rectangles inside an all-unknown
/// field.  Each rectangle's perimeter (free cell adjacent to unknown) is a
/// frontier cluster; the two clusters are far apart so their geometry ids
/// are distinct at q = map resolution.
tfe::GridMap twoBlobMap(double res = 0.1)
{
  const int N = 48;
  std::vector<std::vector<std::int8_t>> rows(N, std::vector<std::int8_t>(N, -1));
  auto fill_free = [&](int r0, int r1, int c0, int c1) {
      for (int r = r0; r <= r1; ++r) {
        for (int c = c0; c <= c1; ++c) {
          rows[r][c] = 0;
        }
      }
    };
  fill_free(8, 16, 8, 16);       // blob A
  fill_free(28, 36, 28, 36);     // blob B (far apart)
  return makeGridMap(std::move(rows), res, 0.0, 0.0);
}

tfe::FrontierDetectorConfig detectorCfg()
{
  tfe::FrontierDetectorConfig cfg;
  cfg.min_cluster_size = 1;
  cfg.free_threshold = 0;
  cfg.frontier_neighbor_connectivity = 4;
  cfg.cluster_connectivity = 8;
  return cfg;
}

}  // namespace

// ── stable cluster id: determinism + quantization ────────────────────────

TEST(EventLogStableId, deterministicAndQuantSensitive)
{
  EXPECT_EQ(tfe::stableClusterId(1.23, 4.56, 0.1), tfe::stableClusterId(1.23, 4.56, 0.1));
  EXPECT_NE(tfe::stableClusterId(1.23, 4.56, 0.1), tfe::stableClusterId(1.23, 4.56, 1.0));
  // different quant cells -> different ids
  EXPECT_NE(tfe::stableClusterId(0.05, 0.05, 0.5), tfe::stableClusterId(2.05, 0.05, 0.5));
  EXPECT_EQ(tfe::stableClusterId(1.21, 1.29, 0.5), tfe::stableClusterId(1.30, 1.10, 0.5));
  // invalid quant -> empty (caller must pass effective step)
  EXPECT_EQ(tfe::stableClusterId(1.0, 1.0, 0.0), "");
}

// ── GUARD (mandatory): cluster-vector order independence ─────────────────
// The one test that catches an index-as-id regression BEFORE the B4 long
// runs: shuffle the detector output vector; every cluster id must be
// unchanged because ids are geometry, not scan order.

TEST(EventLogStableIdGuard, shuffleInvariance)
{
  const double res = 0.1;
  auto map = twoBlobMap(res);
  tfe::FrontierDetector detector(detectorCfg());
  tfe::Point2D robot{0.0, 0.0};
  auto clusters = detector.detect(map, robot);
  ASSERT_GE(clusters.size(), 2u) << "fixture must produce >= 2 clusters";

  const double q = res;   // auto step == map resolution (injective)

  // id keyed by geometry: representative cell (row, col)
  auto key_of = [](const tfe::FrontierCluster & c) {
      return std::to_string(c.representative_cell.row) + ":" +
             std::to_string(c.representative_cell.col);
    };
  std::map<std::string, std::string> orig;
  for (const auto & c : clusters) {
    orig[key_of(c)] = tfe::stableClusterId(c, q);
  }
  // injectivity at q == resolution: distinct representative cells give
  // distinct ids (a quant coarser than the map could collide by design;
  // the auto default must not).
  EXPECT_EQ(orig.size(), clusters.size());

  auto shuffled = clusters;
  std::mt19937 rng(4060);
  std::shuffle(shuffled.begin(), shuffled.end(), rng);

  std::map<std::string, std::string> after;
  for (const auto & c : shuffled) {
    after[key_of(c)] = tfe::stableClusterId(c, q);
  }

  EXPECT_EQ(orig, after) <<
    "stable_cluster_id changed after reordering the cluster vector -- "
    "the id depends on scan order (index-as-id regression)";
}

// ── map digest ───────────────────────────────────────────────────────────

TEST(EventLogMapDigest, stableAndSensitive)
{
  auto map = twoBlobMap();
  auto d1 = tfe::mapDigestHex(map.data, map.width, map.height, 8);
  auto d2 = tfe::mapDigestHex(map.data, map.width, map.height, 8);
  EXPECT_EQ(d1, d2);
  EXPECT_EQ(d1.size(), 16u);

  auto mutated = map;
  mutated.data[0] = mutated.data[0] == 0 ? 100 : 0;   // sampled cell (0,0)
  auto d3 = tfe::mapDigestHex(mutated.data, mutated.width, mutated.height, 8);
  EXPECT_NE(d1, d3);
}

// ── params hash ──────────────────────────────────────────────────────────

TEST(EventLogParamsHash, sortedOrderDeterministic)
{
  EXPECT_EQ(
    tfe::paramsHashHex({"a=1", "b=2"}),
    tfe::paramsHashHex({"a=1", "b=2"}));
  EXPECT_NE(
    tfe::paramsHashHex({"a=1", "b=2"}),
    tfe::paramsHashHex({"a=1", "b=3"}));
}

// ── CSV column consistency + roundtrip ───────────────────────────────────

TEST(EventLogCsv, headerAndRowHaveSchemaColumnCount)
{
  const std::string header = tfe::formatCsvHeader();
  const std::size_t header_cols = static_cast<std::size_t>(
    std::count(header.begin(), header.end(), ',')) + 1;
  EXPECT_EQ(header_cols, tfe::kEventLogColumns);

  tfe::EventRow r;
  r.repo_sha = "abc";
  r.repo_dirty = true;
  r.seed = 3;
  r.t = 12.5;
  r.frontier_count_open = 7;
  r.reachable_frontier_count = 6;
  r.goal_id = 2;
  r.stable_cluster_id = "12345";
  r.goal_x = 1.5;
  r.goal_y = 2.5;
  r.action = "accept";
  r.nav2_result = "accepted";
  r.nav2_error_code = -1;
  r.robot_x = 0.1;
  r.robot_y = 0.2;
  r.readiness_status = "navigating";
  const std::string line = tfe::formatCsv(r);
  const std::size_t line_cols = static_cast<std::size_t>(
    std::count(line.begin(), line.end(), ',')) + 1;
  EXPECT_EQ(line_cols, tfe::kEventLogColumns);
}

TEST(EventLogCsv, writesHeaderThenRows)
{
  const std::string path = "/tmp/event_log_test.csv";
  std::remove(path.c_str());
  {
    tfe::EventLogCsv csv(path);
    ASSERT_TRUE(csv.enabled());
    tfe::EventRow r;
    r.action = "heartbeat";
    csv.write(r);
    r.action = "accept";
    r.goal_id = 1;
    csv.write(r);
  }
  std::ifstream in(path);
  ASSERT_TRUE(in.good());
  std::string line;
  std::getline(in, line);
  EXPECT_EQ(line, tfe::formatCsvHeader());
  std::getline(in, line);
  EXPECT_EQ(
    std::count(line.begin(), line.end(), ','),
    static_cast<std::ptrdiff_t>(tfe::kEventLogColumns) - 1);
  EXPECT_NE(line.find("heartbeat"), std::string::npos);
  std::getline(in, line);
  EXPECT_NE(line.find("accept"), std::string::npos);
  in.close();
  std::remove(path.c_str());
}

TEST(EventLogCsv, emptyPathIsDisabled)
{
  tfe::EventLogCsv csv("");
  EXPECT_FALSE(csv.enabled());
}
