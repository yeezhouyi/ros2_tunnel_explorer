// Copyright 2026 zhouyi
// License: Apache-2.0

#include "tunnel_frontier_explorer/frontier_scorer.hpp"
#include "tunnel_frontier_explorer/tunnel_geometry_grid.hpp"

#include <gtest/gtest.h>
#include <cmath>

using tunnel_frontier_explorer::FrontierCluster;
using tunnel_frontier_explorer::FrontierScorer;
using tunnel_frontier_explorer::FrontierScorerConfig;
using tunnel_frontier_explorer::FrontierVisitHistory;
using tunnel_frontier_explorer::GridMap;
using tunnel_frontier_explorer::Point2D;
using tunnel_frontier_explorer::TunnelGeometryGrid;
using tunnel_frontier_explorer::TunnelGrid;

// Helper: build a simple GridMap with some unknown cells.
GridMap make_map(int w, int h, double res)
{
  GridMap m;
  m.width = w; m.height = h; m.resolution = res;
  m.origin_x = -static_cast<double>(w) * res * 0.5;
  m.origin_y = -static_cast<double>(h) * res * 0.5;
  m.data.resize(w * h, 0);
  // Add a band of unknown cells at the right edge
  for (int y = 0; y < h; ++y)
    for (int x = w * 3 / 4; x < w; ++x)
      m.data[y * w + x] = -1;
  return m;
}

TunnelGeometryGrid make_geometry()
{
  TunnelGrid g;
  g.width = 100; g.height = 100; g.resolution = 0.05;
  g.origin_x = -2.5; g.origin_y = -2.5;
  g.data.resize(100 * 100, 50);  // moderate distance/risk

  TunnelGeometryGrid geo;
  geo.distance_map = g;
  geo.risk_map = g;
  return geo;
}

TEST(FrontierScorerTunnelAware, FallbackMatchesStage3D)
{
  auto map = make_map(40, 40, 0.05);
  FrontierVisitHistory history;

  FrontierCluster c1;
  c1.representative_world = {0.4, 0.0};
  c1.goal_distance_to_robot = 1.0;
  FrontierCluster c2;
  c2.representative_world = {0.8, 0.0};
  c2.goal_distance_to_robot = 1.5;

  FrontierScorerConfig cfg;
  cfg.information_gain_radius_meters = 0.25;
  cfg.revisit_radius_meters = 0.3;
  cfg.max_revisit_count = 3;
  FrontierScorer scorer(cfg);

  TunnelGeometryGrid invalid_geo;  // not valid
  auto result_3d = scorer.scoreAndRank({c1, c2}, map, {0, 0}, history);
  auto result_4b = scorer.scoreAndRankTunnelAware({c1, c2}, map, {0, 0}, history, invalid_geo);

  ASSERT_EQ(result_3d.size(), result_4b.size());
  for (size_t i = 0; i < result_3d.size(); ++i) {
    EXPECT_NEAR(result_3d[i].score, result_4b[i].score, 1e-6);
  }
}

TEST(FrontierScorerTunnelAware, DeterministicTieBreak)
{
  auto map = make_map(40, 40, 0.05);
  FrontierVisitHistory history;
  auto geo = make_geometry();

  FrontierCluster c1, c2;
  c1.representative_world = {0.3, 0.0}; c1.goal_distance_to_robot = 1.0;
  c2.representative_world = {0.3, 0.0}; c2.goal_distance_to_robot = 1.0;

  FrontierScorerConfig cfg;
  FrontierScorer scorer(cfg);

  // Run twice — same inputs should produce same order
  auto r1 = scorer.scoreAndRankTunnelAware({c1, c2}, map, {0, 0}, history, geo);
  auto r2 = scorer.scoreAndRankTunnelAware({c1, c2}, map, {0, 0}, history, geo);

  ASSERT_EQ(r1.size(), 2u);
  EXPECT_NEAR(r1[0].score, r2[0].score, 1e-6);
}

TEST(FrontierScorerTunnelAware, EmptyCandidates)
{
  auto map = make_map(40, 40, 0.05);
  FrontierVisitHistory history;
  auto geo = make_geometry();
  FrontierScorer scorer;

  auto result = scorer.scoreAndRankTunnelAware({}, map, {0, 0}, history, geo);
  EXPECT_TRUE(result.empty());
}
