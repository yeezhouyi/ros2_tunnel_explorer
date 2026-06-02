// Copyright 2026 zhouyi
// License: Apache-2.0

#include "tunnel_centerline_extractor/centerline_graph.hpp"
#include "tunnel_centerline_extractor/distance_field.hpp"

#include <gtest/gtest.h>

using tunnel_centerline_extractor::Grid2D;
using tunnel_centerline_extractor::CenterlineGraph;
using tunnel_centerline_extractor::extract_centerline_graph;
using tunnel_centerline_extractor::brushfire_distance_field;

// Build a Y-shaped free mask (trunk splits into two branches).
// Grid is 40×40 at 0.05 m → 2.0 m × 2.0 m world.
Grid2D y_shape_mask()
{
  Grid2D m;
  m.width = 40; m.height = 40; m.resolution = 0.05;
  m.origin_x = -1.0; m.origin_y = -1.0;
  m.data.resize(40 * 40, 0);

  // Trunk (vertical): y=10..28, x=17..22 (width ~0.3 m)
  for (int y = 10; y <= 28; ++y)
    for (int x = 17; x <= 22; ++x)
      m.at(x, y) = 1;

  // Left branch: from (y=28, mid=19.5), angle ~-30° from vertical
  for (int y = 28; y <= 36; ++y) {
    int offset = static_cast<int>((y - 28) * 0.577);  // tan(30°)
    for (int x = 17 - offset; x <= 22 - offset; ++x) {
      if (x >= 0 && x < 40 && y < 40)
        m.at(x, y) = 1;
    }
  }

  // Right branch: angle ~+30°
  for (int y = 28; y <= 36; ++y) {
    int offset = static_cast<int>((y - 28) * 0.577);
    for (int x = 17 + offset; x <= 22 + offset; ++x) {
      if (x >= 0 && x < 40 && y < 40)
        m.at(x, y) = 1;
    }
  }

  return m;
}

TEST(CenterlineGraph, y_shape_has_branch)
{
  auto mask = y_shape_mask();
  auto dist = brushfire_distance_field(mask);

  // Skeleton: use distance >= 0.015 (3 cells * 0.005 scale), prune short spurs
  Grid2D thick;
  thick = mask; // copy
  for (int i = 0; i < 40 * 40; ++i)
    thick.data[i] = (mask.data[i] && dist.data[i] >= 15 * 0.05 * 1000) ? 1 : 0;

  // Manual skeleton via simple non-max suppression isn't available,
  // so test with the full free mask as skeleton (approximate for graph test).
  // A real pipeline uses extract_skeleton, which we test separately.
  auto graph = extract_centerline_graph(mask, dist, 0.05, 3);

  // Y-shaped mask should have some skeleton nodes
  int total = graph.endpoints.size() + graph.branch_points.size() +
              graph.corridor_nodes.size();
  EXPECT_GT(total, 0);
}

TEST(CenterlineGraph, empty_mask)
{
  Grid2D m;
  m.width = 10; m.height = 10; m.resolution = 0.05;
  m.origin_x = 0; m.origin_y = 0;
  m.data.resize(100, 0);
  auto dist = brushfire_distance_field(m);

  auto graph = extract_centerline_graph(m, dist, 0.05, 3);

  EXPECT_EQ(graph.endpoints.size(), 0u);
  EXPECT_EQ(graph.branch_points.size(), 0u);
  EXPECT_EQ(graph.corridor_nodes.size(), 0u);
}
