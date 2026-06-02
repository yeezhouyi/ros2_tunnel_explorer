// Copyright 2026 zhouyi
// License: Apache-2.0

#include "tunnel_frontier_explorer/tunnel_geometry_grid.hpp"

#include <gtest/gtest.h>

using tunnel_frontier_explorer::TunnelGrid;
using tunnel_frontier_explorer::TunnelGeometryGrid;

TunnelGrid make_grid(int w, int h, double res)
{
  TunnelGrid g;
  g.width = w; g.height = h; g.resolution = res;
  g.origin_x = 0.0; g.origin_y = 0.0;
  g.data.resize(w * h, 0);
  // Fill with increasing values: 0 at origin, 100 at far corner
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      g.data[y * w + x] = static_cast<int8_t>(
        (static_cast<double>(x + y) / static_cast<double>(w + h - 2)) * 100.0);
  return g;
}

TEST(TunnelGeometryGrid, validCheck)
{
  TunnelGeometryGrid geo;
  EXPECT_FALSE(geo.valid());

  auto g = make_grid(10, 10, 0.05);
  geo.distance_map = g;
  EXPECT_FALSE(geo.valid());  // risk_map not set

  geo.risk_map = g;
  EXPECT_TRUE(geo.valid());
}

TEST(TunnelGeometryGrid, sampleNormalizedDistance)
{
  TunnelGeometryGrid geo;
  geo.distance_map = make_grid(10, 10, 0.05);
  geo.risk_map = make_grid(10, 10, 0.05);

  // Center of grid: world (0.25, 0.25), grid (5,5)
  auto v = geo.sampleCenterlineAlignment({0.25, 0.25});
  ASSERT_TRUE(v.has_value());
  EXPECT_GE(*v, 0.0);
  EXPECT_LE(*v, 1.0);
}

TEST(TunnelGeometryGrid, outOfBounds)
{
  TunnelGeometryGrid geo;
  geo.distance_map = make_grid(10, 10, 0.05);
  geo.risk_map = make_grid(10, 10, 0.05);

  auto v = geo.sampleCenterlineAlignment({100.0, 100.0});
  EXPECT_FALSE(v.has_value());
}

TEST(TunnelGeometryGrid, averageInRadius)
{
  TunnelGeometryGrid geo;
  geo.distance_map = make_grid(10, 10, 0.05);
  geo.risk_map = make_grid(10, 10, 0.05);

  double avg = geo.avgCenterlineInRadius({0.25, 0.25}, 0.1);
  EXPECT_GE(avg, 0.0);
  EXPECT_LE(avg, 1.0);
}

TEST(TunnelGeometryGrid, unknownCellReturnsNullopt)
{
  TunnelGrid g;
  g.width = 5; g.height = 5; g.resolution = 0.05;
  g.origin_x = 0; g.origin_y = 0;
  g.data.resize(25, -1);  // all unknown

  TunnelGeometryGrid geo;
  geo.distance_map = g;
  geo.risk_map = g;

  auto v = geo.sampleCenterlineAlignment({0.1, 0.1});
  EXPECT_FALSE(v.has_value());
}
