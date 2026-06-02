// Copyright 2026 zhouyi
// License: Apache-2.0

#include "tunnel_centerline_extractor/distance_field.hpp"

#include <gtest/gtest.h>

using tunnel_centerline_extractor::Grid2D;
using tunnel_centerline_extractor::brushfire_distance_field;

Grid2D make_mask(int w, int h)
{
  Grid2D m;
  m.width = w; m.height = h; m.resolution = 0.05;
  m.origin_x = 0; m.origin_y = 0;
  m.data.resize(w * h, 1);  // all free
  return m;
}

TEST(DistanceField, blocked_cells_zero)
{
  auto m = make_mask(10, 10);
  // Set border as blocked
  for (int x = 0; x < 10; ++x) m.at(x, 0) = 0;
  for (int x = 0; x < 10; ++x) m.at(x, 9) = 0;
  for (int y = 0; y < 10; ++y) m.at(0, y) = 0;
  for (int y = 0; y < 10; ++y) m.at(9, y) = 0;

  auto d = brushfire_distance_field(m);

  EXPECT_EQ(d.at(0, 0), 0);
  EXPECT_EQ(d.at(0, 9), 0);
  EXPECT_EQ(d.at(9, 0), 0);
  EXPECT_EQ(d.at(9, 9), 0);
  // Center cell should be farthest from walls
  EXPECT_GT(d.at(5, 5), d.at(2, 2));
}

TEST(DistanceField, all_free_no_seeds)
{
  auto m = make_mask(5, 5);  // all free, no blocked cells
  auto d = brushfire_distance_field(m);
  // No blocked seeds → distances remain INT_MAX.
  // Production use always has map boundary walls, so this is an edge case.
  EXPECT_GT(d.at(2, 2), 1000);  // large (INT_MAX-ish)
}

TEST(DistanceField, checkerboard_free_spot)
{
  auto m = make_mask(6, 6);
  // Only center 2x2 is free
  for (int y = 0; y < 6; ++y)
    for (int x = 0; x < 6; ++x)
      m.at(x, y) = (x >= 2 && x <= 3 && y >= 2 && y <= 3) ? 1 : 0;

  auto d = brushfire_distance_field(m);

  // Blocked cells = 0
  EXPECT_EQ(d.at(0, 0), 0);
  EXPECT_EQ(d.at(5, 5), 0);
  // Free cells > 0
  EXPECT_GT(d.at(2, 2), 0);
  EXPECT_GT(d.at(3, 3), 0);
}
