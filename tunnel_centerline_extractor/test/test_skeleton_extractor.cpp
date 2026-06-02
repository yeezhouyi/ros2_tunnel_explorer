// Copyright 2026 zhouyi
// License: Apache-2.0

#include "tunnel_centerline_extractor/skeleton_extractor.hpp"

#include <gtest/gtest.h>

using tunnel_centerline_extractor::Grid2D;
using tunnel_centerline_extractor::extract_skeleton;

Grid2D corridor(int w, int h, int cx, int cw)
{
  Grid2D m;
  m.width = w; m.height = h; m.resolution = 0.05;
  m.origin_x = 0; m.origin_y = 0;
  m.data.resize(w * h, 0);
  for (int y = 0; y < h; ++y)
    for (int x = cx; x < cx + cw; ++x)
      m.at(x, y) = 1;
  return m;
}

TEST(SkeletonExtractor, straight_corridor)
{
  auto m = corridor(20, 20, 8, 4);  // 4-wide column at x=8..11
  auto skel = extract_skeleton(m, 0);

  int count = 0;
  for (int y = 0; y < 20; ++y)
    for (int x = 0; x < 20; ++x)
      if (skel.at(x, y)) ++count;

  EXPECT_GT(count, 0);  // skeleton exists
  EXPECT_LT(count, 80); // not the full corridor
}

TEST(SkeletonExtractor, empty_mask)
{
  Grid2D m;
  m.width = 10; m.height = 10; m.resolution = 0.05;
  m.origin_x = 0; m.origin_y = 0;
  m.data.resize(100, 0);
  auto skel = extract_skeleton(m, 0);

  for (int i = 0; i < 100; ++i) EXPECT_EQ(skel.data[i], 0);
}

TEST(SkeletonExtractor, single_cell)
{
  Grid2D m;
  m.width = 3; m.height = 3; m.resolution = 0.05;
  m.origin_x = 0; m.origin_y = 0;
  m.data.resize(9, 0);
  m.at(1, 1) = 1;

  auto skel = extract_skeleton(m, 0);
  // Zhang-Suen preserves isolated pixels: they have 0 neighbors,
  // so none of the removal conditions match (need 2–6 neighbors).
  // This is expected — an isolated free cell is its own skeleton.
  EXPECT_EQ(skel.at(1, 1), 1);
}
