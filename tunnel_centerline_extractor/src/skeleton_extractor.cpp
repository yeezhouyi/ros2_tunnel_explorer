// Copyright 2026 zhouyi
// License: Apache-2.0

#include "tunnel_centerline_extractor/skeleton_extractor.hpp"

#include <algorithm>
#include <vector>

namespace tunnel_centerline_extractor
{

namespace
{

/// Count 8-neighbors of (x,y) in mask that are non-zero.
int count_neighbors(const Grid2D & mask, int x, int y)
{
  const int w = mask.width, h = mask.height;
  int n = 0;
  for (int dy = -1; dy <= 1; ++dy)
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      int nx = x + dx, ny = y + dy;
      if (nx >= 0 && nx < w && ny >= 0 && ny < h && mask.at(nx, ny)) ++n;
    }
  return n;
}

/// Count 0→1 transitions in the ordered 8-neighbor ring (P2..P9,P2).
int count_transitions(const Grid2D & mask, int x, int y)
{
  const int w = mask.width, h = mask.height;
  // ordered neighbors: P2(top),P3(top-right),P4(right),...,P9(top-left)
  constexpr int nx[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  constexpr int ny[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
  int vals[8];
  for (int i = 0; i < 8; ++i) {
    int cx = x + nx[i], cy = y + ny[i];
    vals[i] = (cx >= 0 && cx < w && cy >= 0 && cy < h) ? (mask.at(cx, cy) ? 1 : 0) : 0;
  }
  int transitions = 0;
  for (int i = 0; i < 7; ++i)
    if (vals[i] == 0 && vals[i + 1] == 1) ++transitions;
  if (vals[7] == 0 && vals[0] == 1) ++transitions;
  return transitions;
}

/// Single iteration of Zhang-Suen thinning.
/// Returns true if any pixel was removed.
bool zhang_suen_iteration(Grid2D & mask, bool step1)
{
  const int w = mask.width, h = mask.height;
  std::vector<std::pair<int, int>> to_remove;

  for (int y = 1; y < h - 1; ++y) {
    for (int x = 1; x < w - 1; ++x) {
      if (!mask.at(x, y)) continue;

      int n = count_neighbors(mask, x, y);
      if (n < 2 || n > 6) continue;

      int t = count_transitions(mask, x, y);
      if (t != 1) continue;

      // P2 * P4 * P6 (step 1)  or  P2 * P4 * P8 (step 2)
      int p2 = mask.at(x, y - 1);
      int p4 = mask.at(x + 1, y);
      int p6 = mask.at(x, y + 1);
      int p8 = mask.at(x - 1, y);

      if (step1) {
        if (p2 * p4 * p6 != 0) continue;
        // P4 * P6 * P8
        if (p4 * p6 * p8 != 0) continue;
      } else {
        // P2 * P4 * P8
        if (p2 * p4 * p8 != 0) continue;
        // P2 * P6 * P8
        if (p2 * p6 * p8 != 0) continue;
      }

      to_remove.push_back({x, y});
    }
  }

  for (auto [x, y] : to_remove)
    mask.at(x, y) = 0;

  return !to_remove.empty();
}

/// Zhang-Suen thinning: iterate step 1 + step 2 until convergence.
void zhang_suen_thin(Grid2D & mask)
{
  bool changed = true;
  while (changed) {
    changed = false;
    changed |= zhang_suen_iteration(mask, true);
    changed |= zhang_suen_iteration(mask, false);
  }
}

/// Remove short spurs (branches shorter than prune_len cells).
/// A spur is a skeleton chain ending at a degree-1 node.
void prune_spurs(Grid2D & skeleton, int prune_len)
{
  if (prune_len <= 0) return;
  const int w = skeleton.width, h = skeleton.height;

  for (int pass = 0; pass < prune_len; ++pass) {
    std::vector<std::pair<int, int>> to_remove;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (!skeleton.at(x, y)) continue;
        if (count_neighbors(skeleton, x, y) <= 1)
          to_remove.push_back({x, y});
      }
    }
    if (to_remove.empty()) break;
    for (auto [x, y] : to_remove)
      skeleton.at(x, y) = 0;
  }
}

}  // namespace

Grid2D extract_skeleton(const Grid2D & free_mask, int prune_length_cells)
{
  Grid2D skeleton = free_mask;   // copy — thinning operates in-place
  zhang_suen_thin(skeleton);
  prune_spurs(skeleton, prune_length_cells);
  return skeleton;
}

}  // namespace tunnel_centerline_extractor
