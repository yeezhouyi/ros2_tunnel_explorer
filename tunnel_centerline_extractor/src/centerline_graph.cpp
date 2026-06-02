// Copyright 2026 zhouyi
// License: Apache-2.0

#include "tunnel_centerline_extractor/centerline_graph.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

namespace tunnel_centerline_extractor
{

namespace
{

/// 8-neighbor offsets
constexpr int dx8[8] = {0, 0, -1, 1, -1, 1, -1, 1};
constexpr int dy8[8] = {-1, 1, 0, 0, -1, -1, 1, 1};

int degree(const Grid2D & skel, int x, int y)
{
  int d = 0;
  const int w = skel.width, h = skel.height;
  for (int i = 0; i < 8; ++i) {
    int nx = x + dx8[i], ny = y + dy8[i];
    if (nx >= 0 && nx < w && ny >= 0 && ny < h && skel.at(nx, ny)) ++d;
  }
  return d;
}

}  // namespace

CenterlineGraph extract_centerline_graph(
  const Grid2D & skeleton,
  const Grid2D & distance_field,
  double resolution,
  int merge_radius_cells)
{
  CenterlineGraph graph;
  const int w = skeleton.width, h = skeleton.height;

  // Collect raw nodes from skeleton cells
  std::vector<CenterlineNode> raw;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (!skeleton.at(x, y)) continue;
      CenterlineNode node;
      node.gx = x;
      node.gy = y;
      node.wx = skeleton.origin_x + (x + 0.5) * resolution;
      node.wy = skeleton.origin_y + (y + 0.5) * resolution;
      node.degree = degree(skeleton, x, y);
      // Distance field is in integer units (resolution * 1000)
      node.radius_m = distance_field.at(x, y) / 1000.0;
      raw.push_back(node);
    }
  }

  // Merge nearby branch / endpoint nodes
  auto merge_nodes = [&](std::vector<CenterlineNode> & nodes) {
    std::vector<bool> merged(nodes.size(), false);
    std::vector<CenterlineNode> result;
    for (size_t i = 0; i < nodes.size(); ++i) {
      if (merged[i]) continue;
      // Find group within merge radius
      double sx = nodes[i].gx, sy = nodes[i].gy;
      int count = 1;
      double sum_r = nodes[i].radius_m;
      for (size_t j = i + 1; j < nodes.size(); ++j) {
        if (merged[j]) continue;
        double d = std::hypot(nodes[i].gx - nodes[j].gx, nodes[i].gy - nodes[j].gy);
        if (d <= merge_radius_cells) {
          merged[j] = true;
          sx += nodes[j].gx; sy += nodes[j].gy;
          sum_r += nodes[j].radius_m;
          ++count;
        }
      }
      CenterlineNode merged_node = nodes[i];
      merged_node.gx = static_cast<int>(sx / count);
      merged_node.gy = static_cast<int>(sy / count);
      merged_node.wx = skeleton.origin_x + (merged_node.gx + 0.5) * resolution;
      merged_node.wy = skeleton.origin_y + (merged_node.gy + 0.5) * resolution;
      merged_node.radius_m = sum_r / count;
      result.push_back(merged_node);
    }
    nodes = std::move(result);
  };

  std::vector<CenterlineNode> endpoints, branches;
  for (auto & node : raw) {
    if (node.degree == 1) endpoints.push_back(node);
    else if (node.degree >= 3) branches.push_back(node);
    else if (node.degree == 2) graph.corridor_nodes.push_back(node);
  }

  merge_nodes(endpoints);
  merge_nodes(branches);

  graph.endpoints = std::move(endpoints);
  graph.branch_points = std::move(branches);

  return graph;
}

}  // namespace tunnel_centerline_extractor
