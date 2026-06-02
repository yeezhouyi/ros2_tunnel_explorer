// Copyright 2026 zhouyi
// License: Apache-2.0

#ifndef TUNNEL_CENTERLINE_EXTRACTOR__CENTERLINE_GRAPH_HPP_
#define TUNNEL_CENTERLINE_EXTRACTOR__CENTERLINE_GRAPH_HPP_

#include "tunnel_centerline_extractor/grid_types.hpp"

#include <cstdint>
#include <vector>

namespace tunnel_centerline_extractor
{

/// Node in the centerline graph.
struct CenterlineNode
{
  int gx = 0, gy = 0;   // grid coordinates
  double wx = 0, wy = 0; // world coordinates
  int degree = 0;        // 0=isolated, 1=endpoint, 2=corridor, 3+=branch
  double radius_m = 0.0; // distance to nearest wall (from distance field)
};

/// Extracted centerline graph from a skeleton + distance field.
struct CenterlineGraph
{
  std::vector<CenterlineNode> endpoints;
  std::vector<CenterlineNode> branch_points;
  std::vector<CenterlineNode> corridor_nodes;
};

/// Extract centerline graph nodes from skeleton + distance field.
///
/// Skeleton cells (value == 1) are traversed; each cell's 8-neighbor
/// skeleton count determines its degree.  Nodes within merge_radius
/// cells are merged to avoid duplicate branch points.
CenterlineGraph extract_centerline_graph(
  const Grid2D & skeleton,
  const Grid2D & distance_field,
  double resolution,
  int merge_radius_cells = 3);

}  // namespace tunnel_centerline_extractor

#endif  // TUNNEL_CENTERLINE_EXTRACTOR__CENTERLINE_GRAPH_HPP_
