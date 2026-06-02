// Copyright 2026 zhouyi
// License: Apache-2.0
//
// Stage 4B: tunnel geometry grid sampling utility.
// Wraps /tunnel_centerline/distance_map and /risk_map OccupancyGrid
// messages for use in the frontier scorer.

#ifndef TUNNEL_FRONTIER_EXPLORER__TUNNEL_GEOMETRY_GRID_HPP_
#define TUNNEL_FRONTIER_EXPLORER__TUNNEL_GEOMETRY_GRID_HPP_

#include "tunnel_frontier_explorer/frontier_cluster.hpp"  // Point2D

#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace tunnel_frontier_explorer
{

/// Lightweight grid matching the OccupancyGrid memory layout.
/// Values are 0..100 (occupancy-style), -1 = unknown/out-of-bounds.
struct TunnelGrid
{
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<int8_t> data;

  bool valid() const { return width > 0 && height > 0 && !data.empty(); }

  /// Sample at world coordinates (x,y). Returns nullopt if outside bounds
  /// or if the sampled cell is unknown (< 0).
  std::optional<double> sample(double wx, double wy) const
  {
    int mx = static_cast<int>((wx - origin_x) / resolution);
    int my = static_cast<int>((wy - origin_y) / resolution);
    if (mx < 0 || mx >= width || my < 0 || my >= height) return std::nullopt;
    int8_t v = data[my * width + mx];
    if (v < 0) return std::nullopt;
    return static_cast<double>(v) / 100.0;   // normalise to [0, 1]
  }

  /// Average of sampled values within a circular radius around (wx, wy).
  /// Returns nullopt if no valid cells are found within the radius.
  double averageInRadius(double wx, double wy, double radius_m) const
  {
    int r_cells = static_cast<int>(std::ceil(radius_m / resolution));
    int cx = static_cast<int>((wx - origin_x) / resolution);
    int cy = static_cast<int>((wy - origin_y) / resolution);
    double sum = 0.0;
    int count = 0;
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
      int my = cy + dy;
      if (my < 0 || my >= height) continue;
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        int mx = cx + dx;
        if (mx < 0 || mx >= width) continue;
        double d = std::hypot(dx * resolution, dy * resolution);
        if (d > radius_m) continue;
        int8_t v = data[my * width + mx];
        if (v < 0) continue;
        sum += static_cast<double>(v) / 100.0;
        ++count;
      }
    }
    return count > 0 ? sum / count : 0.0;
  }
};

/// Holds both distance and risk maps from the tunnel centerline extractor.
struct TunnelGeometryGrid
{
  TunnelGrid distance_map;   // 0 = near-wall, 1 = centerline (from OccupancyGrid)
  TunnelGrid risk_map;        // 0 = safe, 1 = high-risk

  bool valid() const { return distance_map.valid() && risk_map.valid(); }

  /// Sample normalised distance at a world point. Higher = farther from wall.
  std::optional<double> sampleCenterlineAlignment(const Point2D & p) const
  {
    if (!valid()) return std::nullopt;
    return distance_map.sample(p.x, p.y);
  }

  /// Sample normalised risk at a world point. Higher = more risky.
  std::optional<double> sampleWallRisk(const Point2D & p) const
  {
    if (!valid()) return std::nullopt;
    return risk_map.sample(p.x, p.y);
  }

  /// Average centerline alignment within a radius.
  double avgCenterlineInRadius(const Point2D & p, double radius_m) const
  {
    if (!valid()) return 0.0;
    return distance_map.averageInRadius(p.x, p.y, radius_m);
  }

  /// Average wall risk within a radius.
  double avgRiskInRadius(const Point2D & p, double radius_m) const
  {
    if (!valid()) return 0.0;
    return risk_map.averageInRadius(p.x, p.y, radius_m);
  }
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__TUNNEL_GEOMETRY_GRID_HPP_
