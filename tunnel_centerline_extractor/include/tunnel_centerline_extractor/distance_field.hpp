// Copyright 2026 zhouyi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ... (standard Apache-2.0 header, see LICENSE)

#ifndef TUNNEL_CENTERLINE_EXTRACTOR__DISTANCE_FIELD_HPP_
#define TUNNEL_CENTERLINE_EXTRACTOR__DISTANCE_FIELD_HPP_

#include "tunnel_centerline_extractor/grid_types.hpp"

#include <vector>

namespace tunnel_centerline_extractor
{

/// Compute a distance field from blocked cells using an 8-neighbor
/// brushfire / Dijkstra expansion.  Free cells store distance in
/// integer centimetres (or arbitrary units); blocked cells = 0.
///
/// Horizontal / vertical step cost = resolution.
/// Diagonal step cost = resolution * sqrt(2).
/// All costs are scaled to integer to avoid floating-point drift
/// during the expansion.
Grid2D brushfire_distance_field(const Grid2D & free_mask);

}  // namespace tunnel_centerline_extractor

#endif  // TUNNEL_CENTERLINE_EXTRACTOR__DISTANCE_FIELD_HPP_
