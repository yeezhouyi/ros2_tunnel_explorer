// Copyright 2026 zhouyi
// License: Apache-2.0

#ifndef TUNNEL_CENTERLINE_EXTRACTOR__SKELETON_EXTRACTOR_HPP_
#define TUNNEL_CENTERLINE_EXTRACTOR__SKELETON_EXTRACTOR_HPP_

#include "tunnel_centerline_extractor/grid_types.hpp"

#include <vector>

namespace tunnel_centerline_extractor
{

/// Zhang-Suen thinning of a binary free-mask.
/// Returns a new mask where 1 = skeleton cell, 0 = background.
///
/// The skeleton is a 1-pixel-wide medial axis of the free space.
/// After thinning, short spurs shorter than `prune_length_cells`
/// are removed to clean the graph.
Grid2D extract_skeleton(const Grid2D & free_mask, int prune_length_cells = 0);

}  // namespace tunnel_centerline_extractor

#endif  // TUNNEL_CENTERLINE_EXTRACTOR__SKELETON_EXTRACTOR_HPP_
