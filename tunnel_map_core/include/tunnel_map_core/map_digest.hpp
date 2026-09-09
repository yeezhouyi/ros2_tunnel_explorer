// Copyright 2026 zhouyi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TUNNEL_MAP_CORE__MAP_DIGEST_HPP_
#define TUNNEL_MAP_CORE__MAP_DIGEST_HPP_

#include <cstdint>
#include <string>

#include "tunnel_map_core/grid_map.hpp"

namespace tunnel_map_core
{

/// 64-bit FNV-1a hash over a canonical byte serialisation of the map's
/// geometry and content.  Deterministic across runs and platforms.
///
/// @param map A well-formed grid map (valid() is not checked here).
/// @return Lower-case hex string of the hash.
std::string mapContentDigest(const GridMap & map);

/// 64-bit FNV-1a hash over an arbitrary canonical string (used for plan
/// digests, parameter digests, etc.).
std::string fnv1a64Hex(const std::string & canonical_text);

}  // namespace tunnel_map_core

#endif  // TUNNEL_MAP_CORE__MAP_DIGEST_HPP_
