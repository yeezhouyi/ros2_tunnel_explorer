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

#include <gtest/gtest.h>

#include <string>

#include "tunnel_map_core/grid_map.hpp"
#include "tunnel_map_core/map_digest.hpp"

namespace tunnel_map_core
{
namespace
{

TEST(MapDigestTest, Fnv1a64KnownVector)
{
  // Well-known FNV-1a 64 test vectors.
  EXPECT_EQ(fnv1a64Hex(""), "cbf29ce484222325");
  EXPECT_EQ(fnv1a64Hex("a"), "af63dc4c8601ec8c");
  EXPECT_EQ(fnv1a64Hex("foobar"), "85944171f73967e8");
}

TEST(MapDigestTest, MapDigestDiffersOnAnyField)
{
  GridMap a;
  a.width = 10;
  a.height = 10;
  a.resolution = 0.05;
  a.origin_x = 0.0;
  a.origin_y = 0.0;
  a.origin_yaw = 0.0;
  a.data.assign(100, OCC_FREE);

  const auto base = mapContentDigest(a);

  GridMap b = a;
  b.data[42] = OCC_OCCUPIED;
  EXPECT_NE(mapContentDigest(b), base);

  GridMap c = a;
  c.data[42] = OCC_UKNOWN;
  EXPECT_NE(mapContentDigest(c), base);

  GridMap d = a;
  d.resolution = 0.1;
  EXPECT_NE(mapContentDigest(d), base);

  GridMap e = a;
  e.origin_x = 0.25;
  EXPECT_NE(mapContentDigest(e), base);

  GridMap f = a;
  f.origin_yaw = 0.01;
  EXPECT_NE(mapContentDigest(f), base);

  // Data changed and changed back -> identical digest.
  GridMap g = a;
  g.data[42] = OCC_OCCUPIED;
  g.data[42] = OCC_FREE;
  EXPECT_EQ(mapContentDigest(g), base);
}

}  // namespace
}  // namespace tunnel_map_core
