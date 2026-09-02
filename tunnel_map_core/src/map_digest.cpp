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

#include "tunnel_map_core/map_digest.hpp"

#include <iomanip>
#include <sstream>

namespace tunnel_map_core
{

namespace
{

/// Append one byte of canonical text to a stream.
void appendByte(std::ostringstream & os, std::int8_t v)
{
  // Write the raw bit pattern so the digest does not depend on char
  // signedness or locale.
  const auto u = static_cast<unsigned char>(v);
  os << static_cast<char>(u);
}

std::uint64_t fnv1a64(const std::string & bytes)
{
  constexpr std::uint64_t kOffsetBasis = 1469598103934665603ULL;
  constexpr std::uint64_t kPrime = 1099511628211ULL;
  std::uint64_t h = kOffsetBasis;
  for (const unsigned char c : bytes) {
    h ^= static_cast<std::uint64_t>(c);
    h *= kPrime;
  }
  return h;
}

std::string hexOf(std::uint64_t h)
{
  std::ostringstream os;
  os << std::hex << std::setw(16) << std::setfill('0') << h;
  return os.str();
}

}  // namespace

std::string mapContentDigest(const GridMap & map)
{
  std::ostringstream os;
  // Geometry first, then the full data payload, so that a content-only
  // change, a resolution change and an origin change all produce distinct
  // digests.
  os << "v1\n";
  os << map.width << ' ' << map.height << '\n';
  // Serialise doubles with enough precision to be lossless in practice.
  os << std::setprecision(17) << map.resolution << ' ' << map.origin_x << ' '
     << map.origin_y << ' ' << map.origin_yaw << '\n';
  os << "data\n";
  for (const auto v : map.data) {
    appendByte(os, v);
  }
  return hexOf(fnv1a64(os.str()));
}

std::string fnv1a64Hex(const std::string & canonical_text)
{
  return hexOf(fnv1a64(canonical_text));
}

}  // namespace tunnel_map_core
