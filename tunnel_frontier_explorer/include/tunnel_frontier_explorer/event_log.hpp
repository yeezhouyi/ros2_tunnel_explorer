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

#ifndef TUNNEL_FRONTIER_EXPLORER__EVENT_LOG_HPP_
#define TUNNEL_FRONTIER_EXPLORER__EVENT_LOG_HPP_

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "tunnel_frontier_explorer/frontier_cluster.hpp"

namespace tunnel_frontier_explorer
{

/// ── B7 (U8) event-log CSV primitives ──────────────────────────────────────
/// Schema v1 in docs/b7_eventlog_schema.md.  All functions are pure
/// (no ROS), so the stable-cluster-id invariant and the digest/CSV
/// formatting are unit-testable without any ROS infrastructure.

/// Canonical 64-bit FNV-1a over a byte string.
std::uint64_t fnv1a64(const std::string & s);

/// Stable geometric cluster id (B7 schema section 2).
///
/// id = fnv1a64("qx:qy") & 0xFFFFFFFFFFFF, where
///   qx = floor(rep_world.x / q), qy = floor(rep_world.y / q).
/// Returned as a decimal string.
///
/// The id is a function of the cluster's GEOMETRY only -- never of its
/// position in the detector output vector (vector index is scan order and
/// shifts when the map grows / components merge-split / min_cluster_size
/// filters one out; index-keyed n=5 pairing would silently pair different
/// frontiers).  q must be > 0; the caller passes the EFFECTIVE step
/// (parameter value when > 0, otherwise the map resolution = injective).
std::string stableClusterId(double rep_world_x, double rep_world_y, double quant_m);

inline std::string stableClusterId(
  const FrontierCluster & c, double quant_m)
{
  return stableClusterId(c.representative_world.x, c.representative_world.y, quant_m);
}

/// Lightweight map digest (B7 schema section 5): FNV-1a over the occupancy
/// bytes sampled with the given stride (rows AND columns), 16 hex chars.
/// Every event row carries the digest of the detection cycle it belongs to.
std::string mapDigestHex(
  const std::vector<std::int8_t> & data,
  std::size_t width, std::size_t height, std::size_t stride);

/// Digest of resolved parameters (B7 schema section 4): FNV-1a over the
/// concatenation of the sorted "name=value" lines, 16 hex chars.
std::string paramsHashHex(const std::vector<std::string> & name_value_sorted);

/// Number of CSV columns (header and every row must agree -- guarded by
/// test_event_log).
extern const std::size_t kEventLogColumns;

/// One event-log row (columns in schema order).  Empty strings for
/// not-applicable fields; nav2_error_code = -1 when no Nav2 error.
struct EventRow
{
  int schema_version = 1;
  std::string repo_sha;
  bool repo_dirty = false;
  std::string diff_sha;             // empty when clean
  std::string config_hash;
  std::string map_digest;           // digest of the detection cycle owning this row
  int seed = 0;
  double t = 0.0;
  int frontier_count_open = 0;
  int reachable_frontier_count = 0;
  int goal_id = -1;                 // -1 = not a goal row
  std::string stable_cluster_id;    // empty when not a frontier-goal row
  double goal_x = 0.0;
  double goal_y = 0.0;
  std::string action;               // heartbeat|accept|reject|probe|timeout|result
  std::string nav2_result;          // accepted|rejected_by_nav2|succeeded|
                                    // aborted|canceled|timed_out|""
  int nav2_error_code = -1;
  std::string event_detail;         // free text; reject rows carry the reason word
  double robot_x = 0.0;
  double robot_y = 0.0;
  std::string readiness_status;     // state-machine enum
};

std::string formatCsvHeader();
std::string formatCsv(const EventRow & row);

/// Append-only CSV sink.  Disabled (no file) when construction path is empty.
class EventLogCsv
{
public:
  explicit EventLogCsv(const std::string & path);
  bool enabled() const {return out_.is_open();}
  void write(const EventRow & row);   // header written on first write
  std::string path() const {return path_;}

private:
  std::string path_;
  std::ofstream out_;
  bool header_written_ = false;
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__EVENT_LOG_HPP_
