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

#include "tunnel_frontier_explorer/event_log.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace tunnel_frontier_explorer
{

// ── FNV-1a 64 ────────────────────────────────────────────────────────────

std::uint64_t fnv1a64(const std::string & s)
{
  std::uint64_t h = 0xcbf29ce484222325ULL;
  for (const unsigned char c : s) {
    h ^= static_cast<std::uint64_t>(c);
    h *= 0x100000001b3ULL;
  }
  return h;
}

static std::string hex16(std::uint64_t v)
{
  std::ostringstream os;
  os << std::hex << std::setw(16) << std::setfill('0') << v;
  return os.str();
}

static std::string decimalOf(std::uint64_t v)
{
  std::ostringstream os;
  os << v;
  return os.str();
}

// ── stable cluster id ────────────────────────────────────────────────────

std::string stableClusterId(double rep_x, double rep_y, double quant_m)
{
  if (!(quant_m > 0.0)) {
    return "";
  }
  const std::int64_t qx = static_cast<std::int64_t>(std::floor(rep_x / quant_m));
  const std::int64_t qy = static_cast<std::int64_t>(std::floor(rep_y / quant_m));
  std::ostringstream os;
  os << qx << ':' << qy;              // canonical decimal text, platform-stable
  return decimalOf(fnv1a64(os.str()) & 0xFFFFFFFFFFFFULL);
}

// ── map digest ───────────────────────────────────────────────────────────

std::string mapDigestHex(
  const std::vector<std::int8_t> & data,
  std::size_t width, std::size_t height, std::size_t stride)
{
  if (width == 0 || height == 0 || stride == 0 || data.size() != width * height) {
    return "";
  }
  std::ostringstream os;
  for (std::size_t r = 0; r < height; r += stride) {
    for (std::size_t c = 0; c < width; c += stride) {
      os << static_cast<int>(data[r * width + c]) << ',';
    }
  }
  return hex16(fnv1a64(os.str()));
}

// ── params hash ──────────────────────────────────────────────────────────

std::string paramsHashHex(const std::vector<std::string> & name_value_sorted)
{
  std::string acc;
  for (const auto & nv : name_value_sorted) {
    acc += nv;
    acc.push_back('\n');
  }
  return hex16(fnv1a64(acc));
}

// ── CSV ──────────────────────────────────────────────────────────────────

const std::size_t kEventLogColumns = 21;

static std::string col(const std::string & v) {return v;}
static std::string col(const char * v) {return v ? std::string(v) : std::string();}
static std::string col(double v)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(6) << v;
  return os.str();
}
static std::string col(int v) {return std::to_string(v);}
static std::string col(std::uint64_t v) {return std::to_string(v);}
static std::string col(bool v) {return v ? "1" : "0";}

std::string formatCsvHeader()
{
  return
    "schema_version,repo_sha,repo_dirty,diff_sha,config_hash,map_digest,seed,"
    "t,frontier_count_open,reachable_frontier_count,goal_id,stable_cluster_id,"
    "goal_x,goal_y,action,nav2_result,nav2_error_code,event_detail,"
    "robot_x,robot_y,readiness_status";
}

std::string formatCsv(const EventRow & row)
{
  std::vector<std::string> f;
  f.reserve(kEventLogColumns);
  f.push_back(col(row.schema_version));
  f.push_back(col(row.repo_sha));
  f.push_back(col(row.repo_dirty));
  f.push_back(col(row.diff_sha));
  f.push_back(col(row.config_hash));
  f.push_back(col(row.map_digest));
  f.push_back(col(row.seed));
  f.push_back(col(row.t));
  f.push_back(col(row.frontier_count_open));
  f.push_back(col(row.reachable_frontier_count));
  f.push_back(col(row.goal_id));
  f.push_back(col(row.stable_cluster_id));
  f.push_back(col(row.goal_x));
  f.push_back(col(row.goal_y));
  f.push_back(col(row.action));
  f.push_back(col(row.nav2_result));
  f.push_back(col(row.nav2_error_code));
  f.push_back(col(row.event_detail));
  f.push_back(col(row.robot_x));
  f.push_back(col(row.robot_y));
  f.push_back(col(row.readiness_status));

  std::string out;
  for (std::size_t i = 0; i < f.size(); ++i) {
    if (i > 0) {out.push_back(',');}
    const std::string & v = f[i];
    bool need_quote = v.find_first_of(",\"\n") != std::string::npos;
    if (need_quote) {
      out.push_back('"');
      for (const char ch : v) {
        if (ch == '"') {out.push_back('"');}
        out.push_back(ch);
      }
      out.push_back('"');
    } else {
      out += v;
    }
  }
  return out;
}

// ── EventLogCsv ──────────────────────────────────────────────────────────

EventLogCsv::EventLogCsv(const std::string & path)
: path_(path)
{
  if (path_.empty()) {
    return;
  }
  out_.open(path_, std::ios::out | std::ios::trunc);
}

void EventLogCsv::write(const EventRow & row)
{
  if (!enabled()) {
    return;
  }
  if (!header_written_) {
    out_ << formatCsvHeader() << '\n';
    header_written_ = true;
  }
  out_ << formatCsv(row) << '\n';
  out_.flush();
}

}  // namespace tunnel_frontier_explorer
