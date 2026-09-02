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

#include "tunnel_coverage_executor/checkpoint_store.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace tunnel_coverage_executor
{

namespace
{

void writeRow(std::ostream & os, const std::string & s)
{
  os << s.size() << ':' << s << '\n';
}

bool readRow(std::istream & is, std::string & out)
{
  std::string len_str;
  if (!std::getline(is, len_str, ':')) {
    return false;
  }
  std::size_t len = 0;
  try {
    len = std::stoull(len_str);
  } catch (const std::exception &) {
    return false;
  }
  if (len > 1u << 28) {  // sanity cap (~256 MB row)
    return false;
  }
  out.resize(len);
  if (len > 0 && !is.read(&out[0], static_cast<std::streamsize>(len))) {
    return false;
  }
  std::string eol;
  return static_cast<bool>(std::getline(is, eol));  // consume '\n'
}

}  // namespace

void CheckpointStore::save(const std::string & path, const CheckpointData & data)
{
  const std::string tmp = path + ".tmp";
  {
    std::ofstream os(tmp, std::ios::binary | std::ios::trunc);
    if (!os.is_open()) {
      throw std::runtime_error("CheckpointStore::save — cannot open " + tmp);
    }
    os << kMagic << '\n';
    os << "v" << data.version << '\n';
    writeRow(os, data.task_input_id);
    writeRow(os, data.plan_id);
    writeRow(os, data.map_digest);
    os << data.resume_segment_index << '\n';
    os << data.path_length_m << '\n';
    os << data.task_duration_s << '\n';
    os << data.segment_ids.size() << '\n';
    for (const auto & id : data.segment_ids) {
      writeRow(os, id);
    }
    os << data.dispositions.size() << '\n';
    for (const auto d : data.dispositions) {
      os << d << ' ';
    }
    os << '\n';
    os << data.visit_counts.size() << '\n';
    for (const auto v : data.visit_counts) {
      os << v << ' ';
    }
    os << '\n';
    if (!os.good()) {
      throw std::runtime_error("CheckpointStore::save — write failed on " + tmp);
    }
  }
  // Atomic replace.
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    throw std::runtime_error("CheckpointStore::save — rename failed for " + path);
  }
}

CheckpointLoadStatus CheckpointStore::loadAndValidate(
  const std::string & path,
  const std::string & expected_task_input_id,
  const std::string & expected_plan_id,
  CheckpointData & data)
{
  std::ifstream is(path, std::ios::binary);
  if (!is.is_open()) {
    return CheckpointLoadStatus::NOT_FOUND;
  }

  std::string magic;
  std::string ver;
  if (!std::getline(is, magic) || magic != kMagic ||
    !std::getline(is, ver) || ver != "v1")
  {
    return CheckpointLoadStatus::CORRUPT;
  }

  CheckpointData d;
  if (!readRow(is, d.task_input_id) || !readRow(is, d.plan_id) ||
    !readRow(is, d.map_digest))
  {
    return CheckpointLoadStatus::CORRUPT;
  }

  std::string line;
  auto readInt = [&](int64_t & v) -> bool {
      if (!std::getline(is, line) || line.empty()) {
        return false;
      }
      try {
        v = std::stoll(line);
      } catch (const std::exception &) {
        return false;
      }
      return true;
    };
  int64_t resume = 0;
  double path_len = 0.0;
  double duration = 0.0;
  if (!readInt(resume) || !std::getline(is, line)) {
    return CheckpointLoadStatus::CORRUPT;
  }
  try {
    path_len = std::stod(line);
    if (!std::getline(is, line)) {
      return CheckpointLoadStatus::CORRUPT;
    }
    duration = std::stod(line);
  } catch (const std::exception &) {
    return CheckpointLoadStatus::CORRUPT;
  }
  d.resume_segment_index = static_cast<int32_t>(resume);
  d.path_length_m = path_len;
  d.task_duration_s = duration;

  // Segment ids.
  int64_t n_seg = 0;
  if (!readInt(n_seg) || n_seg < 0 || n_seg > 1 << 20) {
    return CheckpointLoadStatus::CORRUPT;
  }
  d.segment_ids.reserve(static_cast<std::size_t>(n_seg));
  for (int64_t i = 0; i < n_seg; ++i) {
    std::string id;
    if (!readRow(is, id)) {
      return CheckpointLoadStatus::CORRUPT;
    }
    d.segment_ids.push_back(std::move(id));
  }

  // Dispositions.
  int64_t n_disp = 0;
  if (!readInt(n_disp) || n_disp != n_seg) {
    return CheckpointLoadStatus::CORRUPT;
  }
  if (!std::getline(is, line)) {
    return CheckpointLoadStatus::CORRUPT;
  }
  {
    std::istringstream ss(line);
    int v;
    while (ss >> v) {
      d.dispositions.push_back(v);
    }
  }
  if (static_cast<int64_t>(d.dispositions.size()) != n_disp) {
    return CheckpointLoadStatus::CORRUPT;
  }

  // Visit counts.
  int64_t n_vis = 0;
  if (!readInt(n_vis) || n_vis < 0 || n_vis > 1 << 28) {
    return CheckpointLoadStatus::CORRUPT;
  }
  if (!std::getline(is, line)) {
    return CheckpointLoadStatus::CORRUPT;
  }
  {
    std::istringstream ss(line);
    int v;
    while (ss >> v) {
      d.visit_counts.push_back(v);
    }
  }
  if (static_cast<int64_t>(d.visit_counts.size()) != n_vis) {
    return CheckpointLoadStatus::CORRUPT;
  }

  if (d.task_input_id != expected_task_input_id ||
    d.plan_id != expected_plan_id)
  {
    return CheckpointLoadStatus::MISMATCH;
  }

  data = std::move(d);
  return CheckpointLoadStatus::OK;
}

}  // namespace tunnel_coverage_executor
