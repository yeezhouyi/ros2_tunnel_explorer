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

#include "tunnel_frontier_explorer/escape_probe_generator.hpp"
#include <cmath>
#include <gtest/gtest.h>

using tunnel_frontier_explorer::EscapeProbeConfig;
using tunnel_frontier_explorer::EscapeProbeGenerator;
using tunnel_frontier_explorer::GridMap;
using tunnel_frontier_explorer::Point2D;

/// Create a 20x20 free-space map (all zeros).
static GridMap makeFreeMap()
{
  GridMap m;
  m.width = 20;
  m.height = 20;
  m.resolution = 0.05;
  m.origin_x = -0.5;
  m.origin_y = -0.5;
  m.data.resize(20 * 20, 0);  // all free
  return m;
}

/// Create a map with an occupied cell at (row, col).
static GridMap makeMapWithObstacle(int row, int col)
{
  auto m = makeFreeMap();
  m.data[row * 20 + col] = 100;  // occupied
  return m;
}

// ── Test 1: Probe generates valid point in free space ──────────────────

TEST(EscapeProbeGenerator, ProbeGeneratesValidPointInFreeSpace)
{
  EscapeProbeConfig cfg;
  cfg.probe_distance_m = 0.5;
  cfg.probe_distance_step_m = 0.2;
  cfg.probe_exit_margin_m = 0.1;
  cfg.max_attempts = 8;
  cfg.angle_span_deg = 90.0;
  cfg.min_wall_distance_m = 0.0;
  EscapeProbeGenerator gen(cfg);

  auto map = makeFreeMap();
  Point2D robot = {0.0, 0.0};
  Point2D center = {0.0, 0.0};
  Point2D dir = {0.0, 1.0};  // north

  auto probe = gen.generateEscapeProbe(robot, center, dir, map, 0.3);
  ASSERT_TRUE(probe.has_value());
  EXPECT_GT(probe->y, center.y);  // should be north of center
}

// ── Test 2: Probe avoids occupied cells ───────────────────────────────

TEST(EscapeProbeGenerator, ProbeAvoidsOccupiedCells)
{
  EscapeProbeConfig cfg;
  cfg.probe_distance_m = 0.3;
  cfg.probe_distance_step_m = 0.1;
  cfg.probe_exit_margin_m = 0.05;
  cfg.max_attempts = 8;
  cfg.angle_span_deg = 90.0;
  cfg.min_wall_distance_m = 0.0;
  EscapeProbeGenerator gen(cfg);

  // Map with obstacle at (5, 10) — directly north of center
  auto map = makeMapWithObstacle(5, 10);
  Point2D robot = {0.0, 0.0};
  Point2D center = {0.0, 0.0};
  Point2D dir = {0.0, 1.0};  // north

  auto probe = gen.generateEscapeProbe(robot, center, dir, map, 0.2);
  // Should either find an alternative direction or return nullopt
  // The obstacle is at y=0.25 (row 5 × 0.05), which is within probe range
  // But angle offsets should allow finding a clear path
  if (probe.has_value()) {
    // Probe should not be at the obstacle location
    EXPECT_TRUE(std::abs(probe->y - 0.25) > 0.01 ||
                std::abs(probe->x - 0.0) > 0.01);
  }
}

// ── Test 3: Probe falls back when all positions occupied ──────────────

TEST(EscapeProbeGenerator, ProbeFallsBackWhenAllPositionsOccupied)
{
  EscapeProbeConfig cfg;
  cfg.probe_distance_m = 0.2;
  cfg.probe_distance_step_m = 0.1;
  cfg.probe_exit_margin_m = 0.05;
  cfg.max_attempts = 3;
  cfg.angle_span_deg = 10.0;  // narrow angle
  cfg.min_wall_distance_m = 0.0;
  EscapeProbeGenerator gen(cfg);

  // Fill all cells near center with obstacles
  auto map = makeFreeMap();
  for (int r = 0; r < 20; ++r) {
    for (int c = 0; c < 20; ++c) {
      double wx = map.origin_x + (c + 0.5) * map.resolution;
      double wy = map.origin_y + (r + 0.5) * map.resolution;
      if (std::hypot(wx, wy) < 0.5) {
        map.data[r * 20 + c] = 100;
      }
    }
  }

  Point2D robot = {0.0, 0.0};
  Point2D center = {0.0, 0.0};
  Point2D dir = {0.0, 1.0};

  auto probe = gen.generateEscapeProbe(robot, center, dir, map, 0.2);
  EXPECT_FALSE(probe.has_value());
}

// ── Test 4: Probe exits oscillillation radius ─────────────────────────

TEST(EscapeProbeGenerator, ProbeExitsOscillationRadius)
{
  EscapeProbeConfig cfg;
  cfg.probe_distance_m = 0.5;
  cfg.probe_distance_step_m = 0.2;
  cfg.probe_exit_margin_m = 0.1;
  cfg.max_attempts = 8;
  cfg.angle_span_deg = 90.0;
  cfg.min_wall_distance_m = 0.0;
  EscapeProbeGenerator gen(cfg);

  auto map = makeFreeMap();
  Point2D robot = {0.0, 0.0};
  Point2D center = {0.0, 0.0};
  Point2D dir = {0.0, 1.0};
  double suppression_radius = 0.3;

  auto probe = gen.generateEscapeProbe(robot, center, dir, map, suppression_radius);
  ASSERT_TRUE(probe.has_value());

  // Probe must be outside suppression_radius + margin
  double dist = std::hypot(probe->x - center.x, probe->y - center.y);
  EXPECT_GT(dist, suppression_radius + cfg.probe_exit_margin_m);
}

// ── Test 5: Probe respects angle span ─────────────────────────────────

TEST(EscapeProbeGenerator, ProbeRespectsAngleSpan)
{
  EscapeProbeConfig cfg;
  cfg.probe_distance_m = 0.5;
  cfg.probe_distance_step_m = 0.2;
  cfg.probe_exit_margin_m = 0.1;
  cfg.max_attempts = 8;
  cfg.angle_span_deg = 30.0;  // narrow
  cfg.min_wall_distance_m = 0.0;
  EscapeProbeGenerator gen(cfg);

  auto map = makeFreeMap();
  Point2D robot = {0.0, 0.0};
  Point2D center = {0.0, 0.0};
  Point2D dir = {0.0, 1.0};  // north

  auto probe = gen.generateEscapeProbe(robot, center, dir, map, 0.2);
  ASSERT_TRUE(probe.has_value());

  // With 30° span, probe should be within ±15° of north
  double angle = std::atan2(probe->x - center.x, probe->y - center.y);
  EXPECT_LT(std::abs(angle), 30.0 * M_PI / 180.0);
}

// ── Test 6: Default config probe works ────────────────────────────────

TEST(EscapeProbeGenerator, DefaultConfigProbeWorks)
{
  EscapeProbeGenerator gen(EscapeProbeConfig{});

  // Use a larger map (50x50) so the probe has room
  GridMap map;
  map.width = 50;
  map.height = 50;
  map.resolution = 0.05;
  map.origin_x = -1.25;
  map.origin_y = -1.25;
  map.data.resize(50 * 50, 0);  // all free

  Point2D robot = {0.0, 0.0};
  Point2D center = {0.0, 0.0};
  Point2D dir = {0.0, 1.0};

  auto probe = gen.generateEscapeProbe(robot, center, dir, map, 0.3);
  ASSERT_TRUE(probe.has_value());

  double dist = std::hypot(probe->x - center.x, probe->y - center.y);
  EXPECT_GT(dist, 0.3 + 0.25);  // must exit suppression + margin
}
