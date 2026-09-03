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

#include "tunnel_coverage_planner/robot_cleaning_geometry.hpp"

namespace tunnel_coverage_planner
{
namespace
{

TEST(RobotCleaningGeometryTest, DefaultsAreValid)
{
  RobotCleaningGeometry g;
  EXPECT_TRUE(g.valid());
  EXPECT_DOUBLE_EQ(g.toolSweepRadiusM(), 0.25);
  EXPECT_DOUBLE_EQ(g.clearanceRadiusM(), 0.18);
}

TEST(RobotCleaningGeometryTest, RejectsInvalidValues)
{
  RobotCleaningGeometry g;

  g.robot_radius_m = 0.0;
  EXPECT_FALSE(g.valid());
  EXPECT_FALSE(g.validationError().empty());
  g.robot_radius_m = 0.13;

  g.robot_radius_m = -0.1;
  EXPECT_FALSE(g.valid());
  g.robot_radius_m = 0.13;

  g.safety_margin_m = -0.01;
  EXPECT_FALSE(g.valid());
  g.safety_margin_m = 0.05;

  g.cleaning_width_m = 0.0;
  EXPECT_FALSE(g.valid());
  g.cleaning_width_m = 0.5;

  g.min_cleanable_region_area_m2 = -1.0;
  EXPECT_FALSE(g.valid());
  g.min_cleanable_region_area_m2 = 0.04;
}

TEST(RobotCleaningGeometryTest, RejectsNonCentredTool)
{
  RobotCleaningGeometry g;
  g.tool_offset_x_m = 0.1;  // MVP: offset brush is deferred
  EXPECT_FALSE(g.valid());
  EXPECT_NE(g.validationError().find("centred"), std::string::npos);

  g.tool_offset_x_m = 0.0;
  g.tool_offset_y_m = -0.1;
  EXPECT_FALSE(g.valid());
}

}  // namespace
}  // namespace tunnel_coverage_planner
