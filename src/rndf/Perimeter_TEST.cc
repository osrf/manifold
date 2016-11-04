/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <map>
#include <ignition/math/Helpers.hh>
#include <ignition/math/SphericalCoordinates.hh>

#include "gtest/gtest.h"
#include "manifold/rndf/Exit.hh"
#include "manifold/rndf/Perimeter.hh"
#include "manifold/rndf/UniqueId.hh"
#include "manifold/rndf/Waypoint.hh"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check points-related functions.
TEST(PerimeterTest, points)
{
  Perimeter perimeter;
  EXPECT_FALSE(perimeter.Valid());

  EXPECT_EQ(perimeter.NumPoints(), 0u);
  Waypoint wp;
  // Check an inexistent waypoint Id.
  int id = 1;
  EXPECT_FALSE(perimeter.Point(id, wp));
  // Try to remove an inexistent waypoint id.
  EXPECT_FALSE(perimeter.RemovePoint(id));
  // Try to add a waypoint with an invalid Id.
  EXPECT_FALSE(perimeter.AddPoint(wp));

  // Create a valid waypoint.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  wp.SetId(waypointId);
  wp.Location() = sc;

  // Add a valid point.
  EXPECT_TRUE(perimeter.AddPoint(wp));
  EXPECT_EQ(perimeter.NumPoints(), 1u);
  EXPECT_TRUE(perimeter.Valid());

  // Try to add an existent point.
  EXPECT_FALSE(perimeter.AddPoint(wp));
  EXPECT_EQ(perimeter.NumPoints(), 1u);
  EXPECT_TRUE(perimeter.Valid());

  // Get the point.
  Waypoint wp2;
  EXPECT_TRUE(perimeter.Point(wp.Id(), wp2));
  EXPECT_EQ(wp, wp2);

  // Update a point.
  double newElevation = 2000;
  wp2.Location().SetElevationReference(newElevation);
  EXPECT_TRUE(perimeter.UpdatePoint(wp2));
  Waypoint wp3;
  EXPECT_TRUE(perimeter.Point(wp2.Id(), wp3));
  EXPECT_EQ(wp3, wp2);

  // Get a mutable reference to all points.
  std::vector<Waypoint> &points = perimeter.Points();
  ASSERT_EQ(points.size(), 1u);
  // Modify a point.
  Waypoint &aWp = points.at(0);
  aWp.Location().SetElevationReference(500.0);
  EXPECT_TRUE(perimeter.Point(wp2.Id(), wp3));
  EXPECT_TRUE(ignition::math::equal(wp3.Location().ElevationReference(),
    500.0));

  for (auto const &aPoint : perimeter.Points())
    EXPECT_TRUE(aPoint.Valid());

  // Remove a point.
  EXPECT_TRUE(perimeter.RemovePoint(wp2.Id()));
  EXPECT_EQ(perimeter.NumPoints(), 0u);
  EXPECT_FALSE(perimeter.Valid());
}

//////////////////////////////////////////////////
/// \brief Check exits-related functions.
TEST(PerimeterTest, checkExits)
{
  Perimeter perimeter;

  EXPECT_EQ(perimeter.NumExits(), 0u);
  // Try to remove an inexistent exit.
  EXPECT_FALSE(perimeter.RemoveExit(Exit()));
  // Try to add an invalid exit.
  EXPECT_FALSE(perimeter.AddExit(Exit()));

  // Add a valid exit.
  UniqueId exitId(1, 2, 3);
  UniqueId entryId(4, 5, 6);
  Exit exit1(exitId, entryId);
  EXPECT_TRUE(perimeter.AddExit(exit1));
  EXPECT_EQ(perimeter.NumExits(), 1u);

  // Try to add an existent exit.
  EXPECT_FALSE(perimeter.AddExit(exit1));
  EXPECT_EQ(perimeter.NumExits(), 1u);

  // Get a mutable reference to all exits.
  std::vector<Exit> &exits = perimeter.Exits();
  ASSERT_EQ(exits.size(), 1u);

  for (auto const &exit : perimeter.Exits())
    EXPECT_TRUE(exit.Valid());

  // Remove an exit.
  EXPECT_TRUE(perimeter.RemoveExit(exit1));
  EXPECT_EQ(perimeter.NumExits(), 0u);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
