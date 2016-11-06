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

#include "manifold/rndf/Checkpoint.hh"
#include "manifold/rndf/ParkingSpot.hh"
#include "manifold/rndf/Waypoint.hh"
#include "manifold/test_config.h"

#include "gtest/gtest.h"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check id-related accessors.
TEST(ParkingSpotTest, id)
{
  int id = 1;
  ParkingSpot spot(id);

  EXPECT_EQ(spot.Id(), id);
  int newId = 2;
  EXPECT_TRUE(spot.SetId(newId));
  EXPECT_EQ(spot.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  int wrongId = -1;
  EXPECT_FALSE(spot.SetId(wrongId));
  EXPECT_EQ(spot.Id(), newId);

  // Check that using the constructor with a wrong id results in a Id = 0.
  ParkingSpot wrongSpot(wrongId);
  EXPECT_EQ(wrongSpot.Id(), 0);
}

//////////////////////////////////////////////////
/// \brief Check waypoints-related functions.
TEST(ParkingSpotTest, waypoints)
{
  int id = 1;
  ParkingSpot spot(id);

  EXPECT_EQ(spot.NumWaypoints(), 0u);
  Waypoint wp;
  // Check an inexistent waypoint Id.
  EXPECT_FALSE(spot.Waypoint(id, wp));
  // Try to remove an inexistent waypoint id.
  EXPECT_FALSE(spot.RemoveWaypoint(id));
  // Try to add a waypoint with an invalid Id.
  EXPECT_FALSE(spot.AddWaypoint(wp));

  // Create a valid waypoint.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  wp.SetId(waypointId);
  wp.Location() = sc;

  // Add a valid waypoint.
  EXPECT_TRUE(spot.AddWaypoint(wp));
  EXPECT_EQ(spot.NumWaypoints(), 1u);

  // Try to add an existent waypoint.
  EXPECT_FALSE(spot.AddWaypoint(wp));
  EXPECT_EQ(spot.NumWaypoints(), 1u);

  // Get the waypoint.
  Waypoint wp2;
  EXPECT_TRUE(spot.Waypoint(wp.Id(), wp2));
  EXPECT_EQ(wp, wp2);

  // Update a waypoint.
  double newElevation = 2000;
  wp2.Location().SetElevationReference(newElevation);
  EXPECT_TRUE(spot.UpdateWaypoint(wp2));
  Waypoint wp3;
  EXPECT_TRUE(spot.Waypoint(wp2.Id(), wp3));
  EXPECT_EQ(wp3, wp2);

  // Get a mutable reference to all waypoints.
  std::vector<Waypoint> &waypoints = spot.Waypoints();
  ASSERT_EQ(waypoints.size(), 1u);
  // Modify a waypoint.
  Waypoint &aWp = waypoints.at(0);
  aWp.Location().SetElevationReference(500.0);
  EXPECT_TRUE(spot.Waypoint(wp2.Id(), wp3));
  EXPECT_TRUE(ignition::math::equal(wp3.Location().ElevationReference(),
    500.0));

  for (auto const &aWaypoint : spot.Waypoints())
    EXPECT_TRUE(aWaypoint.Valid());

  // Remove a waypoint.
  EXPECT_TRUE(spot.RemoveWaypoint(wp2.Id()));
  EXPECT_EQ(spot.NumWaypoints(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check parking spot width.
TEST(ParkingSpotTest, Width)
{
  int id = 1;
  ParkingSpot spot(id);

  // Default parking spot width is 0.
  EXPECT_TRUE(ignition::math::equal(spot.Width(), 0.0));

  // Unable to change the width with an incorrect value.
  EXPECT_FALSE(spot.SetWidth(-1));

  // Change the width.
  double width = 1.0;
  EXPECT_TRUE(spot.SetWidth(width));
  EXPECT_TRUE(ignition::math::equal(spot.Width(), width));
}

//////////////////////////////////////////////////
/// \brief Check checkpoints-related functions.
TEST(ParkingSpotTest, checkpoints)
{
  int id = 1;
  ParkingSpot spot(id);

  // Create a valid checkpoint.
  Checkpoint cp;
  int checkpointId = 2;
  int waypointId = 3;
  cp.SetCheckpointId(checkpointId);
  cp.SetWaypointId(waypointId);
  EXPECT_TRUE(cp.Valid());

  // Set the checkpoint.
  spot.Checkpoint() = cp;

  // Get the checkpoint.
  Checkpoint cp2;
  cp2 = spot.Checkpoint();
  EXPECT_EQ(cp, cp2);
}

//////////////////////////////////////////////////
/// \brief Check function that validates the Id of a parking spot.
TEST(ParkingSpotTest, valid)
{
  std::map<int, bool> cases =
  {
    {-1 , false},
    {0  , false},
    {1  , true},
    {100, true},
  };

  // Check all cases.
  for (auto const &usecase : cases)
  {
    int id = usecase.first;
    ParkingSpot spot(id);

    EXPECT_EQ(spot.Valid(), usecase.second);
  }
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(ParkingSpotTest, equality)
{
  int id1 = 1;
  ParkingSpot ps1(id1);

  int id2 = 2;
  ParkingSpot ps2(id2);

  ParkingSpot ps3(id1);

  EXPECT_FALSE(ps1 == ps2);
  EXPECT_TRUE(ps1 != ps2);

  EXPECT_TRUE(ps1 == ps3);
  EXPECT_FALSE(ps1 != ps3);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(ParkingSpotTest, assignment)
{
  int id1 = 1;
  ParkingSpot ps1(id1);

  int id2 = 2;
  ParkingSpot ps2(id2);
  EXPECT_NE(ps1, ps2);

  ps2 = ps1;
  EXPECT_EQ(ps1, ps2);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
