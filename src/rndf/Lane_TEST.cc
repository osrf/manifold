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
#include "manifold/rndf/Checkpoint.hh"
#include "manifold/rndf/Lane.hh"
#include "manifold/rndf/Waypoint.hh"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check id-related accessors.
TEST(LaneTest, id)
{
  int id = 1;
  Lane lane(id);

  EXPECT_EQ(lane.Id(), id);
  int newId = 2;
  EXPECT_TRUE(lane.SetId(newId));
  EXPECT_EQ(lane.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  int wrongId = -1;
  EXPECT_FALSE(lane.SetId(wrongId));
  EXPECT_EQ(lane.Id(), newId);

  // Check that using the constructor with a wrong id results in a Id = 0.
  Lane wrongLane(wrongId);
  EXPECT_EQ(wrongLane.Id(), 0);
}

//////////////////////////////////////////////////
/// \brief Check waypoints-related functions.
TEST(LaneTest, waypoints)
{
  int id = 1;
  Lane lane(id);

  EXPECT_EQ(lane.NumWaypoints(), 0u);
  Waypoint wp;
  // Check an inexistent waypoint Id.
  EXPECT_FALSE(lane.Waypoint(id, wp));
  // Try to remove an inexistent waypoint id.
  EXPECT_FALSE(lane.RemoveWaypoint(id));
  // Try to add a waypoint with an invalid Id.
  EXPECT_FALSE(lane.AddWaypoint(wp));

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
  EXPECT_TRUE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Try to add an existent waypoint.
  EXPECT_FALSE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Get the waypoint.
  Waypoint wp2;
  EXPECT_TRUE(lane.Waypoint(wp.Id(), wp2));
  EXPECT_EQ(wp, wp2);

  // Update a waypoint.
  double newElevation = 2000;
  wp2.Location().SetElevationReference(newElevation);
  EXPECT_TRUE(lane.UpdateWaypoint(wp2));
  Waypoint wp3;
  EXPECT_TRUE(lane.Waypoint(wp2.Id(), wp3));
  EXPECT_EQ(wp3, wp2);

  // Get a mutable reference to all waypoints.
  std::vector<Waypoint> &waypoints = lane.Waypoints();
  ASSERT_EQ(waypoints.size(), 1u);
  // Modify a waypoint.
  Waypoint &aWp = waypoints.at(0);
  aWp.Location().SetElevationReference(500.0);
  EXPECT_TRUE(lane.Waypoint(wp2.Id(), wp3));
  EXPECT_TRUE(ignition::math::equal(wp3.Location().ElevationReference(),
    500.0));

  for (auto const &aWaypoint : lane.Waypoints())
    EXPECT_TRUE(aWaypoint.Valid());
}

//////////////////////////////////////////////////
/// \brief Check function that validates the Id of a lane.
TEST(LaneTest, valid)
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
    // Create a valid waypoint.
    ignition::math::SphericalCoordinates::SurfaceType st =
      ignition::math::SphericalCoordinates::EARTH_WGS84;
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
    int waypointId = 1;
    Waypoint wp(waypointId, sc);

    int laneId = usecase.first;
    Lane lane(laneId);

    // Add a valid waypoint.
    EXPECT_TRUE(lane.AddWaypoint(wp));

    EXPECT_EQ(lane.Valid(), usecase.second);
  }
}

//////////////////////////////////////////////////
/// \brief Check lane width.
TEST(LaneTest, Width)
{
  int id = 1;
  Lane lane(id);

  // Default lane width is 0.
  EXPECT_TRUE(ignition::math::equal(lane.Width(), 0.0));

  // Unable to change the width with an incorrect value.
  EXPECT_FALSE(lane.SetWidth(-1));

  // Change the width.
  double width = 1.0;
  EXPECT_TRUE(lane.SetWidth(width));
  EXPECT_TRUE(ignition::math::equal(lane.Width(), width));
}

//////////////////////////////////////////////////
/// \brief Check lane boundaries.
TEST(LaneTest, Boundaries)
{
  int id = 1;
  Lane lane(id);

  // Default boundary is UNDEFINED.
  EXPECT_EQ(lane.LeftBoundary(), Lane::Marking::UNDEFINED);
  EXPECT_EQ(lane.RightBoundary(), Lane::Marking::UNDEFINED);

  // Change the left boundary.
  Lane::Marking leftBoundary = Lane::Marking::DOUBLE_YELLOW;
  lane.SetLeftBoundary(leftBoundary);
  EXPECT_EQ(lane.LeftBoundary(), leftBoundary);
  EXPECT_EQ(lane.RightBoundary(), Lane::Marking::UNDEFINED);

  // Change the right boundary.
  Lane::Marking rightBoundary = Lane::Marking::SOLID_YELLOW;
  lane.SetRightBoundary(rightBoundary);
  EXPECT_EQ(lane.RightBoundary(), rightBoundary);
  EXPECT_EQ(lane.LeftBoundary(), leftBoundary);
}

//////////////////////////////////////////////////
/// \brief Check checkpoints-related functions.
TEST(LaneTest, checkpoints)
{
  int id = 1;
  Lane lane(id);

  EXPECT_EQ(lane.NumCheckpoints(), 0u);
  Checkpoint cp;
  // Check an inexistent checkpoint Id.
  EXPECT_FALSE(lane.Checkpoint(id, cp));
  // Try to remove an inexistent checkpoint id.
  EXPECT_FALSE(lane.RemoveCheckpoint(id));
  // Try to add a checkpoint with an invalid Id.
  EXPECT_FALSE(lane.AddCheckpoint(cp));

  // Create a valid checkpoint.
  int checkpointId = 2;
  int waypointId = 3;
  cp.SetCheckpointId(checkpointId);
  cp.SetWaypointId(waypointId);
  EXPECT_TRUE(cp.Valid());

  // Add a valid checkpoint.
  EXPECT_TRUE(lane.AddCheckpoint(cp));
  EXPECT_EQ(lane.NumCheckpoints(), 1u);

  // Try to add an existent checkpoint.
  EXPECT_FALSE(lane.AddCheckpoint(cp));
  EXPECT_EQ(lane.NumCheckpoints(), 1u);

  // Get the checkpoint.
  Checkpoint cp2;
  EXPECT_TRUE(lane.Checkpoint(cp.CheckpointId(), cp2));
  EXPECT_EQ(cp, cp2);

  // Update a checkpoint.
  int newWaypointId = 4;
  cp2.SetWaypointId(newWaypointId);
  EXPECT_TRUE(lane.UpdateCheckpoint(cp2));
  Checkpoint cp3;
  EXPECT_TRUE(lane.Checkpoint(cp2.CheckpointId(), cp3));
  EXPECT_EQ(cp3, cp2);

  // Get a mutable reference to all checkpoints.
  std::vector<Checkpoint> &checkpoints = lane.Checkpoints();
  ASSERT_EQ(checkpoints.size(), 1u);
  // Modify a checkpoint.
  Checkpoint &aCp = checkpoints.at(0);
  aCp.SetWaypointId(5);
  EXPECT_TRUE(lane.Checkpoint(cp2.CheckpointId(), cp3));
  EXPECT_EQ(cp3.WaypointId(), 5);

  for (auto const &aCheckpoint : lane.Checkpoints())
    EXPECT_TRUE(aCheckpoint.Valid());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
