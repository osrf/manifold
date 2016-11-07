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

#include <string>
#include <vector>
#include <ignition/math/Helpers.hh>
#include <ignition/math/SphericalCoordinates.hh>

#include "gtest/gtest.h"
#include "manifold/rndf/Lane.hh"
#include "manifold/rndf/Segment.hh"
#include "manifold/rndf/Waypoint.hh"
#include "manifold/test_config.h"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check id-related accessors.
TEST(SegmentTest, id)
{
  int id = 1;
  Segment segment(id);

  EXPECT_EQ(segment.Id(), id);
  int newId = 2;
  EXPECT_TRUE(segment.SetId(newId));
  EXPECT_EQ(segment.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  int wrongId = -1;
  EXPECT_FALSE(segment.SetId(wrongId));
  EXPECT_EQ(segment.Id(), newId);

  // Check that using the constructor with a wrong id results in a Id = 0.
  Segment wrongSegment(wrongId);
  EXPECT_EQ(wrongSegment.Id(), 0);
}

//////////////////////////////////////////////////
/// \brief Check lanes-related functions.
TEST(LaneTest, waypoints)
{
  int id = 1;
  Segment segment(id);

  EXPECT_EQ(segment.NumLanes(), 0u);
  Lane lane(id);
  // Check an inexistent lane id.
  EXPECT_FALSE(segment.Lane(id, lane));
  // Try to remove an inexistent lane id.
  EXPECT_FALSE(segment.RemoveLane(id));
  // Try to add a lane with an invalid Id.
  EXPECT_FALSE(segment.AddLane(lane));

  // Create a valid waypoint.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  Waypoint wp;
  wp.SetId(waypointId);
  wp.Location() = sc;

  // Create a valid lane.
  EXPECT_TRUE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Add the lane to the segment
  EXPECT_TRUE(segment.AddLane(lane));
  EXPECT_EQ(segment.NumLanes(), 1u);

  // Try to add an existent lane.
  EXPECT_FALSE(segment.AddLane(lane));
  EXPECT_EQ(segment.NumLanes(), 1u);

  // Get the Lane.
  Lane lane2;
  EXPECT_TRUE(segment.Lane(lane.Id(), lane2));
  EXPECT_EQ(lane, lane2);

  // Update a lane.
  double newElevation = 2000;
  lane2.Waypoints().at(0).Location().SetElevationReference(newElevation);
  EXPECT_TRUE(segment.UpdateLane(lane2));
  Lane lane3;
  EXPECT_TRUE(segment.Lane(lane2.Id(), lane3));
  EXPECT_EQ(lane3, lane2);

  // Get a mutable reference to all lanes.
  std::vector<Lane> &lanes = segment.Lanes();
  ASSERT_EQ(lanes.size(), 1u);
  // Modify a lane.
  Lane &aLane = lanes.at(0);
  aLane.Waypoints().at(0).Location().SetElevationReference(500.0);
  EXPECT_TRUE(segment.Lane(lane2.Id(), lane3));
  EXPECT_TRUE(ignition::math::equal(
    lane3.Waypoints().at(0).Location().ElevationReference(), 500.0));

  for (auto const &l : segment.Lanes())
    EXPECT_TRUE(l.Valid());

  // Remove a lane.
  EXPECT_TRUE(segment.RemoveLane(lane2.Id()));
  EXPECT_EQ(segment.NumLanes(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check segment name.
TEST(SegmentTest, Name)
{
  int id = 1;
  Segment segment(id);
  EXPECT_TRUE(segment.Name().empty());

  std::string name = "Wisconsin_Ave";
  segment.SetName(name);
  EXPECT_EQ(segment.Name(), name);
}

//////////////////////////////////////////////////
/// \brief Check segment validation.
TEST(SegmentTest, Validation)
{
  int id = 1;
  Segment segment(id);
  EXPECT_TRUE(segment.Name().empty());
  EXPECT_FALSE(segment.Valid());

  std::string name = "Wisconsin_Ave";
  segment.SetName(name);
  EXPECT_EQ(segment.Name(), name);
  EXPECT_FALSE(segment.Valid());

  // Create a waypoint.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  Waypoint wp;
  wp.SetId(waypointId);
  wp.Location() = sc;

  // Create a valid lane.
  Lane lane(id);
  EXPECT_TRUE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Add the lane to the segment
  EXPECT_TRUE(segment.AddLane(lane));
  EXPECT_EQ(segment.NumLanes(), 1u);

  EXPECT_TRUE(segment.Valid());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
