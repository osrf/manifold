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
#include <string>
#include <ignition/math/SphericalCoordinates.hh>

#include "gtest/gtest.h"
#include "manifold/rndf/Lane.hh"
#include "manifold/rndf/Waypoint.hh"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check id-related accessors.
TEST(LaneTest, id)
{
  std::string id = "1.2";
  Lane lane(id);

  EXPECT_EQ(lane.Id(), id);
  std::string newId = "1.3";
  EXPECT_TRUE(lane.SetId(newId));
  EXPECT_EQ(lane.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  std::string wrongId = "1.0";
  EXPECT_FALSE(lane.SetId(wrongId));
  EXPECT_EQ(lane.Id(), newId);

  // Check that using the constructor with a wrong id results in an empty Id.
  Lane wrongLane(wrongId);
  EXPECT_TRUE(wrongLane.Id().empty());
}

//////////////////////////////////////////////////
/// \brief Check waypoints-related functions.
TEST(LaneTest, waypoints)
{
  std::string id = "1.2";
  Lane lane(id);

  EXPECT_EQ(lane.NumWaypoints(), 0u);
  Waypoint wp;
  // Check an incorrect index.
  EXPECT_FALSE(lane.Waypoint(0u, wp));
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
  std::string waypointId = "1.2.3";
  wp.SetId(waypointId);
  wp.Location() = sc;

  // Add a valid waypoint.
  EXPECT_TRUE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Try to add an existent waypoint.
  EXPECT_FALSE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Get the waypoint by index.
  Waypoint wp2;
  EXPECT_TRUE(lane.Waypoint(0u, wp2));
  EXPECT_EQ(wp, wp2);
}

//////////////////////////////////////////////////
/// \brief Check function that validates the Id of a lane.
TEST(LaneTest, valid)
{
  std::map<std::string, bool> cases =
  {
    {""      , false},
    {"1"     , false},
    {"a"     , false},
    {"1."    , false},
    {"1.a"   , false},
    {"1.a."  , false},
    {"1.2."  , false},
    {"1.2 "  , false},
    {"1. 2"  , false},
    {"1a.2"  , false},
    {"1.2a"  , false},
    {"0.2"   , false},
    {"1.-2"  , false},
    {"foo1.2", false},
    {"1.2bar", false},
    {".."    , false},
    {"1.2"   , true},
    {"10.200", true},
  };

  // Check all cases.
  for (auto const &usecase : cases)
  {
    std::string id = usecase.first;
    EXPECT_EQ(Lane::valid(id), usecase.second);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
