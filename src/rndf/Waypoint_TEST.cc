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
#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/SphericalCoordinates.hh>

#include "gtest/gtest.h"
#include "manifold/rndf/Waypoint.hh"
#include "manifold/test_config.h"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check id-related accessors.
TEST(WaypointTest, id)
{
  // Default surface type
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);

  std::string id = "1.2.3";
  Waypoint waypoint(id, sc);

  EXPECT_EQ(waypoint.Id(), id);
  std::string newId = "1.2.4";
  EXPECT_TRUE(waypoint.SetId(newId));
  EXPECT_EQ(waypoint.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  std::string wrongId = "1.2.0";
  EXPECT_FALSE(waypoint.SetId(wrongId));
  EXPECT_EQ(waypoint.Id(), newId);

  // Check that using the constructor with a wrong id results in an empty Id.
  Waypoint wrongWp(wrongId, sc);
  EXPECT_TRUE(wrongWp.Id().empty());
}

//////////////////////////////////////////////////
/// \brief Check location-related accessors.
TEST(WaypointTest, location)
{
  // Default surface type
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);

  std::string id = "1.2.3";
  Waypoint waypoint(id, sc);

  // Check that I can read and modify the location with the mutable accessor.
  auto &location = waypoint.Location();
  EXPECT_EQ(location, sc);
  double newElev = 1000.0;
  location.SetElevationReference(newElev);
  EXPECT_TRUE(
    ignition::math::equal(waypoint.Location().ElevationReference(), newElev));
}

//////////////////////////////////////////////////
/// \brief Check function that validates the Id of a waypoint.
TEST(WaypointTest, valid)
{
  std::map<std::string, bool> cases =
  {
    {""           , false},
    {"1"          , false},
    {"a"          , false},
    {"1."         , false},
    {"1.a"        , false},
    {"1.a."       , false},
    {"1.2"        , false},
    {"1.2."       , false},
    {".."         , false},
    {"1.2.a"      , false},
    {"1.2.3."     , false},
    {"1.2. 3"     , false},
    {"1a.2.3"     , false},
    {"1.2a.3"     , false},
    {"1.2.3a"     , false},
    {"0.2.3"      , false},
    {"1.-2.3"     , false},
    {"1.2.3."     , false},
    {"foo1.2.3"   , false},
    {"1.2.3bar"   , false},
    {"1.2.3"      , true},
    {"10.200.3000", true},
  };

  // Check all cases.
  for (auto const &usecase : cases)
  {
    // Default surface type
    ignition::math::SphericalCoordinates::SurfaceType st =
      ignition::math::SphericalCoordinates::EARTH_WGS84;
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);

    std::string id = usecase.first;
    EXPECT_EQ(Waypoint::valid(id), usecase.second);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
