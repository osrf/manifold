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

  int id = 1;
  Waypoint waypoint(id, sc);

  EXPECT_EQ(waypoint.Id(), id);
  int newId = 2;
  EXPECT_TRUE(waypoint.SetId(newId));
  EXPECT_EQ(waypoint.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  int wrongId = -1;
  EXPECT_FALSE(waypoint.SetId(wrongId));
  EXPECT_EQ(waypoint.Id(), newId);

  // Check that using the constructor with a wrong id results in an Id = 0.
  Waypoint wrongWp(wrongId, sc);
  EXPECT_EQ(wrongWp.Id(), 0);
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

  int id = 1;
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
    // Default surface type
    ignition::math::SphericalCoordinates::SurfaceType st =
      ignition::math::SphericalCoordinates::EARTH_WGS84;
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);

    int id = usecase.first;
    Waypoint wp(id, sc);
    EXPECT_EQ(wp.Valid(), usecase.second);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
