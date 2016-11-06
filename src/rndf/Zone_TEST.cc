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
#include "manifold/rndf/ParkingSpot.hh"
#include "manifold/rndf/Perimeter.hh"
#include "manifold/rndf/Waypoint.hh"
#include "manifold/rndf/Zone.hh"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check id-related accessors.
TEST(ZoneTest, id)
{
  int id = 1;
  Zone zone(id);

  EXPECT_EQ(zone.Id(), id);
  int newId = 2;
  EXPECT_TRUE(zone.SetId(newId));
  EXPECT_EQ(zone.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  int wrongId = -1;
  EXPECT_FALSE(zone.SetId(wrongId));
  EXPECT_EQ(zone.Id(), newId);

  // Check that using the constructor with a wrong id results in a Id = 0.
  Zone wrongZone(wrongId);
  EXPECT_EQ(wrongZone.Id(), 0);
}

//////////////////////////////////////////////////
/// \brief Check parking spots-related functions.
TEST(ZoneTest, spots)
{
  int id = 1;
  Zone zone(id);

  EXPECT_EQ(zone.NumSpots(), 0u);
  ParkingSpot ps;
  // Check an inexistent parking spot Id.
  EXPECT_FALSE(zone.Spot(id, ps));
  // Try to remove an inexistent spot id.
  EXPECT_FALSE(zone.RemoveSpot(id));
  // Try to add a parking spot with an invalid Id.
  EXPECT_FALSE(zone.AddSpot(ps));

  // Create a valid parking spot.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  Waypoint wp;
  wp.SetId(waypointId);
  wp.Location() = sc;
  ps.SetId(id);
  ps.AddWaypoint(wp);
  EXPECT_EQ(ps.NumWaypoints(), 1u);

  // Add a valid parking spot.
  EXPECT_TRUE(zone.AddSpot(ps));
  EXPECT_EQ(zone.NumSpots(), 1u);

  // Try to add an existent parking spot.
  EXPECT_FALSE(zone.AddSpot(ps));
  EXPECT_EQ(zone.NumSpots(), 1u);

  // Get the parking spot.
  ParkingSpot ps2;
  EXPECT_TRUE(zone.Spot(ps.Id(), ps2));
  EXPECT_EQ(ps, ps2);
  EXPECT_EQ(ps2.NumWaypoints(), 1u);

  // Update a parking spot.
  double newElevation = 2000;
  ASSERT_GT(ps2.NumWaypoints(), 0u);
  ps2.Waypoints().at(0).Location().SetElevationReference(newElevation);
  EXPECT_TRUE(zone.UpdateSpot(ps2));
  ParkingSpot ps3;
  EXPECT_TRUE(zone.Spot(ps2.Id(), ps3));
  EXPECT_EQ(ps3, ps2);

  // Get a mutable reference to all parking spots.
  std::vector<ParkingSpot> &spots = zone.Spots();
  ASSERT_EQ(spots.size(), 1u);
  // Modify a parking spot.
  ParkingSpot &aPs = spots.at(0);
  aPs.Waypoints().at(0).Location().SetElevationReference(500.0);
  EXPECT_TRUE(zone.Spot(ps2.Id(), ps3));
  EXPECT_TRUE(ignition::math::equal(
    ps3.Waypoints().at(0).Location().ElevationReference(), 500.0));

  for (auto const &aSpot : zone.Spots())
    EXPECT_TRUE(aSpot.Valid());

  // Remove a parking spot.
  EXPECT_TRUE(zone.RemoveSpot(ps2.Id()));
  EXPECT_EQ(zone.NumSpots(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check perimeter-related functions.
TEST(ZoneTest, perimeter)
{
  int id = 1;
  Zone zone(id);

  // Add a perimeter point.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  Waypoint wp;
  wp.SetId(waypointId);
  wp.Location() = sc;

  EXPECT_TRUE(zone.Perimeter().AddPoint(wp));
  EXPECT_EQ(zone.Perimeter().Points().size(), 1u);
}

//////////////////////////////////////////////////
/// \brief Check zone name.
TEST(ZoneTest, Name)
{
  int id = 1;
  Zone zone(id);
  EXPECT_TRUE(zone.Name().empty());

  std::string name = "North_parking_lot";
  zone.SetName(name);
  EXPECT_EQ(zone.Name(), name);
}

//////////////////////////////////////////////////
/// \brief Check zone validation.
TEST(ZoneTest, Validation)
{
  int id = 1;
  Zone zone(id);
  EXPECT_TRUE(zone.Name().empty());
  EXPECT_FALSE(zone.Valid());

  std::string name = "North_parking_lot";
  zone.SetName(name);
  EXPECT_EQ(zone.Name(), name);
  EXPECT_FALSE(zone.Valid());

  // Add a perimeter point.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  Waypoint wp;
  wp.SetId(waypointId);
  wp.Location() = sc;

  EXPECT_TRUE(zone.Perimeter().AddPoint(wp));
  EXPECT_TRUE(zone.Valid());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
