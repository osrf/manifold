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
#include "manifold/rndf/Perimeter.hh"
#include "manifold/rndf/RNDF.hh"
#include "manifold/rndf/Segment.hh"
#include "manifold/rndf/Waypoint.hh"
#include "manifold/rndf/Zone.hh"
#include "manifold/test_config.h"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check RNDF name.
TEST(RNDFTest, Name)
{
  RNDF rndf;
  EXPECT_TRUE(rndf.Name().empty());

  std::string name = "test.rndf";
  rndf.SetName(name);
  EXPECT_EQ(rndf.Name(), name);
}

//////////////////////////////////////////////////
/// \brief Check segments-related functions.
TEST(RNDFTest, segments)
{
  RNDF rndf;
  EXPECT_EQ(rndf.NumSegments(), 0u);

  int id = 1;
  Segment segment;
  // Check an inexistent segment id.
  EXPECT_FALSE(rndf.Segment(id, segment));
  // Try to remove an inexistent segment id.
  EXPECT_FALSE(rndf.RemoveSegment(id));
  // Try to add a segment with an invalid Id.
  EXPECT_FALSE(rndf.AddSegment(segment));

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
  Lane lane(id);
  EXPECT_TRUE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Add the lane to the segment.
  segment.SetId(id);
  EXPECT_TRUE(segment.AddLane(lane));
  EXPECT_EQ(segment.NumLanes(), 1u);

  // Add the segment to the RNDF.
  EXPECT_TRUE(rndf.AddSegment(segment));
  EXPECT_EQ(rndf.NumSegments(), 1u);

  // Try to add an existent segment.
  EXPECT_FALSE(rndf.AddSegment(segment));
  EXPECT_EQ(rndf.NumSegments(), 1u);

  // Get the segment.
  Segment segment2;
  EXPECT_TRUE(rndf.Segment(segment.Id(), segment2));
  EXPECT_EQ(segment, segment2);

  // Update a segment.
  segment2.SetName("segment2");
  EXPECT_TRUE(rndf.UpdateSegment(segment2));
  Segment segment3;
  EXPECT_TRUE(rndf.Segment(segment2.Id(), segment3));
  EXPECT_EQ(segment3, segment2);

  // Get a mutable reference to all segments.
  std::vector<Segment> &segments = rndf.Segments();
  ASSERT_EQ(segments.size(), 1u);
  // Modify a segment.
  Segment &aSegment = segments.at(0);
  aSegment.SetName("updated_name");
  EXPECT_TRUE(rndf.Segment(segment2.Id(), segment3));
  EXPECT_EQ(segment3.Name(), "updated_name");

  for (auto const &s : rndf.Segments())
    EXPECT_TRUE(s.Valid());

  // Remove a segment.
  EXPECT_TRUE(rndf.RemoveSegment(segment2.Id()));
  EXPECT_EQ(rndf.NumSegments(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check zones-related functions.
TEST(RNDFTest, zones)
{
  RNDF rndf;
  EXPECT_EQ(rndf.NumZones(), 0u);

  int id = 1;
  Zone zone;
  // Check an inexistent zone id.
  EXPECT_FALSE(rndf.Zone(id, zone));
  // Try to remove an inexistent zone id.
  EXPECT_FALSE(rndf.RemoveZone(id));
  // Try to add a zone with an invalid Id.
  EXPECT_FALSE(rndf.AddZone(zone));

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

  // Add the zone to the RNDF.
  zone.SetId(id);
  EXPECT_TRUE(rndf.AddZone(zone));
  EXPECT_EQ(rndf.NumZones(), 1u);

  // Try to add an existent zone.
  EXPECT_FALSE(rndf.AddZone(zone));
  EXPECT_EQ(rndf.NumZones(), 1u);

  // Get the zone.
  Zone zone2;
  EXPECT_TRUE(rndf.Zone(zone.Id(), zone2));
  EXPECT_EQ(zone, zone2);

  // Update a zone.
  zone2.SetName("segment2");
  EXPECT_TRUE(rndf.UpdateZone(zone2));
  Zone zone3;
  EXPECT_TRUE(rndf.Zone(zone2.Id(), zone3));
  EXPECT_EQ(zone3, zone2);

  // Get a mutable reference to all zones.
  std::vector<Zone> &zones = rndf.Zones();
  ASSERT_EQ(zones.size(), 1u);
  // Modify a zone.
  Zone &aZone = zones.at(0);
  aZone.SetName("updated_name");
  EXPECT_TRUE(rndf.Zone(zone2.Id(), zone3));
  EXPECT_EQ(zone3.Name(), "updated_name");

  for (auto const &z : rndf.Zones())
    EXPECT_TRUE(z.Valid());

  // Remove a zone.
  EXPECT_TRUE(rndf.RemoveZone(zone2.Id()));
  EXPECT_EQ(rndf.NumZones(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check RNDF date.
TEST(RNDFTest, Date)
{
  RNDF rndf;
  EXPECT_TRUE(rndf.Date().empty());

  std::string date = "2016.11.06";
  rndf.SetDate(date);
  EXPECT_EQ(rndf.Date(), date);
}

//////////////////////////////////////////////////
/// \brief Check RNDF version.
TEST(RNDFTest, Version)
{
  RNDF rndf;
  EXPECT_TRUE(rndf.Version().empty());

  std::string version = "2.3.6";
  rndf.SetVersion(version);
  EXPECT_EQ(rndf.Version(), version);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
