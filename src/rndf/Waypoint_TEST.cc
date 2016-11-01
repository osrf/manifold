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
#include <ignition/math/SphericalCoordinates.hh>

#include "manifold/rndf/Waypoint.hh"
#include "manifold/test_config.h"
#include "gtest/gtest.h"

//////////////////////////////////////////////////
/// \brief Check the
TEST(WaypointTest, validator)
{
  std::map<std::string, bool> cases =
  {
    {"1.2.3" ,  true},
    {"1a.2.3", false},
    {"1"     , false}
  };

  for (auto const &usecase : cases)
  {
    // Default surface type
    ignition::math::SphericalCoordinates::SurfaceType st =
      ignition::math::SphericalCoordinates::EARTH_WGS84;
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);

    std::string id = usecase.first;
    manifold::rndf::Waypoint wp(id, sc);
    EXPECT_EQ(wp.ValidWaypoint(id), usecase.second);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
