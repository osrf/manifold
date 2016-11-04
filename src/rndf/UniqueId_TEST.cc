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

#include "gtest/gtest.h"
#include "manifold/rndf/UniqueId.hh"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check accessors.
TEST(UniqueIdTest, accessors)
{
  // Test invalid Ids.
  {
    UniqueId id(-1, 1, 1);
    EXPECT_FALSE(id.Valid());
  }
  {
    UniqueId id(1, -1, 1);
    EXPECT_FALSE(id.Valid());
  }
  {
    UniqueId id(1, 1, 0);
    EXPECT_FALSE(id.Valid());
  }

  // Test valid Ids.
  {
    int segmentId = 1;
    int laneId = 2;
    int waypointId = 3;
    UniqueId id(segmentId, laneId, waypointId);
    EXPECT_TRUE(id.Valid());
    EXPECT_EQ(id.SegmentId(), segmentId);
    EXPECT_EQ(id.LaneId(), laneId);
    EXPECT_EQ(id.WaypointId(), waypointId);

    // Try to modify the segment with an invalid Id.
    EXPECT_FALSE(id.SetSegmentId(-1));
    EXPECT_EQ(id.SegmentId(), segmentId);
    EXPECT_TRUE(id.Valid());

    // Modify segment Id.
    int newSegmentId = 10;
    EXPECT_TRUE(id.SetSegmentId(newSegmentId));
    EXPECT_EQ(id.SegmentId(), newSegmentId);
    EXPECT_TRUE(id.Valid());

    // Try to modify the lane with an invalid Id.
    EXPECT_FALSE(id.SetLaneId(-1));
    EXPECT_EQ(id.LaneId(), laneId);
    EXPECT_TRUE(id.Valid());

    // Modify lane Id.
    int newLaneId = 10;
    EXPECT_TRUE(id.SetLaneId(newLaneId));
    EXPECT_EQ(id.LaneId(), newLaneId);
    EXPECT_TRUE(id.Valid());

    // Try to modify the waypoint with an invalid Id.
    EXPECT_FALSE(id.SetWaypointId(-1));
    EXPECT_EQ(id.WaypointId(), waypointId);
    EXPECT_TRUE(id.Valid());

    // Modify waypoint Id.
    int newWaypointId = 10;
    EXPECT_TRUE(id.SetWaypointId(newWaypointId));
    EXPECT_EQ(id.WaypointId(), newWaypointId);
    EXPECT_TRUE(id.Valid());
  }
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(UniqueIdTest, equality)
{
  int segmentId1 = 1;
  int laneId1 = 2;
  int waypointId1 = 3;
  UniqueId id1(segmentId1, laneId1, waypointId1);

  int segmentId2 = 4;
  int laneId2 = 5;
  int waypointId2 = 6;
  UniqueId id2(segmentId2, laneId2, waypointId2);

  UniqueId id3(segmentId1, laneId2, waypointId2);

  EXPECT_FALSE(id1 == id2);
  EXPECT_TRUE(id1 != id2);

  EXPECT_FALSE(id1 == id3);
  EXPECT_TRUE(id1 != id3);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(UniqueIdTest, assignment)
{
  int segmentId1 = 1;
  int laneId1 = 2;
  int waypointId1 = 3;
  UniqueId id1(segmentId1, laneId1, waypointId1);

  int segmentId2 = 4;
  int laneId2 = 5;
  int waypointId2 = 6;
  UniqueId id2(segmentId2, laneId2, waypointId2);
  EXPECT_NE(id1, id2);

  id2 = id1;
  EXPECT_EQ(id1, id2);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
