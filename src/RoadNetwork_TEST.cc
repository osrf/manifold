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

#include <algorithm>
#include <string>

#include "manifold/rndf/RNDF.hh"
#include "manifold/RoadNetwork.hh"
#include "manifold/test_config.h"
#include "gtest/gtest.h"

using namespace manifold;

//////////////////////////////////////////////////
/// \brief Check the RoadNetwork constructor.
TEST(RoadNetwork, Constructor)
{
  std::string dirPath(std::string(PROJECT_SOURCE_PATH));

  rndf::RNDF rndf(dirPath + "/test/rndf/sample1.rndf");
  EXPECT_TRUE(rndf.Valid());

  RoadNetwork roadNetwork(rndf);

  EXPECT_EQ(roadNetwork.RoadType(), "rndf");

  auto &graph = roadNetwork.Graph();

  // We should have 164 vertexes.
  EXPECT_EQ(graph.Vertexes().size(), 164u);
  for (auto i = 0; i < 164; ++i)
    EXPECT_NE(graph.VertexById(i), nullptr);

  // We should have 312 edges.
  EXPECT_EQ(graph.Edges().size(), 318u);

  // All waypoints within the same lane are connected.
  ASSERT_EQ(graph.Vertexes("1.1.1").size(), 1u);
  auto v = graph.Vertexes("1.1.1").front();
  auto neighbors = graph.Adjacents(v);
  ASSERT_EQ(neighbors.size(), 1u);
  EXPECT_EQ(neighbors.front()->Name(), "1.1.2");

  ASSERT_EQ(graph.Vertexes("1.1.2").size(), 1u);
  v = graph.Vertexes("1.1.2").front();
  neighbors = graph.Adjacents(v);
  ASSERT_EQ(neighbors.size(), 1u);
  EXPECT_EQ(neighbors.front()->Name(), "1.1.3");

  ASSERT_EQ(graph.Vertexes("1.1.3").size(), 1u);
  v = graph.Vertexes("1.1.3").front();
  neighbors = graph.Adjacents(v);
  ASSERT_EQ(neighbors.size(), 1u);
  EXPECT_EQ(neighbors.front()->Name(), "1.1.4");

  ASSERT_EQ(graph.Vertexes("1.1.4").size(), 1u);
  v = graph.Vertexes("1.1.4").front();
  neighbors = graph.Adjacents(v);
  ASSERT_TRUE(neighbors.empty());

  // Verify an exit from one segment to another segment.
  ASSERT_EQ(graph.Vertexes("1.2.4").size(), 1u);
  v = graph.Vertexes("1.2.4").front();
  neighbors = graph.Adjacents(v);
  ASSERT_EQ(neighbors.size(), 2u);
  EXPECT_EQ(neighbors.at(0)->Name(), "1.2.5");
  EXPECT_EQ(neighbors.at(1)->Name(), "3.1.1");

  // Verify an exit from one segment to a zone.
  ASSERT_EQ(graph.Vertexes("12.1.2").size(), 1u);
  v = graph.Vertexes("12.1.2").front();
  neighbors = graph.Adjacents(v);
  ASSERT_EQ(neighbors.size(), 1u);
  EXPECT_EQ(neighbors.at(0)->Name(), "14.0.2");

  // Verify a perimeter point.
  ASSERT_EQ(graph.Vertexes("14.0.5").size(), 1u);
  v = graph.Vertexes("14.0.5").front();
  neighbors = graph.Adjacents(v);
  ASSERT_EQ(neighbors.size(), 12u);

  for (auto wp : {"14.0.2", "14.6.1", "14.0.4", "14.2.1", "14.0.6", "14.0.1",
                  "14.0.3", "14.1.1", "14.3.1", "14.4.1", "14.5.1", "11.1.1"})
  {
    auto res = std::find_if(neighbors.begin(), neighbors.end(),
      [&wp](const ignition::math::VertexPtr<std::string> &_vPtr)
      {
        return _vPtr->Name() == wp;
      });
    EXPECT_NE(res, neighbors.end());
  }

  // Verify a parking spot.
  ASSERT_EQ(graph.Vertexes("14.3.1").size(), 1u);
  v = graph.Vertexes("14.3.1").front();
  neighbors = graph.Adjacents(v);
  ASSERT_EQ(neighbors.size(), 12u);

  for (auto wp : {"14.3.2", "14.0.4", "14.0.6", "14.0.3", "14.4.1", "14.0.1",
                  "14.6.1", "14.0.2", "14.0.5", "14.2.1", "14.1.1", "14.5.1"})
  {
    auto res = std::find_if(neighbors.begin(), neighbors.end(),
      [&wp](const ignition::math::VertexPtr<std::string> &_vPtr)
      {
        return _vPtr->Name() == wp;
      });
    EXPECT_NE(res, neighbors.end());
  }

  // Verify that the second waypoint of a parking spot is only linked with its
  // first waypoint.
  ASSERT_EQ(graph.Vertexes("14.3.2").size(), 1u);
  v = graph.Vertexes("14.3.2").front();
  neighbors = graph.Adjacents(v);
  ASSERT_EQ(neighbors.size(), 1u);
  EXPECT_EQ(neighbors.at(0)->Name(), "14.3.1");
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
