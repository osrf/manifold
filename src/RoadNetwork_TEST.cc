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

  auto &graph = roadNetwork.Graph();

  // We should have 14 vertexes with IDs ranging from 1 to 14.
  EXPECT_EQ(graph.Vertexes().size(), 14u);
  for (auto i = 1; i < 14; ++i)
    EXPECT_NE(graph.VertexById(i), nullptr);

  // We should have 30 edges.
  EXPECT_EQ(graph.Edges().size(), 30u);

  // From the segment #1 is possible to reach other two segments.
  auto adjacents = graph.Adjacents(1);
  EXPECT_EQ(adjacents.size(), 2u);

  // From the segment #14 is possible to reach segment #11.
  adjacents = graph.Adjacents(14);
  ASSERT_EQ(adjacents.size(), 1u);
  EXPECT_EQ(adjacents.at(0)->Id(), 11);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
