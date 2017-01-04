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
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
