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

#include "gtest/gtest.h"
#include "manifold/rndf/Checkpoint.hh"
#include "manifold/rndf/Exit.hh"
#include "manifold/rndf/Lane.hh"
#include "manifold/rndf/ParserUtils.hh"
#include "manifold/rndf/UniqueId.hh"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with a lane width.
TEST(ParserUtilsTest, laneWidth)
{
  int lineN = 0;
  int width;

  EXPECT_FALSE(parseLaneWidth("xxx 1", width, lineN));
  EXPECT_FALSE(parseLaneWidth("lane_width", width, lineN));
  EXPECT_FALSE(parseLaneWidth("lane_width -1", width, lineN));\
  EXPECT_FALSE(parseLaneWidth("lane_width 1 ", width, lineN));
  EXPECT_FALSE(parseLaneWidth(" lane_width 1", width, lineN));
  EXPECT_FALSE(parseLaneWidth("lane_width 1 2", width, lineN));
  EXPECT_FALSE(parseLaneWidth("lane_width 32769", width, lineN));
  EXPECT_FALSE(parseLaneWidth("lane_width    50", width, lineN));
  EXPECT_TRUE(parseLaneWidth("lane_width 0", width, lineN));
  EXPECT_EQ(width, 0);
  EXPECT_TRUE(parseLaneWidth("lane_width 32768", width, lineN));
  EXPECT_EQ(width, 32768);
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with a lane boundary.
TEST(ParserUtilsTest, laneBoundary)
{
  int lineN = 0;
  Lane::Marking boundary;

  for (const auto &side : {"left", "right"})
  {
    std::string delim = side + std::string("_boundary");
    EXPECT_FALSE(parseBoundary("xxx double_yellow", boundary, lineN));
    EXPECT_FALSE(parseBoundary(delim, boundary, lineN));
    EXPECT_FALSE(parseBoundary(delim + " xxx", boundary, lineN));
    EXPECT_FALSE(parseBoundary(delim + " double_yellow ", boundary, lineN));
    EXPECT_FALSE(parseBoundary(" " + delim + " double_yellow", boundary,
      lineN));
    EXPECT_FALSE(parseBoundary(delim + " double_yellow solid_yellow",
      boundary, lineN));
    EXPECT_FALSE(parseBoundary(delim + "   double_yellow", boundary, lineN));
    EXPECT_TRUE(parseBoundary(delim + " double_yellow", boundary, lineN));
    EXPECT_EQ(boundary, Lane::Marking::DOUBLE_YELLOW);
    EXPECT_TRUE(parseBoundary(delim + " solid_yellow", boundary, lineN));
    EXPECT_EQ(boundary, Lane::Marking::SOLID_YELLOW);
    EXPECT_TRUE(parseBoundary(delim + " solid_white", boundary, lineN));
    EXPECT_EQ(boundary, Lane::Marking::SOLID_WHITE);
    EXPECT_TRUE(parseBoundary(delim + " broken_white", boundary, lineN));
    EXPECT_EQ(boundary, Lane::Marking::BROKEN_WHITE);
  }
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with a checkpoint.
TEST(ParserUtilsTest, checkPoint)
{
  int lineN = 0;
  Checkpoint cp;

  EXPECT_FALSE(parseCheckpoint("xxx 1.2.3 1", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 1", 1, 9, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 1", 9, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.0 1", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 0", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 -1", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 32769", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.0 1", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.32769 1", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.-1 1", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint(" checkpoint 1.2.3 1", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint  1.2.3 1", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3  1", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 1 ", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 x", 1, 2, cp, lineN));
  EXPECT_FALSE(parseCheckpoint("checkpoint xxx 1", 1, 2, cp, lineN));
  EXPECT_TRUE(parseCheckpoint("checkpoint 1.2.3 1", 1, 2, cp, lineN));
  EXPECT_EQ(cp, Checkpoint(1, 3));
  EXPECT_TRUE(parseCheckpoint("checkpoint 1.2.32768 1", 1, 2, cp, lineN));
  EXPECT_EQ(cp, Checkpoint(1, 32768));
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with a stop.
TEST(ParserUtilsTest, stop)
{
  int lineN = 0;
  UniqueId stop;

  EXPECT_FALSE(parseStop("xxx 1.2.3", 1, 2, stop, lineN));
  EXPECT_FALSE(parseStop("stop", 1, 2, stop, lineN));
  EXPECT_FALSE(parseStop("1.2.3", 1, 2, stop, lineN));
  EXPECT_FALSE(parseStop("stop xxx", 1, 2, stop, lineN));
  EXPECT_FALSE(parseStop("stop 1.2.3", 1, 9, stop, lineN));
  EXPECT_FALSE(parseStop("stop 1.2.3", 9, 2, stop, lineN));
  EXPECT_FALSE(parseStop("stop 1.2.0", 1, 2, stop, lineN));
  EXPECT_FALSE(parseStop("stop 1.2.-1", 1, 2, stop, lineN));
  EXPECT_FALSE(parseStop("stop 1.2.32769", 1, 2, stop, lineN));
  EXPECT_FALSE(parseStop(" stop 1.2.3", 1, 2, stop, lineN));
  EXPECT_FALSE(parseStop("stop  1.2.3", 1, 2, stop, lineN));
  EXPECT_FALSE(parseStop("stop 1.2.3 ", 1, 2, stop, lineN));
  EXPECT_TRUE(parseStop("stop 1.2.3", 1, 2, stop, lineN));
  EXPECT_EQ(stop, UniqueId(1, 2, 3));
  EXPECT_TRUE(parseStop("stop 1.2.32768", 1, 2, stop, lineN));
  EXPECT_EQ(stop, UniqueId(1, 2, 32768));
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with an exit.
TEST(ParserUtilsTest, exit)
{
  int lineN = 0;
  Exit exit;

  EXPECT_FALSE(parseExit("xxx 1.2.3 2.3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit xxx 2.3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 xxx", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.4", 1, 9, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.4", 9, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 0.3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.0.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.0", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 -2.3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.-3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.-4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 32769.3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.32769.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.32769", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.-1 2.3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.0 2.3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.32769 2.3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit(" exit 1.2.3 2.3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit  1.2.3 2.3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3  2.3.4", 1, 2, exit, lineN));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.4 ", 1, 2, exit, lineN));
  EXPECT_TRUE(parseExit("exit 1.2.3 2.3.4", 1, 2, exit, lineN));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 3), UniqueId(2, 3, 4)));
  EXPECT_TRUE(parseExit("exit 1.2.32768 2.3.4", 1, 2, exit, lineN));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 32768), UniqueId(2, 3, 4)));
  EXPECT_TRUE(parseExit("exit 1.2.3 2.3.32768", 1, 2, exit, lineN));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 3), UniqueId(2, 3, 32768)));
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
