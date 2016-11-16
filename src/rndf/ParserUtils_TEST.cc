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
  int width;

  EXPECT_FALSE(parseLaneWidth("xxx 1"           , width));
  EXPECT_FALSE(parseLaneWidth("lane_width"      , width));
  EXPECT_FALSE(parseLaneWidth("lane_width -1"   , width));
  EXPECT_FALSE(parseLaneWidth("lane_width 1 "   , width));
  EXPECT_FALSE(parseLaneWidth(" lane_width 1"   , width));
  EXPECT_FALSE(parseLaneWidth("lane_width 1 2"  , width));
  EXPECT_FALSE(parseLaneWidth("lane_width 32769", width));
  EXPECT_FALSE(parseLaneWidth("lane_width    50", width));
  EXPECT_TRUE(parseLaneWidth("lane_width 0"     , width));
  EXPECT_EQ(width, 0);
  EXPECT_TRUE(parseLaneWidth("lane_width 32768" , width));
  EXPECT_EQ(width, 32768);
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with a lane boundary.
TEST(ParserUtilsTest, laneBoundary)
{
  Lane::Marking b;

  for (const auto &side : {"left", "right"})
  {
    std::string delim = side + std::string("_boundary");
    EXPECT_FALSE(parseBoundary("xxx double_yellow"                        , b));
    EXPECT_FALSE(parseBoundary(delim                                      , b));
    EXPECT_FALSE(parseBoundary(delim       + " xxx"                       , b));
    EXPECT_FALSE(parseBoundary(delim       + " double_yellow "            , b));
    EXPECT_FALSE(parseBoundary(" " + delim + " double_yellow"             , b));
    EXPECT_FALSE(parseBoundary(delim       + " double_yellow solid_yellow", b));
    EXPECT_FALSE(parseBoundary(delim       + "   double_yellow"           , b));
    EXPECT_TRUE(parseBoundary(delim        + " double_yellow"             , b));
    EXPECT_EQ(b, Lane::Marking::DOUBLE_YELLOW);
    EXPECT_TRUE(parseBoundary(delim        + " solid_yellow"              , b));
    EXPECT_EQ(b, Lane::Marking::SOLID_YELLOW);
    EXPECT_TRUE(parseBoundary(delim        + " solid_white"               , b));
    EXPECT_EQ(b, Lane::Marking::SOLID_WHITE);
    EXPECT_TRUE(parseBoundary(delim        + " broken_white"              , b));
    EXPECT_EQ(b, Lane::Marking::BROKEN_WHITE);
  }
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with a checkpoint.
TEST(ParserUtilsTest, checkPoint)
{
  Checkpoint cp;

  EXPECT_FALSE(parseCheckpoint("xxx 1.2.3 1"           , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 1"    , 1, 9, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 1"    , 9, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1"          , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3"      , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.0 1"    , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 0"    , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 -1"   , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 32769", 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.0 1"    , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.32769 1", 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.-1 1"   , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint(" checkpoint 1.2.3 1"   , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint  1.2.3 1"   , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3  1"   , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 1 "   , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 x"    , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint xxx 1"      , 1, 2, cp));
  EXPECT_TRUE(parseCheckpoint("checkpoint 1.2.3 1"     , 1, 2, cp));
  EXPECT_EQ(cp, Checkpoint(1, 3));
  EXPECT_TRUE(parseCheckpoint("checkpoint 1.2.32768 1" , 1, 2, cp));
  EXPECT_EQ(cp, Checkpoint(1, 32768));
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with a stop.
TEST(ParserUtilsTest, stop)
{
  UniqueId stop;

  EXPECT_FALSE(parseStop("xxx 1.2.3"     , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop"          , 1, 2, stop));
  EXPECT_FALSE(parseStop("1.2.3"         , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop xxx"      , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop 1.2.3"    , 1, 9, stop));
  EXPECT_FALSE(parseStop("stop 1.2.3"    , 9, 2, stop));
  EXPECT_FALSE(parseStop("stop 1.2.0"    , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop 1.2.-1"   , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop 1.2.32769", 1, 2, stop));
  EXPECT_FALSE(parseStop(" stop 1.2.3"   , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop  1.2.3"   , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop 1.2.3 "   , 1, 2, stop));
  EXPECT_TRUE(parseStop("stop 1.2.3"     , 1, 2, stop));
  EXPECT_EQ(stop, UniqueId(1, 2, 3));
  EXPECT_TRUE(parseStop("stop 1.2.32768" , 1, 2, stop));
  EXPECT_EQ(stop, UniqueId(1, 2, 32768));
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with an exit.
TEST(ParserUtilsTest, exit)
{
  Exit exit;

  EXPECT_FALSE(parseExit("xxx 1.2.3 2.3.4"     , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit xxx 2.3.4"      , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 xxx"      , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.4"    , 1, 9, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.4"    , 9, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3"          , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 0.3.4"    , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.0.4"    , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.0"    , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 -2.3.4"   , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.-3.4"   , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.-4"   , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 32769.3.4", 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.32769.4", 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.32769", 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.-1 2.3.4"   , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.0 2.3.4"    , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.32769 2.3.4", 1, 2, exit));
  EXPECT_FALSE(parseExit(" exit 1.2.3 2.3.4"   , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit  1.2.3 2.3.4"   , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3  2.3.4"   , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.4 "   , 1, 2, exit));
  EXPECT_TRUE(parseExit("exit 1.2.3 2.3.4"     , 1, 2, exit));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 3), UniqueId(2, 3, 4)));
  EXPECT_TRUE(parseExit("exit 1.2.32768 2.3.4" , 1, 2, exit));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 32768), UniqueId(2, 3, 4)));
  EXPECT_TRUE(parseExit("exit 1.2.3 2.3.32768" , 1, 2, exit));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 3), UniqueId(2, 3, 32768)));
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
