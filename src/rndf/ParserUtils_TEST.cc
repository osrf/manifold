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

#include <climits>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <tuple>
#include <random>
#include <string>

#include "gtest/gtest.h"
#include "manifold/rndf/Checkpoint.hh"
#include "manifold/rndf/Exit.hh"
#include "manifold/rndf/Lane.hh"
#include "manifold/rndf/ParserUtils.hh"
#include "manifold/rndf/UniqueId.hh"

using namespace manifold;
using namespace rndf;

// The fixture for testing class Foo.
class ParserUtilsTest : public ::testing::Test
{
  /// \brief Code here will be called immediately after the constructor
  /// (right before each test).
  virtual void SetUp()
  {
    this->fileName = this->GetRandomNumber() + ".txt";
  }

  /// \brief Code here will be called immediately after each test
  /// (right before the destructor).
  virtual void TearDown()
  {
    if (this->testFile.is_open())
      this->testFile.close();
    std::remove(this->fileName.c_str());
  }

  /// \brief
  protected: void PopulateFile(const std::string &_content)
  {
    if (this->testFile.is_open())
      this->testFile.close();
    this->testFile.open(this->fileName);
    this->testFile << _content << std::endl;
  }

  /// \brief Get a random number based on an integer converted to string.
  /// \return A random integer converted to string.
  private: std::string GetRandomNumber()
  {
    // Initialize random number generator.
    uint32_t seed = std::random_device {}();
    std::mt19937 randGenerator(seed);

    // Create a random number based on an integer converted to string.
    std::uniform_int_distribution<int32_t> d(0, INT_MAX);

    return std::to_string(d(randGenerator));
  }

  /// \brief The name of the test file created.
  protected: std::string fileName;

  /// \brief If file content is not empty, its value will be used to populate
  /// a text file and check some parser functions.
  protected: std::string fileContent = "";

  /// \brief A test file.
  protected: std::ofstream testFile;
};

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with a lane width.
TEST_F(ParserUtilsTest, laneWidth)
{
  int width;

  EXPECT_FALSE(parseLaneWidth("xxx 1"                         , width));
  EXPECT_FALSE(parseLaneWidth("lane_width"                    , width));
  EXPECT_FALSE(parseLaneWidth("lane_width -1"                 , width));
  EXPECT_FALSE(parseLaneWidth(" lane_width 1"                 , width));
  EXPECT_FALSE(parseLaneWidth("lane_width 1 2"                , width));
  EXPECT_FALSE(parseLaneWidth("lane_width 32769"              , width));
  EXPECT_FALSE(parseLaneWidth("lane_width 1 /* a comment */ 2", width));
  EXPECT_FALSE(parseLaneWidth("lane_width 1 /* bad comment"   , width));
  EXPECT_TRUE(parseLaneWidth("lane_width 0"                   , width));
  EXPECT_EQ(width, 0);
  EXPECT_TRUE(parseLaneWidth("lane_width 32768"               , width));
  EXPECT_EQ(width, 32768);
  EXPECT_TRUE(parseLaneWidth("lane_width 1 "                  , width));
  EXPECT_EQ(width, 1);
  EXPECT_TRUE(parseLaneWidth("lane_width    50"               , width));
  EXPECT_EQ(width, 50);
  EXPECT_TRUE(parseLaneWidth("lane_width 1 /* a comment */"   , width));
  EXPECT_EQ(width, 1);
  EXPECT_TRUE(parseLaneWidth("lane_width 1  /* a comment */ " , width));
  EXPECT_EQ(width, 1);
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with a lane boundary.
TEST_F(ParserUtilsTest, laneBoundary)
{
  Lane::Marking b;

  for (const auto &side : {"left", "right"})
  {
    std::string delim = side + std::string("_boundary");
    EXPECT_FALSE(parseBoundary("xxx double_yellow"                        , b));
    EXPECT_FALSE(parseBoundary(delim                                      , b));
    EXPECT_FALSE(parseBoundary(delim       + " xxx"                       , b));
    EXPECT_FALSE(parseBoundary(" " + delim + " double_yellow"             , b));
    EXPECT_FALSE(parseBoundary(delim       + " double_yellow solid_yellow", b));
    EXPECT_FALSE(parseBoundary(delim       + " double_yellow /* bad"      , b));
    EXPECT_TRUE(parseBoundary(delim        + " double_yellow "            , b));
    EXPECT_EQ(b, Lane::Marking::DOUBLE_YELLOW);
    EXPECT_TRUE(parseBoundary(delim        + " double_yellow"             , b));
    EXPECT_EQ(b, Lane::Marking::DOUBLE_YELLOW);
    EXPECT_TRUE(parseBoundary(delim        + "   double_yellow"           , b));
    EXPECT_EQ(b, Lane::Marking::DOUBLE_YELLOW);
    EXPECT_TRUE(parseBoundary(delim        + " solid_yellow"              , b));
    EXPECT_EQ(b, Lane::Marking::SOLID_YELLOW);
    EXPECT_TRUE(parseBoundary(delim        + " solid_white"               , b));
    EXPECT_EQ(b, Lane::Marking::SOLID_WHITE);
    EXPECT_TRUE(parseBoundary(delim        + " broken_white"              , b));
    EXPECT_EQ(b, Lane::Marking::BROKEN_WHITE);
    EXPECT_TRUE(parseBoundary(delim        + " double_yellow /*comment*/" , b));
    EXPECT_EQ(b, Lane::Marking::DOUBLE_YELLOW);
    EXPECT_TRUE(parseBoundary(delim        + " double_yellow  /*     */ " , b));
    EXPECT_EQ(b, Lane::Marking::DOUBLE_YELLOW);
  }
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with a checkpoint.
TEST_F(ParserUtilsTest, checkPoint)
{
  Checkpoint cp;

  EXPECT_FALSE(parseCheckpoint("xxx 1.2.3 1"                       , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 1"                , 1, 9, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 1"                , 9, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1"                      , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3"                  , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.0 1"                , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 0"                , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 -1"               , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 32769"            , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.0 1"                , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.32769 1"            , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.-1 1"               , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint(" checkpoint 1.2.3 1"               , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 x"                , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint xxx 1"                  , 1, 2, cp));
  EXPECT_FALSE(parseCheckpoint("checkpoint 1.2.3 1 /*bad"          , 1, 2, cp));
  EXPECT_TRUE(parseCheckpoint("checkpoint 1.2.3 1"                 , 1, 2, cp));
  EXPECT_EQ(cp, Checkpoint(1, 3));
  EXPECT_TRUE(parseCheckpoint("checkpoint  1.2.3 1"                , 1, 2, cp));
  EXPECT_EQ(cp, Checkpoint(1, 3));
  EXPECT_TRUE(parseCheckpoint("checkpoint 1.2.3  1"                , 1, 2, cp));
  EXPECT_EQ(cp, Checkpoint(1, 3));
  EXPECT_TRUE(parseCheckpoint("checkpoint 1.2.3 1 "                , 1, 2, cp));
  EXPECT_EQ(cp, Checkpoint(1, 3));
  EXPECT_TRUE(parseCheckpoint("checkpoint 1.2.32768 1"             , 1, 2, cp));
  EXPECT_EQ(cp, Checkpoint(1, 32768));
  EXPECT_TRUE(parseCheckpoint("checkpoint 1.2.3 1 /*cmt*/"         , 1, 2, cp));
  EXPECT_EQ(cp, Checkpoint(1, 3));
  EXPECT_TRUE(parseCheckpoint("checkpoint 1.2.3 1  /*cmt*/ "       , 1, 2, cp));
  EXPECT_EQ(cp, Checkpoint(1, 3));
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with a stop.
TEST_F(ParserUtilsTest, stop)
{
  UniqueId stop;

  EXPECT_FALSE(parseStop("xxx 1.2.3"                    , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop"                         , 1, 2, stop));
  EXPECT_FALSE(parseStop("1.2.3"                        , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop xxx"                     , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop 1.2.3"                   , 1, 9, stop));
  EXPECT_FALSE(parseStop("stop 1.2.3"                   , 9, 2, stop));
  EXPECT_FALSE(parseStop("stop 1.2.0"                   , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop 1.2.-1"                  , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop 1.2.32769"               , 1, 2, stop));
  EXPECT_FALSE(parseStop(" stop 1.2.3"                  , 1, 2, stop));
  EXPECT_FALSE(parseStop("stop 1.2.3 /*bad"             , 1, 2, stop));
  EXPECT_TRUE(parseStop("stop 1.2.3"                    , 1, 2, stop));
  EXPECT_EQ(stop, UniqueId(1, 2, 3));
  EXPECT_TRUE(parseStop("stop  1.2.3"                   , 1, 2, stop));
  EXPECT_EQ(stop, UniqueId(1, 2, 3));
  EXPECT_TRUE(parseStop("stop 1.2.3 "                   , 1, 2, stop));
  EXPECT_EQ(stop, UniqueId(1, 2, 3));
  EXPECT_TRUE(parseStop("stop 1.2.32768"                , 1, 2, stop));
  EXPECT_EQ(stop, UniqueId(1, 2, 32768));
  EXPECT_TRUE(parseStop("stop 1.2.3 /* comment  */"     , 1, 2, stop));
  EXPECT_EQ(stop, UniqueId(1, 2, 3));
  EXPECT_TRUE(parseStop("stop 1.2.3   /* comment  */   ", 1, 2, stop));
  EXPECT_EQ(stop, UniqueId(1, 2, 3));
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a line with an exit.
TEST_F(ParserUtilsTest, exit)
{
  Exit exit;

  EXPECT_FALSE(parseExit("xxx 1.2.3 2.3.4"                       , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit xxx 2.3.4"                        , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 xxx"                        , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.4"                      , 1, 9, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.4"                      , 9, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3"                            , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 0.3.4"                      , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.0.4"                      , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.0"                      , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 -2.3.4"                     , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.-3.4"                     , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.-4"                     , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 32769.3.4"                  , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.32769.4"                  , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.32769"                  , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.-1 2.3.4"                     , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.0 2.3.4"                      , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.32769 2.3.4"                  , 1, 2, exit));
  EXPECT_FALSE(parseExit(" exit 1.2.3 2.3.4"                     , 1, 2, exit));
  EXPECT_FALSE(parseExit("exit 1.2.3 2.3.4 /*bad"                , 1, 2, exit));
  EXPECT_TRUE(parseExit("exit 1.2.3 2.3.4"                       , 1, 2, exit));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 3), UniqueId(2, 3, 4)));
  EXPECT_TRUE(parseExit("exit  1.2.3 2.3.4"                      , 1, 2, exit));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 3), UniqueId(2, 3, 4)));
  EXPECT_TRUE(parseExit("exit 1.2.3  2.3.4"                      , 1, 2, exit));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 3), UniqueId(2, 3, 4)));
  EXPECT_TRUE(parseExit("exit 1.2.3 2.3.4"                       , 1, 2, exit));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 3), UniqueId(2, 3, 4)));
  EXPECT_TRUE(parseExit("exit 1.2.3 2.3.32768"                   , 1, 2, exit));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 3), UniqueId(2, 3, 32768)));
  EXPECT_TRUE(parseExit("exit 1.2.32767 2.3.32768"               , 1, 2, exit));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 32767), UniqueId(2, 3, 32768)));
  EXPECT_TRUE(parseExit("exit 1.2.3 2.3.4/*comment*/"            , 1, 2, exit));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 3), UniqueId(2, 3, 4)));
  EXPECT_TRUE(parseExit("exit 1.2.3 2.3.4  /*  comment*/   "     , 1, 2, exit));
  EXPECT_EQ(exit, Exit(UniqueId(1, 2, 3), UniqueId(2, 3, 4)));
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a nonNegative value.
TEST_F(ParserUtilsTest, nonNegative)
{
  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the expected line value.
  // The four element is the expected non-negative value parsed.
  std::vector<std::tuple<std::string, bool, int, int>> testCases =
  {
    std::make_tuple(""                            , false, 1, 0),
    std::make_tuple("\n\n"                        , false, 3, 0),
    std::make_tuple("\n\n2"                       , false, 3, 0),
    std::make_tuple("\n\n delim 2"                , false, 3, 0),
    std::make_tuple("\n\ndelim -1"                , false, 3, 0),
    std::make_tuple("\n\ndelim 32769"             , false, 3, 0),
    std::make_tuple("\n\ndelim 2 /*bad"           , false, 3, 0),
    std::make_tuple("\n\ndelim 0"                 , true , 3, 0),
    std::make_tuple("\n\ndelim  2"                , true, 3, 2),
    std::make_tuple("\n\ndelim 2 "                , true, 3, 2),
    std::make_tuple("\n\ndelim 32768"             , true, 3, 32768),
    std::make_tuple("\n\ndelim 2  /* comment */"  , true, 3, 2),
    std::make_tuple("\n\ndelim 2/*  comment  */  ", true, 3, 2),
  };

  for (auto const &testCase : testCases)
  {
    int line = 0;
    std::string content = std::get<0>(testCase);

    // Expectations.
    bool expectedResult = std::get<1>(testCase);
    int expectedLineNum = std::get<2>(testCase);
    int expectedValue = std::get<3>(testCase);

    // Write the content of this test case into the test file.
    this->PopulateFile(content);
    std::ifstream f(this->fileName);

    // Leave this comment for knowing wich test case failed if needed.
    std::cout << "Testing [" << content << "]" << std::endl;

    // Check expectations.
    bool res;
    int value;
    EXPECT_EQ(res = parseNonNegative(f, "delim", value, line), expectedResult);
    EXPECT_EQ(line, expectedLineNum);
    if (res)
      EXPECT_EQ(value, expectedValue);
  }
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a positive value.
TEST_F(ParserUtilsTest, positive)
{
  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the expected line value.
  // The four element is the expected positive value parsed.
  std::vector<std::tuple<std::string, bool, int, int>> testCases =
  {
    std::make_tuple(""                            , false, 1, 0),
    std::make_tuple("\n\n"                        , false, 3, 0),
    std::make_tuple("\n\n2"                       , false, 3, 0),
    std::make_tuple("\n\n delim 2"                , false, 3, 0),
    std::make_tuple("\n\ndelim -1"                , false, 3, 0),
    std::make_tuple("\n\ndelim 32769"             , false, 3, 0),
    std::make_tuple("\n\ndelim 0"                 , false, 3, 0),
    std::make_tuple("\n\ndelim 2 /* bad"          , false, 3, 0),
    std::make_tuple("\n\ndelim  2"                , true, 3, 2),
    std::make_tuple("\n\ndelim 2 "                , true, 3, 2),
    std::make_tuple("\n\ndelim 32768"             , true , 3, 32768),
    std::make_tuple("\n\ndelim 2 /* comment */"   , true, 3, 2),
    std::make_tuple("\n\ndelim 2/*  comment  */  ", true, 3, 2),
  };

  for (auto const &testCase : testCases)
  {
    int line = 0;
    std::string content = std::get<0>(testCase);

    // Expectations.
    bool expectedResult = std::get<1>(testCase);
    int expectedLineNum = std::get<2>(testCase);
    int expectedValue = std::get<3>(testCase);

    // Write the content of this test case into the test file.
    this->PopulateFile(content);
    std::ifstream f(this->fileName);

    // Leave this comment for knowing wich test case failed if needed.
    std::cout << "Testing [" << content << "]" << std::endl;

    // Check expectations.
    bool res;
    int value;
    EXPECT_EQ(res = parsePositive(f, "delim", value, line), expectedResult);
    EXPECT_EQ(line, expectedLineNum);
    if (res)
      EXPECT_EQ(value, expectedValue);
  }
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a delimiter.
TEST_F(ParserUtilsTest, delimiter)
{
  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the expected line value.
  std::vector<std::tuple<std::string, bool, int>> testCases =
  {
    std::make_tuple(""                              , false, 1),
    std::make_tuple("\n\n"                          , false, 3),
    std::make_tuple("\n\nxxx"                       , false, 3),
    std::make_tuple("\n\n delim"                    , false, 3),
    std::make_tuple("\n\n delim /* bad"             , false, 3),
    std::make_tuple("\n\ndelim "                    , true, 3),
    std::make_tuple("\n\ndelim  "                   , true, 3),
    std::make_tuple("\n\ndelim"                     , true , 3),
    std::make_tuple("\n\ndelim/*comment*/"          , true , 3),
    std::make_tuple("\n\ndelim  /*  comment  */  "  , true , 3),
  };

  for (auto const &testCase : testCases)
  {
    int line = 0;
    std::string content = std::get<0>(testCase);

    // Expectations.
    bool expectedResult = std::get<1>(testCase);
    int expectedLineNum = std::get<2>(testCase);

    // Write the content of this test case into the test file.
    this->PopulateFile(content);
    std::ifstream f(this->fileName);

    // Leave this comment for knowing wich test case failed if needed.
    std::cout << "Testing [" << content << "]" << std::endl;

    // Check expectations.
    EXPECT_EQ(parseDelimiter(f, "delim", line), expectedResult);
    EXPECT_EQ(line, expectedLineNum);
  }
}

//////////////////////////////////////////////////
/// \brief Check the function that parses a string with a delimiter.
TEST_F(ParserUtilsTest, string)
{
  const std::string kIinvalidStr(129, 'x');
  const std::string kLongStr(128, 'a');

  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the expected line value.
  // The four element is the expected value parsed.
  std::vector<std::tuple<std::string, bool, int, std::string>> testCases =
  {
    std::make_tuple(""                                   , false, 1, ""),
    std::make_tuple("\n\n"                               , false, 3, ""),
    std::make_tuple("\n\naValue"                         , false, 3, ""),
    std::make_tuple("\n\n delim aValue"                  , false, 3, ""),
    std::make_tuple("\n\ndelim aValue\\"                 , false, 3, ""),
    std::make_tuple("\n\ndelim aValue*"                  , false, 3, ""),
    std::make_tuple("\n\ndelim aVa lue"                  , false, 3, ""),
    std::make_tuple("\n\ndelim " + kIinvalidStr          , false, 3, ""),
    std::make_tuple("\n\ndelim aValue /*bad"             , false, 3, ""),
    std::make_tuple("\n\ndelim aValue"                   , true , 3, "aValue"),
    std::make_tuple("\n\ndelim  aValue"                  , true , 3, "aValue"),
    std::make_tuple("\n\ndelim aValue  "                 , true , 3, "aValue"),
    std::make_tuple("\n\ndelim 12"                       , true , 3, "12"),
    std::make_tuple("\n\ndelim aValue "                  , true , 3, "aValue"),
    std::make_tuple("\n\ndelim " + kLongStr              , true , 3, kLongStr),
    std::make_tuple("\n\ndelim aValue/*comment */"       , true , 3, "aValue"),
    std::make_tuple("\n\ndelim aValue  /*  comment  */  ", true , 3, "aValue"),
  };

  for (auto const &testCase : testCases)
  {
    int line = 0;
    std::string content = std::get<0>(testCase);

    // Expectations.
    bool expectedResult = std::get<1>(testCase);
    int expectedLineNum = std::get<2>(testCase);
    std::string expectedValue = std::get<3>(testCase);

    // Write the content of this test case into the test file.
    this->PopulateFile(content);
    std::ifstream f(this->fileName);

    // Leave this comment for knowing wich test case failed if needed.
    std::cout << "Testing [" << content << "]" << std::endl;

    // Check expectations.
    bool res;
    std::string value;
    EXPECT_EQ(res = parseString(f, "delim", value, line), expectedResult);
    EXPECT_EQ(line, expectedLineNum);
    if (res)
      EXPECT_EQ(value, expectedValue);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
