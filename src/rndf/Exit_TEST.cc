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
#include "manifold/rndf/Exit.hh"
#include "manifold/rndf/UniqueId.hh"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check accessors.
TEST(ExitTest, accessors)
{
  // Test invalid exits.
  {
    Exit exit;
    EXPECT_FALSE(exit.Valid());
  }
  {
    Exit exit(UniqueId(0, 1, 2), UniqueId(1, 2, 3));
    EXPECT_FALSE(exit.Valid());
  }
  {
    Exit exit(UniqueId(1, 2, 3), UniqueId(0, 1, 2));
    EXPECT_FALSE(exit.Valid());
  }

  // Test valid exit.
  {
    UniqueId exitId(1, 2, 3);
    UniqueId entryId(4, 5, 6);
    Exit exit(exitId, entryId);
    EXPECT_TRUE(exit.Valid());
    EXPECT_EQ(exit.ExitId(), exitId);
    EXPECT_EQ(exit.EntryId(), entryId);
  }
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(ExitTest, equality)
{
  UniqueId exitId1(1, 2, 3);
  UniqueId entryId1(4, 5, 6);
  Exit exit1(exitId1, entryId1);

  UniqueId exitId2(10, 20, 30);
  UniqueId entryId2(40, 50, 60);
  Exit exit2(exitId2, entryId2);

  Exit exit3(exitId1, entryId1);

  EXPECT_FALSE(exit1 == exit2);
  EXPECT_TRUE(exit1 != exit2);

  EXPECT_FALSE(exit1 != exit3);
  EXPECT_TRUE(exit1 == exit3);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(ExitTest, assignment)
{
  UniqueId exitId1(1, 2, 3);
  UniqueId entryId1(4, 5, 6);
  Exit exit1(exitId1, entryId1);

  UniqueId exitId2(10, 20, 30);
  UniqueId entryId2(40, 50, 60);
  Exit exit2(exitId2, entryId2);
  EXPECT_NE(exit1, exit2);

  exit2 = exit1;
  EXPECT_EQ(exit1, exit2);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
