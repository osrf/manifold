/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include "manifold/test_config.h"
#include "manifold/opendrive/Importer.hh"

using namespace manifold;
using namespace opendrive;

//////////////////////////////////////////////////
/// \brief Check loading an OpenDRIVE from an inexistent file.
TEST(OpenDrive, loadInexistentFiles)
{
  {
    Importer importer("__inexistentFile___.xodr");
    //EXPECT_FALSE(importer.Valid());
  }

  {
    Importer importer;
    importer.Load("__inexistentFile___.xodr");
    //EXPECT_FALSE(importer.Valid());
  }
}

//////////////////////////////////////////////////
/// \brief Check loading real OpenDRIVE files.
TEST(OpenDrive, loadSamples)
{
  std::string dirPath(std::string(PROJECT_SOURCE_PATH));
  {
    Importer importer(dirPath + "/test/opendrive/CulDeSac.xodr");
    EXPECT_TRUE(importer.Valid());
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
