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

#include <iostream>
#include <string>
#include <ignition/math/Graph.hh>
#include <manifold/manifold.hh>

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Show some details of a waypoint.\n\n"
            << " wpt_info <RNDF_file> <waypoint Id>\n\n"
            << " E.g.: ./wpt_info sample1.rndf 14.3.1\n\n"
            << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Sanity check.
  if (argc != 3)
  {
    usage();
    return -1;
  }

  // Parse the RNDF file.
  std::string fileName = argv[1];
  std::string wptName = argv[2];
  manifold::rndf::RNDF rndf(fileName);
  if (!rndf.Valid())
  {
    std::cerr << "File [" << fileName << "] is invalid" << std::endl;
    return -1;
  }

  // Create the roadNetwork object.
  manifold::RoadNetwork roadNetwork(rndf);

  // Show stats of a given waypoint.
  auto &graph = roadNetwork.Graph();
  auto vertexes = graph.Vertexes(wptName);
  if (vertexes.empty())
  {
    std::cout << "Waypoint [" << wptName << "] not found" << std::endl;
    return 0;
  }

  // Get the pointer to the random vertex containing a waypoint.
  auto wptPtr = vertexes.front();
  auto neighbors = graph.Adjacents(wptPtr);

  std::cout << "Waypoint [" << wptPtr->Name() << "]" << std::endl;

  manifold::rndf::UniqueId id(wptPtr->Name());
  auto info = rndf.Info(manifold::rndf::UniqueId(wptPtr->Name()));
  if (!info)
  {
    std::cerr << "Additional information not found" << std::endl;
    return -1;
  }
  if (info->Segment())
  {
    std::cout << "\tInfo: This waypoint is contained in segment ["
              << info->Segment()->Id() << "] and lane [" << info->Lane()->Id()
              << "]" << std::endl;
  }
  else if (info->Zone())
  {
    std::cout << "\tInfo: This waypoint is contained in zone ["
              << info->Zone()->Id() << "]" << std::endl;
  }

  std::cout << "\tNeighbors:" << std::endl;
  for (const auto &neighborPtr : neighbors)
    std::cout << "\t\t" << neighborPtr->Name() << std::endl;

  return 0;
}
