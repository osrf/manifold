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
  std::cerr << "Show some details of a road network.\n\n"
            << " road_info <RNDF_file>\n\n"
            << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Sanity check.
  if (argc != 2)
  {
    usage();
    return -1;
  }

  // Parse the RNDF file.
  std::string fileName = argv[1];
  manifold::rndf::RNDF rndf(fileName);
  if (!rndf.Valid())
  {
    std::cerr << "File [" << fileName << "] is invalid" << std::endl;
    return -1;
  }

  // Create the roadNetwork object.
  manifold::RoadNetwork roadNetwork(rndf);

  // Show stats.
  auto &graph = roadNetwork.Graph();

  auto vertexes = graph.Vertexes();
  for (auto const &vPtr : vertexes)
  {
    auto adjacents = graph.Adjacents(vPtr);
    std::cout << "Segment/Zone [" << vPtr->Id() << "] is connected with "
              << "segments/zones [ ";
    for (auto const &adjPtr : adjacents)
      std::cout << adjPtr->Id() << " ";
    std::cout << "]" << std::endl;
  }
}
