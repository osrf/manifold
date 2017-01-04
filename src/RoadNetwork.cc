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

#include <ignition/math/Graph.hh>

#include "manifold/rndf/Exit.hh"
#include "manifold/rndf/Perimeter.hh"
#include "manifold/rndf/RNDF.hh"
#include "manifold/rndf/Segment.hh"
#include "manifold/rndf/Zone.hh"
#include "manifold/RoadNetwork.hh"

using namespace manifold;

namespace manifold
{
  /// \internal
  /// \brief Private data for RoadNetwork class.
  class RoadNetworkPrivate
  {
    /// \brief Constructor.
    public: RoadNetworkPrivate() = default;

    /// \brief Destructor.
    public: virtual ~RoadNetworkPrivate() = default;

    /// \brief Graph of segments.
    /// The vertex contains an integer (segment Id) and the edge another
    /// integer (unused).
    public: ignition::math::Graph<int, int> network;
  };
}

//////////////////////////////////////////////////
RoadNetwork::RoadNetwork(const rndf::RNDF &_rndf)
  : dataPtr(new RoadNetworkPrivate())
{
  // Populate the graph.

  // Add all segments IDs as vertexes.
  for (auto const &segment : _rndf.Segments())
    this->dataPtr->network.AddVertex(segment.Id(), segment.Id());

  // Add all zone IDs as vertexes.
  for (auto const &zone : _rndf.Zones())
    this->dataPtr->network.AddVertex(zone.Id(), zone.Id());

  // Edges from a segment to another segment/zone.
  for (auto const &segment : _rndf.Segments())
    for (auto const &lane : segment.Lanes())
      for (auto const &exit : lane.Exits())
        this->dataPtr->network.AddEdge(segment.Id(), exit.EntryId().X(), 0);

  // Edges from a zone to another segment/zone.
  for (auto const &zone : _rndf.Zones())
    for (auto const &exit : zone.Perimeter().Exits())
      this->dataPtr->network.AddEdge(zone.Id(), exit.EntryId().X(), 0);
}

//////////////////////////////////////////////////
RoadNetwork::~RoadNetwork()
{
}

//////////////////////////////////////////////////
ignition::math::Graph<int, int> &RoadNetwork::Graph()
{
  return this->dataPtr->network;
}

//////////////////////////////////////////////////
const ignition::math::Graph<int, int> &RoadNetwork::Graph() const
{
  return this->dataPtr->network;
}
