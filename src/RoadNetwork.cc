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
#include <ignition/math/Graph.hh>

#include "manifold/rndf/Exit.hh"
#include "manifold/rndf/Perimeter.hh"
#include "manifold/rndf/RNDF.hh"
#include "manifold/rndf/ParkingSpot.hh"
#include "manifold/rndf/Segment.hh"
#include "manifold/rndf/Waypoint.hh"
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
    /// The vertex contains a string (waypoint Id) and the edge an
    /// integer (unused).
    public: ignition::math::DirectedGraph<std::string, int> network;

    /// \brief Type of road file loaded into the graph.
    public: std::string type = "";
  };
}

//////////////////////////////////////////////////
RoadNetwork::RoadNetwork(const rndf::RNDF &_rndf)
  : dataPtr(new RoadNetworkPrivate())
{
  // Populate the graph.
  // Vertexes (all waypoints):
  //   * All waypoints in each segment.
  //   * All perimeter points in each zone.
  //   * All waypoints in each parking spot.
  // Edges:
  //   * Waypoint_i to waypoint_i_+_1 within the same lane and segment.
  //   * Exit waypoint from a segment to entry waypoint of another segment/zone.
  //   * Perimeter point to another perimeter point within the same zone.
  //   * Perimeter point to first waypoint of parking spot within the same zone.
  //   * First waypoint of parking spot to perimeter point within the same zone.
  //   * First waypoint of a parking spot to its second waypoint.
  //   * Second waypoint of a parking spot to its first waypoint.
  //   * Exit waypoint of a zone to an entry waypoint of another segment/zone.

  // Add all waypoints within segments as vertexes.
  for (auto const &segment : _rndf.Segments())
    for (auto const &lane : segment.Lanes())
    {
      std::shared_ptr<ignition::math::Vertex<std::string>> tailPtr = nullptr;
      for (auto const &waypoint : lane.Waypoints())
      {
        rndf::UniqueId id(segment.Id(), lane.Id(), waypoint.Id());
        std::string name = id.String();
        auto headPtr = this->dataPtr->network.AddVertex(id.String(), name);

        if (tailPtr)
        {
          // Connect all waypoints within a segment.
          this->dataPtr->network.AddEdge(tailPtr, headPtr, 0);
        }

        tailPtr = headPtr;
      }
    }

  for (auto const &zone : _rndf.Zones())
  {
    std::vector<std::shared_ptr<ignition::math::Vertex<std::string>>> pointsV;
    // Add all waypoints delimiting the perimeter of zones as vertexes.
    for (auto const &waypoint : zone.Perimeter().Points())
    {
      rndf::UniqueId id(zone.Id(), 0, waypoint.Id());
      std::string name = id.String();
      auto vPtr = this->dataPtr->network.AddVertex(id.String(), name);
      pointsV.push_back(vPtr);
    }

    // Add also the two waypoints of a parking spot as vertexes.
    for (auto const &spot : zone.Spots())
    {
      assert(spot.Waypoints().size() == 2);
      auto wpId = spot.Waypoints().front().Id();
      rndf::UniqueId wpt1(zone.Id(), spot.Id(), wpId);
      std::string name = wpt1.String();
      auto wpt1Ptr = this->dataPtr->network.AddVertex(wpt1.String(), name);
      pointsV.push_back(wpt1Ptr);

      // Add the second waypoint.
      wpId = spot.Waypoints().at(1).Id();
      rndf::UniqueId wpt2(zone.Id(), spot.Id(), wpId);
      name = wpt2.String();
      auto wpt2Ptr = this->dataPtr->network.AddVertex(wpt2.String(), name);

      // You can always go from wpt1->wpt2 and from wpt2->wpt1.
      this->dataPtr->network.AddEdge(wpt1Ptr, wpt2Ptr, 0);
      this->dataPtr->network.AddEdge(wpt2Ptr, wpt1Ptr, 0);
    }

    // From a perimeter point you can go to any other perimeter point or
    // to the first waypoint of a parking spot.
    for (auto const &tailPtr : pointsV)
    {
      for (auto const &headPtr : pointsV)
      {
        if (tailPtr != headPtr)
          this->dataPtr->network.AddEdge(tailPtr, headPtr, 0);
      }
    }

    // Connect all exit waypoints of this zone with other entry waypoints.
    for (auto const &exit : zone.Perimeter().Exits())
    {
      auto nameExitWpt = exit.ExitId().String();
      auto vertexes = this->dataPtr->network.Vertexes(nameExitWpt);
      assert(vertexes.size() == 1);
      auto tailPtr = vertexes.front();

      auto nameEntryWpt = exit.EntryId().String();
      vertexes = this->dataPtr->network.Vertexes(nameEntryWpt);
      assert(vertexes.size() == 1);
      auto headPtr = vertexes.front();

      this->dataPtr->network.AddEdge(tailPtr, headPtr, 0);
    }
  }

  // Connect all waypoints from one segment to another or from one segment to
  // a zone.
  for (auto const &segment : _rndf.Segments())
    for (auto const &lane : segment.Lanes())
      for (auto const &exit : lane.Exits())
      {
        auto nameExitWpt = exit.ExitId().String();
        auto vertexes = this->dataPtr->network.Vertexes(nameExitWpt);
        assert(vertexes.size() == 1);
        auto tailPtr = vertexes.front();

        auto nameEntryWpt = exit.EntryId().String();
        vertexes = this->dataPtr->network.Vertexes(nameEntryWpt);
        assert(vertexes.size() == 1);
        auto headPtr = vertexes.front();

        this->dataPtr->network.AddEdge(tailPtr, headPtr, 0);
      }

  this->dataPtr->type = "rndf";
}

//////////////////////////////////////////////////
RoadNetwork::~RoadNetwork()
{
}

//////////////////////////////////////////////////
ignition::math::DirectedGraph<std::string, int> &RoadNetwork::Graph()
{
  return this->dataPtr->network;
}

//////////////////////////////////////////////////
const ignition::math::DirectedGraph<std::string, int> &RoadNetwork::Graph()
  const
{
  return this->dataPtr->network;
}

//////////////////////////////////////////////////
std::string RoadNetwork::RoadType() const
{
  return this->dataPtr->type;
}
