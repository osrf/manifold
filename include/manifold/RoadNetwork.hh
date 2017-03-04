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

#ifndef MANIFOLD_ROADNETWORK_HH_
#define MANIFOLD_ROADNETWORK_HH_

#include <memory>
#include <string>
#include <ignition/math/Graph.hh>

#include "manifold/Helpers.hh"

namespace manifold
{
  namespace rndf
  {
    class RNDF;
  }

  // Forward declarations.
  class RoadNetworkPrivate;

  /// \brief A class that stores an RNDF object preserving its topological
  /// information. You can use the Graph() method to get access to a graph
  /// where all nodes are the waypoints of the RNDF object.
  class MANIFOLD_VISIBLE RoadNetwork
  {
    /// \brief Constructor.
    public: explicit RoadNetwork(const rndf::RNDF &_rndf);

    /// \brief Destructor.
    public: virtual ~RoadNetwork();

    /// \brief Get a mutable reference to the graph of road segments.
    /// \return A mutable reference to the graph of road segments.
    public: ignition::math::DirectedGraph<std::string, int> &Graph();

    /// \brief Get a reference to the graph of road segments.
    /// \return The reference to the graph of road segments.
    public: const ignition::math::DirectedGraph<std::string, int> &Graph()
      const;

    /// \brief Get the type of road file loaded into the graph.
    /// E.g.: rndf, opendrive
    /// \return The type of road file loaded (e.g.: rndf, opendrive).
    public: std::string RoadType() const;

    /// \internal
    /// \brief Smart pointer to private data.
    private: std::unique_ptr<RoadNetworkPrivate> dataPtr;
  };
}
#endif
