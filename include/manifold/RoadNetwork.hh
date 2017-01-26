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

  /// \brief ToDo.
  class MANIFOLD_VISIBLE RoadNetwork
  {
    /// \brief Constructor.
    public: RoadNetwork(const rndf::RNDF &_rndf);

    /// \brief Destructor.
    public: virtual ~RoadNetwork();

    /// \brief Get a mutable reference to the graph of road segments.
    /// \return A mutable reference to the graph of road segments.
    public: ignition::math::Graph<std::string, int> &Graph();

    /// \brief Get a reference to the graph of road segments.
    /// \return The reference to the graph of road segments.
    public: const ignition::math::Graph<std::string, int> &Graph() const;

    /// \internal
    /// \brief Smart pointer to private data.
    private: std::unique_ptr<RoadNetworkPrivate> dataPtr;
  };
}
#endif
