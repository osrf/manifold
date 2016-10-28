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

#ifndef MANIFOLD_GRAPH_HH_
#define MANIFOLD_GRAPH_HH_

#include <memory>

#include "manifold/Helpers.hh"

namespace manifold
{
  // Forward declarations.
  class GraphPrivate;

  /// \brief ToDo.
  class MANIFOLD_VISIBLE Graph
  {
    /// \brief Constructor.
    public: Graph();

    /// \brief Destructor.
    public: virtual ~Graph() = default;

    /// \internal
    /// \brief Smart pointer to private data.
    private: std::unique_ptr<GraphPrivate> dataPtr;
  };
}
#endif
