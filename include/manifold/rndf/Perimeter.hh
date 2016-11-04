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

#ifndef MANIFOLD_RNDF_PERIMETER_HH_
#define MANIFOLD_RNDF_PERIMETER_HH_

#include <memory>
#include <vector>

#include "manifold/Helpers.hh"

namespace manifold
{
  namespace rndf
  {
    // Forward declarations.
    class Exit;
    class PerimeterPrivate;
    class Waypoint;

    /// \brief ToDo.
    class MANIFOLD_VISIBLE Perimeter
    {
      /// \brief Constructor.
      public: Perimeter();

      /// \brief Destructor.
      public: virtual ~Perimeter();

      ////////////////////
      /// Perimeter points
      ////////////////////

      /// \brief Get the number of perimeter points stored.
      /// \return The number of perimeter points in the current perimeter.
      public: unsigned int NumPoints() const;

      /// \brief Get a mutable reference to the vector of perimeter points.
      /// \return A mutable reference to the vector of perimeter points.
      public: std::vector<rndf::Waypoint> &Points();

      /// \brief Get the vector of perimeter points;
      /// \return \return The vector of perimeter points.
      public: const std::vector<rndf::Waypoint> &Points() const;

      /// \brief Get the details of one of the points with Id _wpId.
      /// \param[in] _wpId The point Id.
      /// \param[out] _wp The waypoint requested.
      /// \return True if the point was found or false otherwise.
      public: bool Point(const int _wpId, rndf::Waypoint &_wp) const;

      /// \brief Update an existing point.
      /// \param[in] _wp The updated waypoint.
      /// \return True if the point was found and updated or false otherwise.
      public: bool UpdatePoint(const rndf::Waypoint &_wp);

      /// \brief Add a new perimeter point.
      /// \param[in] _newWaypoint A new waypoint to be added.
      /// \return True when the waypoint was successfully added to the list or
      /// false otherwise (e.g. if the Id of the waypoint was already existing
      /// or invalid).
      public: bool AddPoint(const rndf::Waypoint &_newWaypoint);

      /// \brief Remove an existing perimeter point.
      /// \param[in] _wpId The waypoint Id to be removed.
      /// \return True when the waypoint was successfully deleted from the list
      /// or false otherwise (e.g. if the Id of the waypoint was not found
      /// or invalid).
      public: bool RemovePoint(const int _wpId);

      /////////
      /// Exits
      /////////

      /// \brief Get the number of exits stored.
      /// \return The number of exits in the current lane.
      public: unsigned int NumExits() const;

      /// \brief Get a mutable reference to the vector of exits.
      /// \return A mutable reference to the vector of exits.
      public: std::vector<Exit> &Exits();

      /// \brief Get the vector of stops. The elements are waypoint Ids.
      /// \return The vector of stops.
      public: const std::vector<Exit> &Exits() const;

      /// \brief Add a new exit.
      /// \param[in] _newExit The exit to add.
      /// \return True when the exit was successfully added or
      /// false otherwise (e.g. if the exit  was already existing or invalid).
      public: bool AddExit(const Exit &_newExit);

      /// \brief Remove an existing exit.
      /// \param[in] _exit The exit to be removed.
      /// \return True when the exit was successfully deleted
      /// or false otherwise (e.g. if the exit was not found or invalid).
      public: bool RemoveExit(const Exit &_exit);

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<PerimeterPrivate> dataPtr;
    };
  }
}
#endif
