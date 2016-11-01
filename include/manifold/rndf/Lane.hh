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

#ifndef MANIFOLD_RNDF_LANE_HH_
#define MANIFOLD_RNDF_LANE_HH_

#include <memory>
#include <string>

#include "manifold/Helpers.hh"

namespace manifold
{
  namespace rndf
  {
    // Forward declarations.
    class LanePrivate;
    class LaneHeaderPrivate;
    class Waypoint;

    /// \brief A class that represents a road lane composed by a set of
    /// waypoints.
    class MANIFOLD_VISIBLE Lane
    {
      /// \brief Constructor.
      /// \param[in] _id Lane Id.
      /// \sa valid.
      public: explicit Lane(const std::string &_id);

      /// \brief Destructor.
      public: virtual ~Lane();

      /// \brief Get the unique identifier of the lane.
      /// \return The lane Id.
      public: std::string Id() const;

      /// \brief Set the identifier of the lane.
      /// \param[in] _id New unique Id.
      /// \return True if the operation succeed or false otherwise (e.g.: if the
      /// id is not valid).
      /// \sa valid.
      public: bool SetId(const std::string &_id);

      /// \brief Get the number of waypoints stored.
      /// \return The number of waypoints of the current line.
      public: unsigned int NumWaypoints() const;

      /// \brief Get a mutable reference to a waypoint with index _idx.
      /// The first index is 0 and the the last index is (NumWaypoints() - 1).
      /// \param[in] _idx The waypoint index.
      /// \param[out] _waypoint The waypoint requested.
      /// \return True if the index is valid or false otherwise.
      public: bool Waypoint(const unsigned int _idx,
                            rndf::Waypoint &_waypoint);

      /// \brief Get a mutable reference to the waypoint with Id _wpId.
      /// \param[in] _wpId The waypoint Id.
      /// \param[out] _waypoint The waypoint requested.
      /// \return True if the waypoint is found or false otherwise.
      public: bool Waypoint(const std::string &_wpId,
                            rndf::Waypoint &_waypoint);

      /// \brief Add a new waypoint.
      /// \param[in] _newWaypoint A new waypoint to be added.
      /// \return True when the waypoint was successfully added to the list or
      /// false otherwise (e.g. if the Id of the waypoint was already existing
      /// or invalid).
      public: bool AddWaypoint(const rndf::Waypoint &_newWaypoint);

      /// \brief Remove an existing waypoint.
      /// \param[in] _wpId The waypoint Id to be removed.
      /// \return True when the waypoint was successfully deleted from the list
      /// or false otherwise (e.g. if the Id of the waypoint was not found
      /// or invalid).
      public: bool RemoveWaypoint(const std::string &_wpId);

      /// \brief \param[in] _id A lane Id to validate.
      /// \return True if the lane Id is valid. A valid id has the
      /// following format: x.y, where x, y are positive integers.
      /// E.g.: "1.2" is a valid; "1.0" is not valid.
      static bool valid(const std::string &_id);

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<LanePrivate> dataPtr;
    };
  }
}
#endif
