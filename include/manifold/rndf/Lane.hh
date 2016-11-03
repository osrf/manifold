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
#include <vector>

#include "manifold/Helpers.hh"

namespace manifold
{
  namespace rndf
  {
    // Forward declarations.
    class Checkpoint;
    class LanePrivate;
    class Waypoint;

    /// \brief A class that represents a road lane composed by a set of
    /// waypoints.
    class MANIFOLD_VISIBLE Lane
    {
      /// \def Scope Different options for the lane boundaries.
      public: enum class Marking
      {
        /// \brief Double yellow type.
        DOUBLE_YELLOW,
        /// \brief Solid yellow type.
        SOLID_YELLOW,
        /// \brief Solid white type.
        SOLID_WHITE,
        /// \brief Broken white,
        BROKEN_WHITE,
        /// \brief Undefined.
        UNDEFINED,
      };

      /// \brief Constructor.
      /// \param[in] _id Lane Id (a positive number).
      /// \sa valid.
      public: explicit Lane(const int _id);

      /// \brief Destructor.
      public: virtual ~Lane();

      ///////
      /// Id
      ///////

      /// \brief Get the unique identifier of the lane.
      /// \return The lane Id.
      public: int Id() const;

      /// \brief Set the identifier of the lane.
      /// \param[in] _id New unique Id.
      /// \return True if the operation succeed or false otherwise (e.g.: if the
      /// id is not valid).
      /// \sa valid.
      public: bool SetId(const int _id);

      /////////////
      /// Waypoints
      /////////////

      /// \brief Get the number of waypoints stored.
      /// \return The number of waypoints in the current lane.
      public: unsigned int NumWaypoints() const;

      /// \brief Get a mutable reference to the vector of waypoints;
      /// \return A mutable reference to the vector of waypoints.
      public: std::vector<rndf::Waypoint> &Waypoints();

      /// \brief Get the vector of waypoints;
      /// \return \return The vector of waypoints.
      public: const std::vector<rndf::Waypoint> &Waypoints() const;

      /// \brief Get the details of one of the waypoints with Id _wpId.
      /// \param[in] _wpId The waypoint Id.
      /// \param[out] _wp The waypoint requested.
      /// \return True if the waypoint was found or false otherwise.
      public: bool Waypoint(const int _wpId, rndf::Waypoint &_wp) const;

      /// \brief Update an existing waypoint.
      /// \param[in] _wp The updated waypoint.
      /// \return True if the waypoint was found and updated or false otherwise.
      public: bool UpdateWaypoint(const rndf::Waypoint &_wp);

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
      public: bool RemoveWaypoint(const int _wpId);

      //////////////
      /// Validation
      //////////////

      /// \return True if the lane is valid.
      public: bool Valid() const;

      /////////
      /// Width
      /////////

      /// \brief Get the lane width in meters.
      /// \return Return the lane width in meters.
      public: double Width() const;

      /// \brief Set the lane width.
      /// \param[in] _newWidth The new width in meters.
      public: bool SetWidth(const double _newWidth);

      //////////////
      /// Boundaries
      //////////////

      /// \brief Get the left boundary type.
      /// \return The left boundary type.
      public: Marking LeftBoundary() const;

      /// \brief Set the new left boundary type.
      /// \param[in] _boundary The new left boundary type.
      public: void SetLeftBoundary(const Marking &_boundary);

      /// \brief Get the right boundary type.
      /// \return The right boundary type.
      public: Marking RightBoundary() const;

      /// \brief Set the new right boundary type.
      /// \param[in] _boundary The new right boundary type.
      public: void SetRightBoundary(const Marking &_boundary);

      ///////////////
      /// Checkpoints
      ///////////////

      /// \brief Get the number of checkpoints stored.
      /// \return The number of checkpoints in the current lane.
      public: unsigned int NumCheckpoints() const;

      /// \brief Get a mutable reference to the vector of checkpoints;
      /// \return A mutable reference to the vector of checkpoints.
      public: std::vector<rndf::Checkpoint> &Checkpoints();

      /// \brief Get the vector of checkpoints;
      /// \return The vector of checkpoints.
      public: const std::vector<rndf::Checkpoint> &Checkpoints() const;

      /// \brief Get the details of one of the checkpoints with Id _cpId.
      /// \param[in] _cpId The checkpoint Id.
      /// \param[out] _cp The checkpoint requested.
      /// \return True if the checkpoint was found or false otherwise.
      public: bool Checkpoint(const int _cpId, rndf::Checkpoint &_cp) const;

      /// \brief Update an existing checkpoint.
      /// \param[in] _cp The updated checkpoint.
      /// \return True if the checkpoint was found and updated or false
      /// otherwise.
      public: bool UpdateCheckpoint(const rndf::Checkpoint &_cp);

      /// \brief Add a new checkpoint.
      /// \param[in] _newCheckpoint A new checkpoint to be added.
      /// \return True when the checkpoint was successfully added to the list or
      /// false otherwise (e.g. if the Id of the checkpoint was already existing
      /// or invalid).
      public: bool AddCheckpoint(const rndf::Checkpoint &_newCheckpoint);

      /// \brief Remove an existing checkpoint.
      /// \param[in] _cpId The checkpoint Id to be removed.
      /// \return True when the checkpoint was successfully deleted
      /// or false otherwise (e.g. if the Id of the checkpoint was not found
      /// or invalid).
      public: bool RemoveCheckpoint(const int _cpId);


      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<LanePrivate> dataPtr;
    };
  }
}
#endif
