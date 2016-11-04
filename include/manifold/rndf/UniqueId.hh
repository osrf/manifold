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

#ifndef MANIFOLD_RNDF_UNIQUEID_HH_
#define MANIFOLD_RNDF_UNIQUEID_HH_

#include "manifold/Helpers.hh"

namespace manifold
{
  namespace rndf
  {
    /// \brief A unique id of the form x.y.z, where x,y.z are all positive
    /// numbers.
    class MANIFOLD_VISIBLE UniqueId
    {
      /// \brief Constructor.
      /// \param[in] _segmentId Segment Id (a positive number).
      /// \param[in] _laneId Lane Id (a positive number).
      /// \param[in] _waypointId Waypoint Id (a positive number).
      /// \sa Valid.
      public: explicit UniqueId(const int _segmentId,
                                const int _laneId,
                                const int _waypointId);

      /// \brief Copy constructor.
      /// \param[in] _other Other UniqueId.
      public: UniqueId(const UniqueId &_other);

      /// \brief Destructor.
      public: virtual ~UniqueId();

      /// \brief Get the segment Id.
      /// \return The segment Id.
      public: int SegmentId() const;

      /// \brief Set the segment Id.
      /// \param[in] _id New segment Id.
      /// \return True if the operation succeed or false otherwise (e.g.: if the
      /// id is not valid).
      /// \sa Valid.
      public: bool SetSegmentId(const int _id);

      /// \brief Get the lane Id.
      /// \return The lane Id.
      public: int LaneId() const;

      /// \brief Set the lane Id.
      /// \param[in] _id New lane Id.
      /// \return True if the operation succeed or false otherwise (e.g.: if the
      /// id is not valid).
      /// \sa Valid.
      public: bool SetLaneId(const int _id);

      /// \brief Get the waypoint Id.
      /// \return The waypoint Id.
      public: int WaypointId() const;

      /// \brief Set the identifier of the waypoint.
      /// \param[in] _id New unique Id.
      /// \return True if the operation succeed or false otherwise (e.g.: if the
      /// id is not valid).
      /// \sa Valid.
      public: bool SetWaypointId(const int _id);

      /// \return True if the unique Id is valid.
      public: bool Valid() const;

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other UniqueId to check for equality
      /// \return true if this == _other
      public: bool operator==(const UniqueId &_other) const;

      /// \brief Inequality
      /// \param[in] _other UniqueId to check for inequality
      /// \return true if this != _other
      public: bool operator!=(const UniqueId &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new UniqueId.
      /// \return A reference to this instance.
      public: UniqueId &operator=(const UniqueId &_other);

      /// \brief The segment Id.
      private: int segmentId;

      /// \brief The lane Id.
      private: int laneId;

      /// \brief The waypoint Id.
      private: int waypointId;
    };
  }
}
#endif
