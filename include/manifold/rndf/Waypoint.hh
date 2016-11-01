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

#ifndef MANIFOLD_WAYPOINT_WAYPOINT_HH_
#define MANIFOLD_WAYPOINT_WAYPOINT_HH_

#include <memory>
#include <string>

#include "manifold/Helpers.hh"

namespace ignition
{
  namespace math
  {
    class SphericalCoordinates;
  }
}

namespace manifold
{
  namespace rndf
  {
    // Forward declarations.
    class WaypointPrivate;

    /// \brief A reference point.
    class MANIFOLD_VISIBLE Waypoint
    {
      /// \brief Default constructor.
      public: Waypoint();

      /// \brief Constructor.
      /// \param[in] _id Waypoint Id.
      /// \param[in] _location Location of the waypoint in decimal-degrees,
      /// using ITRF00 reference frame and the GRS80 ellipsoid.
      /// \sa valid.
      public: Waypoint(const std::string &_id,
                       const ignition::math::SphericalCoordinates &_location);

      /// \brief Copy constructor.
      /// \param[in] _other Other waypoint.
      public: Waypoint(const Waypoint &_other);

      /// \brief Destructor.
      public: virtual ~Waypoint();

      /// \brief Get the unique identifier of the waypoint.
      /// \return The waypoint Id.
      public: std::string Id() const;

      /// \brief Set the identifier of the waypoint.
      /// \param[in] _id New unique Id.
      /// \return True if the operation succeed or false otherwise (e.g.: if the
      /// id is not valid).
      /// \sa valid.
      public: bool SetId(const std::string &_id);

      /// \brief Get a mutable reference to the waypoint location.
      /// \return A mutable reference to the waypoint location.
      public: ignition::math::SphericalCoordinates &Location();

      /// \brief \param[in] _waypoint A waypoint Id to validate.
      /// \return True if the waypoint Id is valid. A valid waypoint Id has the
      /// following format: x.y.z, where x, y, z are positive integers.
      /// E.g.: "1.2.3" is a valid; "1.0.2" is not valid.
      static bool valid(const std::string &_waypoint);

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other Waypoint to check for equality
      /// \return true if this == _other
      public: bool operator==(const Waypoint &_other) const;

      /// \brief Inequality
      /// \param[in] _other Waypoint to check for inequality
      /// \return true if this != _other
      public: bool operator!=(const Waypoint &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new Waypoint.
      /// \return A reference to this instance.
      public: Waypoint &operator=(const Waypoint &_other);

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<WaypointPrivate> dataPtr;
    };
  }
}
#endif
