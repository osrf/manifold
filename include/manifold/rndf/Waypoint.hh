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

#include "manifold/Helpers.hh"

namespace manifold
{
  namespace rndf
  {
    // Forward declarations.
    class WaypointPrivate;

    /// \brief ToDo.
    class MANIFOLD_VISIBLE Waypoint
    {
      /// \brief Constructor.
      /// \param[in] _id Waypoint ID.
      /// \param[in] _lat Latitude of the waypoint in decimal-degrees,
      /// using ITRF00 reference frame and the GRS80 ellipsoid.
      /// \param[in] _lon Longitude of the waypoint in decimal-degrees,
      /// using ITRF00 reference frame and the GRS80 ellipsoid.
      public: Waypoint(const std::string &_id,
                       const double _lat,
                       const double _lon);

      /// \brief Destructor.
      public: virtual ~Waypoint() = default;

      /// \brief ToDo.
      public: std::string Id() const;

      /// \brief ToDo.
      public: bool SetId(const std::string &_id);

      /// \brief ToDo.
      public: double Latitude() const;

      /// \brief ToDo.
      public: bool SetLatitude(const double &_lat);

      /// \brief ToDo.
      public: double Longitude() const;

      /// \brief ToDo.
      public: bool SetLongitude(const double &_lon);

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<WaypointPrivate> dataPtr;
    };
  }
}
#endif
