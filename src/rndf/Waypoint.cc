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

#include "manifold/rndf/Waypoint.hh"

using namespace manifold;
using namespace rndf;

namespace manifold
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for Waypoint class.
    class WaypointPrivate
    {
      /// \brief Constructor.
      public: WaypointPrivate(const std::string &_id, const double _lat,
        const double _lon)
        : id(_id),
          latitude(_lat),
          longitude(_lon)
      {
      }

      /// \brief Destructor.
      public: virtual ~WaypointPrivate() = default;

      /// \brief Unique waypoint identifier. E.g.: 17.1.1
      public: std::string id;

      /// \brief Latitude of the waypoint in decimal-degrees, using ITRF00
      /// reference frame and the GRS80 ellipsoid.
      public: double latitude;

      /// \brief Longitude of the waypoint in decimal-degrees, using ITRF00
      /// reference frame and the GRS80 ellipsoid.
      public: double longitude;
    };
  }
}

//////////////////////////////////////////////////
Waypoint::Waypoint(const std::string &_id, const double _lat, const double _lon)
  : dataPtr(new WaypointPrivate(_id, _lat, _lon))
{
}

//////////////////////////////////////////////////
std::string Waypoint::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool Waypoint::SetId(const std::string &_id)
{
  // ToDo: Validate the id.
  this->dataPtr->id = _id;
  return true;
}

//////////////////////////////////////////////////
double Waypoint::Latitude() const
{
  return this->dataPtr->latitude;
}

//////////////////////////////////////////////////
bool Waypoint::SetLatitude(const double &_lat)
{
  // ToDo: Validate the latitude.
  this->dataPtr->latitude = _lat;
  return true;
}

//////////////////////////////////////////////////
double Waypoint::Longitude() const
{
  return this->dataPtr->longitude;
}

//////////////////////////////////////////////////
bool Waypoint::SetLongitude(const double &_lon)
{
  // ToDo: Validate the longitude.
  this->dataPtr->longitude = _lon;
  return true;
}