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

#include <iostream>
#include <regex>
#include <string>
#include <ignition/math/SphericalCoordinates.hh>

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
      /// \param[in] _id Waypoint Id.
      /// \param[in] _location Location of the waypoint in decimal-degrees,
      /// using ITRF00 reference frame and the GRS80 ellipsoid.
      /// \sa valid.
      public: WaypointPrivate(const std::string &_id,
                          const ignition::math::SphericalCoordinates &_location)
        : id(_id),
          location(_location)
      {
      }

      /// \brief Destructor.
      public: virtual ~WaypointPrivate() = default;

      /// \brief Unique waypoint identifier. E.g.: 17.1.1
      public: std::string id;

      /// \brief Location of the waypoint in decimal-degrees, using ITRF00
      /// reference frame and the GRS80 ellipsoid.
      public: ignition::math::SphericalCoordinates location;
    };
  }
}

//////////////////////////////////////////////////
Waypoint::Waypoint()
{
  this->dataPtr.reset(new WaypointPrivate("",
    ignition::math::SphericalCoordinates()));
}

//////////////////////////////////////////////////
Waypoint::Waypoint(const std::string &_id,
                   const ignition::math::SphericalCoordinates &_location)
{
  std::string id = _id;
  if (!valid(_id))
  {
    std::cerr << "[WayPoint()] Invalid waypoint Id[" << _id << "]" << std::endl;
    id = "";
  }

  this->dataPtr.reset(new WaypointPrivate(id, _location));
}

//////////////////////////////////////////////////
Waypoint::Waypoint(const Waypoint &_other)
{
  ignition::math::SphericalCoordinates location = _other.dataPtr->location;
  this->dataPtr.reset(new WaypointPrivate(_other.Id(), location));
}

//////////////////////////////////////////////////
Waypoint::~Waypoint()
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
  bool isValid = valid(_id);
  if (isValid)
    this->dataPtr->id = _id;
  return isValid;
}

//////////////////////////////////////////////////
ignition::math::SphericalCoordinates &Waypoint::Location()
{
  return this->dataPtr->location;
}

//////////////////////////////////////////////////
bool Waypoint::valid(const std::string &_id)
{
  const std::regex rgx("^[1-9][[:d:]]*\\.[1-9][[:d:]]*\\.[1-9][[:d:]]*$");
  return std::regex_match(_id, rgx);
}

//////////////////////////////////////////////////
bool Waypoint::operator==(const Waypoint &_other) const
{
  return this->Id() == _other.Id();
}

//////////////////////////////////////////////////
bool Waypoint::operator!=(const Waypoint &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
Waypoint &Waypoint::operator=(const Waypoint &_other)
{
  this->SetId(_other.Id());
  this->dataPtr->location = _other.dataPtr->location;
  return *this;
}
