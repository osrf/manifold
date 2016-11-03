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
      public: WaypointPrivate(const int &_id,
                          const ignition::math::SphericalCoordinates &_location)
        : id(_id),
          location(_location)
      {
      }

      public: WaypointPrivate() = default;

      /// \brief Destructor.
      public: virtual ~WaypointPrivate() = default;

      /// \brief Unique waypoint identifier.
      public: int id;

      /// \brief Location of the waypoint in decimal-degrees, using ITRF00
      /// reference frame and the GRS80 ellipsoid.
      public: ignition::math::SphericalCoordinates location;
    };
  }
}

//////////////////////////////////////////////////
Waypoint::Waypoint()
{
  this->dataPtr.reset(new WaypointPrivate(0,
    ignition::math::SphericalCoordinates()));
}

//////////////////////////////////////////////////
Waypoint::Waypoint(const int _id,
                   const ignition::math::SphericalCoordinates &_location)
{
  int id = _id;
  if (_id <= 0)
  {
    std::cerr << "[WayPoint()] Invalid waypoint Id[" << _id << "]" << std::endl;
    id = 0;
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
int Waypoint::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool Waypoint::SetId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->dataPtr->id = _id;
  return valid;
}

//////////////////////////////////////////////////
ignition::math::SphericalCoordinates &Waypoint::Location()
{
  return this->dataPtr->location;
}

//////////////////////////////////////////////////
bool Waypoint::Valid() const
{
  return this->Id() > 0;
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
