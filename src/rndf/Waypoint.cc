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

#include <cassert>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <ignition/math/SphericalCoordinates.hh>

#include "manifold/rndf/ParserUtils.hh"
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
      public: int id = -1;

      /// \brief Location of the waypoint in decimal-degrees, using ITRF00
      /// reference frame and the GRS80 ellipsoid.
      public: ignition::math::SphericalCoordinates location;
    };
  }
}

//////////////////////////////////////////////////
Waypoint::Waypoint()
{
  this->dataPtr.reset(new WaypointPrivate(
    -1, ignition::math::SphericalCoordinates()));
}

//////////////////////////////////////////////////
Waypoint::Waypoint(const int _id,
                   const ignition::math::SphericalCoordinates &_location)
  : Waypoint()
{
  if (_id <= 0)
    return;

  this->SetId(_id);
  this->Location() = _location;
}

//////////////////////////////////////////////////
Waypoint::Waypoint(const Waypoint &_other)
  : Waypoint()
{
  *this = _other;
}

//////////////////////////////////////////////////
Waypoint::~Waypoint()
{
}

//////////////////////////////////////////////////
bool Waypoint::Load(std::ifstream &_rndfFile, const int _segmentId,
  const int _laneId, int &_lineNumber)
{
  std::string lineread;
  if (!nextRealLine(_rndfFile, lineread, _lineNumber))
    return false;

  // Parse the "waypoint" .
  std::smatch result;
  std::regex rgxWaypointId("^" + std::to_string(_segmentId) + "\\." +
    std::to_string(_laneId) + "\\." + kRgxPositive + "\\s" + kRgxDouble +
    "\\s" + kRgxDouble + "$");
  std::regex_search(lineread, result, rgxWaypointId);
  if (result.size() < 3)
  {
    std::cerr << "[Line " << _lineNumber << "]: Unable to parse waypoint "
              << " element" << std::endl;
    std::cerr << " \"" << lineread << "\"" << std::endl;
    return false;
  }

  assert(result.size() >= 3);

  std::string::size_type sz;
  int waypointId = std::stoi(result[1], &sz);
  double latitude  = std::stod(result[2], &sz);
  double longitude = std::stod(result[3], &sz);

  // Populate the waypoint.
  this->SetId(waypointId);

  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(IGN_DTOR(latitude));
  ignition::math::Angle lon(IGN_DTOR(longitude));
  ignition::math::Angle heading(ignition::math::Angle::Zero);
  double elev = 0.0;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);

  this->Location() = sc;

  return true;
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
