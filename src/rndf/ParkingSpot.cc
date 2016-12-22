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

#include <algorithm>
#include <cassert>
#include <iostream>
#include <regex>
#include <vector>
#include <ignition/math/Helpers.hh>
#include <ignition/math/SphericalCoordinates.hh>

#include "manifold/rndf/Checkpoint.hh"
#include "manifold/rndf/ParkingSpot.hh"
#include "manifold/rndf/ParserUtils.hh"
#include "manifold/rndf/Waypoint.hh"

using namespace manifold;
using namespace rndf;

namespace manifold
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for ParkingSpotHeader class.
    class ParkingSpotHeaderPrivate
    {
      /// \brief Default constructor.
      public: ParkingSpotHeaderPrivate() = default;

      /// \brief Destructor.
      public: virtual ~ParkingSpotHeaderPrivate() = default;

      /// \brief Spot width in meters.
      public: double width = 0.0;

      /// \brief If the waypoint is a checkpoint.
      public: Checkpoint checkpoint;
    };

    /// \internal
    /// \brief Private data for ParkingSpot class.
    class ParkingSpotPrivate
    {
      /// \brief Constructor.
      /// \param[in] _id Parking spot Id.
      public: explicit ParkingSpotPrivate(const int _spotId)
        : id(_spotId)
      {
      }

      /// \brief Constructor.
      public: ParkingSpotPrivate() = default;

      /// \brief Destructor.
      public: virtual ~ParkingSpotPrivate() = default;

      /// \brief Parking spot identifier. E.g.: 1
      public: int id = -1;

      /// \brief The two waypoints that define the spot.
      public: std::vector<Waypoint> waypoints;

      /// Below are the optional spot header members.
      public: ParkingSpotHeader header;
    };
  }
}

//////////////////////////////////////////////////
ParkingSpotHeader::ParkingSpotHeader()
{
  this->dataPtr.reset(new ParkingSpotHeaderPrivate());
  this->SetWidth(0);
}

//////////////////////////////////////////////////
bool ParkingSpotHeader::Load(std::ifstream &_rndfFile, const int _zoneId,
  const int _spotId, int &_lineNumber)
{
  double width = 0;
  rndf::Checkpoint cp;

  bool checkpointFound = false;
  bool widthFound = false;

  std::regex rgxHeader("^(spot_width|checkpoint|" + kRgxUniqueId + ")\\s");

  for (auto i = 0; i < 2; ++i)
  {
    auto oldPos = _rndfFile.tellg();
    int oldLineNumber = _lineNumber;

    std::string lineread;
    if (!nextRealLine(_rndfFile, lineread, _lineNumber))
      return false;

    std::smatch result;
    std::regex_search(lineread, result, rgxHeader);
    if ((result.size() < 2)                              ||
        (result[1] == "spot_width"  && widthFound)       ||
        (result[1] == "checkpoint"  && checkpointFound))
    {
      // Invalid or repeated header element.
      std::cerr << "[Line " << _lineNumber << "]: Unable to parse spot header "
                << "element." << std::endl;
      std::cerr << " \"" << lineread << "\"" << std::endl;
      return false;
    }

    assert(result.size() >= 2);

    if (result[1] == "spot_width")
    {
      int widthFeet;
      if (!parseNonNegative(lineread, "spot_width", widthFeet))
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "spot width element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      // Convert from feet to meters.
      width = widthFeet * 0.3048;
      widthFound = true;
    }
    else if (result[1] == "checkpoint")
    {
      if (!parseCheckpoint(lineread, _zoneId, _spotId, cp))
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "spot checkpoint element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      checkpointFound = true;
    }
    else
    {
      // This is the end of the header and the start of the waypoint section.
      // Restore the file position and line number.
      // ParseHeader() shouldn't have any effect.
      _rndfFile.seekg(oldPos);
      _lineNumber = oldLineNumber;
      break;
    }
  }

  // Populate the header.
  this->SetWidth(width);
  this->Checkpoint() = cp;

  return true;
}

//////////////////////////////////////////////////
double ParkingSpotHeader::Width() const
{
  return this->dataPtr->width;
}

//////////////////////////////////////////////////
bool ParkingSpotHeader::SetWidth(const double _newWidth)
{
  bool valid = _newWidth > 0;
  if (!valid)
    return false;

  this->dataPtr->width = _newWidth;
  return true;
}


//////////////////////////////////////////////////
Checkpoint &ParkingSpotHeader::Checkpoint()
{
  return this->dataPtr->checkpoint;
}

//////////////////////////////////////////////////
const Checkpoint &ParkingSpotHeader::Checkpoint() const
{
  return this->dataPtr->checkpoint;
}

//////////////////////////////////////////////////
ParkingSpot::ParkingSpot()
{
  this->dataPtr.reset(new ParkingSpotPrivate(-1));
}

//////////////////////////////////////////////////
ParkingSpot::ParkingSpot(const int _spotId)
  : ParkingSpot()
{
  if (_spotId <= 0)
    return;

  this->SetId(_spotId);
}

//////////////////////////////////////////////////
ParkingSpot::ParkingSpot(const ParkingSpot &_other)
  : ParkingSpot()
{
  *this = _other;
}

//////////////////////////////////////////////////
ParkingSpot::~ParkingSpot()
{
}

//////////////////////////////////////////////////
bool ParkingSpot::Load(std::ifstream &_rndfFile, const int _zoneId,
  int &_lineNumber)
{
  std::smatch result;
  std::string lineread;

  if (!nextRealLine(_rndfFile, lineread, _lineNumber))
    return false;

  // Parse the "spot Id" .
  std::regex rgxSpotId("^spot\\s" + std::to_string(_zoneId) + "\\." +
    kRgxPositive + "$");
  std::regex_search(lineread, result, rgxSpotId);
  if (result.size() < 2)
  {
    std::cerr << "[Line " << _lineNumber << "]: Unable to parse spot element"
              << std::endl;
    std::cerr << " \"" << lineread << "\"" << std::endl;
    return false;
  }
  std::string::size_type sz;
  int spotId = std::stoi(result[1], &sz);

  // Parse optional parking spot header.
  ParkingSpotHeader header;
  if (!header.Load(_rndfFile, _zoneId, spotId, _lineNumber))
    return false;

  // Parse waypoints.
  std::vector<rndf::Waypoint> waypoints;
  for (auto i = 0; i < 2; ++i)
  {
    rndf::Waypoint waypoint;
    if (!waypoint.Load(_rndfFile, _zoneId, spotId, _lineNumber))
      return false;

    if (waypoint.Id() != i + 1)
    {
      std::cerr << "[Line " << _lineNumber << "]: Found non-consecutive "
                << "waypoint Id [" << waypoint.Id() << "]" << std::endl;
      return false;
    }

    waypoints.push_back(waypoint);
  }

  // Parse "end_spot".
  if (!parseDelimiter(_rndfFile, "end_spot", _lineNumber))
    return false;

  // Populate the spot.
  this->SetId(spotId);
  this->Waypoints() = waypoints;
  this->SetWidth(header.Width());
  this->Checkpoint() = header.Checkpoint();

  return true;
}

//////////////////////////////////////////////////
int ParkingSpot::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool ParkingSpot::SetId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->dataPtr->id = _id;
  return valid;
}

//////////////////////////////////////////////////
unsigned int ParkingSpot::NumWaypoints() const
{
  return this->dataPtr->waypoints.size();
}

//////////////////////////////////////////////////
std::vector<rndf::Waypoint> &ParkingSpot::Waypoints()
{
  return this->dataPtr->waypoints;
}

//////////////////////////////////////////////////
const std::vector<rndf::Waypoint> &ParkingSpot::Waypoints() const
{
  return this->dataPtr->waypoints;
}

//////////////////////////////////////////////////
bool ParkingSpot::Waypoint(const int _wpId, rndf::Waypoint &_wp) const
{
  auto it = std::find_if(this->dataPtr->waypoints.begin(),
    this->dataPtr->waypoints.end(),
    [_wpId](const rndf::Waypoint &_waypoint)
    {
      return _waypoint.Id() == _wpId;
    });

  bool found = it != this->dataPtr->waypoints.end();
  if (found)
    _wp = *it;

  return found;
}

//////////////////////////////////////////////////
bool ParkingSpot::UpdateWaypoint(const rndf::Waypoint &_wp)
{
  auto it = std::find(this->dataPtr->waypoints.begin(),
    this->dataPtr->waypoints.end(), _wp);

  bool found = it != this->dataPtr->waypoints.end();
  if (found)
    *it = _wp;

  return found;
}

//////////////////////////////////////////////////
bool ParkingSpot::AddWaypoint(const rndf::Waypoint &_newWaypoint)
{
  // Validate the waypoint.
  if (!_newWaypoint.Valid())
  {
    std::cerr << "ParkingSpot::Addwaypoint() Invalid waypoint Id ["
              << _newWaypoint.Id() << "]" << std::endl;
    return false;
  }

  // We allow a maximum of two waypoints.
  if (this->dataPtr->waypoints.size() >= 2)
  {
    std::cerr << "ParkingSpot::AddWaypoint() We only allow two waypoints "
              << "per spot and two waypoints were already found" << std::endl;
    return false;
  }

  // Check whether the waypoint already exists.
  if (std::find(this->dataPtr->waypoints.begin(),
        this->dataPtr->waypoints.end(), _newWaypoint) !=
          this->dataPtr->waypoints.end())
  {
    std::cerr << "ParkingSpot::AddWaypoint() error: Existing waypoint"
              << std::endl;
    return false;
  }

  this->dataPtr->waypoints.push_back(_newWaypoint);
  assert(this->NumWaypoints() == this->dataPtr->waypoints.size());
  return true;
}

//////////////////////////////////////////////////
bool ParkingSpot::RemoveWaypoint(const int _wpId)
{
  rndf::Waypoint wp(_wpId, ignition::math::SphericalCoordinates());
  return (this->dataPtr->waypoints.erase(std::remove(
    this->dataPtr->waypoints.begin(), this->dataPtr->waypoints.end(), wp),
      this->dataPtr->waypoints.end()) != this->dataPtr->waypoints.end());
}

//////////////////////////////////////////////////
double ParkingSpot::Width() const
{
  return this->dataPtr->header.Width();
}

//////////////////////////////////////////////////
bool ParkingSpot::SetWidth(const double _newWidth)
{
  return this->dataPtr->header.SetWidth(_newWidth);
}

//////////////////////////////////////////////////
Checkpoint &ParkingSpot::Checkpoint()
{
  return this->dataPtr->header.Checkpoint();
}

//////////////////////////////////////////////////
const Checkpoint &ParkingSpot::Checkpoint() const
{
  return this->dataPtr->header.Checkpoint();
}

//////////////////////////////////////////////////
bool ParkingSpot::Valid() const
{
  if (this->Id() <= 0 || this->NumWaypoints() != 2)
    return false;

  // All waypoints must be valid and consecutive.
  for (auto i = 0u; i < this->NumWaypoints(); ++i)
  {
    int expectedWaypointId = i + 1;
    const rndf::Waypoint &w = this->Waypoints().at(i);
    if (!w.Valid() || w.Id() != expectedWaypointId)
      return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool ParkingSpot::operator==(const ParkingSpot &_other) const
{
  return this->Id() == _other.Id();
}

//////////////////////////////////////////////////
bool ParkingSpot::operator!=(const ParkingSpot &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
ParkingSpot &ParkingSpot::operator=(const ParkingSpot &_other)
{
  this->SetId(_other.Id());
  this->Waypoints() = _other.Waypoints();
  this->SetWidth(_other.Width());
  this->Checkpoint() = _other.Checkpoint();
  return *this;
}
