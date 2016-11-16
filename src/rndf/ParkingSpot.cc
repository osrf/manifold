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
    /// \brief Private data for ParkingSpot class.
    class ParkingSpotPrivate
    {
      /// \brief Constructor.
      /// \param[in] _id Parking spot Id.
      public: explicit ParkingSpotPrivate(const int _spotId)
        : id(_spotId),
          width(0.0)
      {
      }

      /// \brief Constructor.
      public: ParkingSpotPrivate() = default;

      /// \brief Destructor.
      public: virtual ~ParkingSpotPrivate() = default;

      /// \brief Unique checkpoint identifier. E.g.: 1
      public: int id;

      /// \brief The (normally two) waypoints that define the spot.
      public: std::vector<Waypoint> waypoints;

      /// Below are the optional spot header members.

      /// \brief Spot width in meters.
      public: double width = 0.0;

      /// \brief If the waypoint is a checkpoint.
      public: Checkpoint checkpoint;
    };
  }
}

//////////////////////////////////////////////////
ParkingSpot::ParkingSpot()
  : ParkingSpot(0)
{
}

//////////////////////////////////////////////////
ParkingSpot::ParkingSpot(const int _spotId)
{
  int spotId = _spotId;
  if (_spotId <= 0)
  {
    std::cerr << "[ParkingSpot()] Invalid parking spot Id[" << _spotId
              << "]" << std::endl;
    spotId = 0;
  }

  this->dataPtr.reset(new ParkingSpotPrivate(spotId));
}

//////////////////////////////////////////////////
ParkingSpot::ParkingSpot(const ParkingSpot &_other)
  : ParkingSpot(_other.Id())
{
  *this = _other;
}

//////////////////////////////////////////////////
ParkingSpot::~ParkingSpot()
{
}

//////////////////////////////////////////////////
bool ParkingSpot::Parse(std::ifstream &_rndfFile, const int _zoneId,
  rndf::ParkingSpot &_spot, int &_lineNumber)
{
  std::smatch result;
  std::string lineread;

  if (!nextRealLine(_rndfFile, lineread, _lineNumber))
    return false;

  // Parse the "spot Id" .
  std::regex rgxSpotId("^spot " + std::to_string(_zoneId) + "\\." +
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
  int width = 0;
  rndf::Checkpoint checkpoint;
  if (!this->ParseHeader(_rndfFile, _zoneId, spotId, width, checkpoint,
    _lineNumber))
  {
    return false;
  }

  // Parse waypoints.
  std::vector<rndf::Waypoint> waypoints;
  for (auto i = 0; i < 2; ++i)
  {
    rndf::Waypoint waypoint;
    if (!waypoint.Parse(_rndfFile, _zoneId, spotId, waypoint, _lineNumber))
      return false;

    waypoints.push_back(waypoint);
  }

  // Parse "end_spot".
  if (!parseDelimiter(_rndfFile, "end_spot", _lineNumber))
    return false;

  // Populate the spot.
  _spot.SetId(spotId);
  _spot.Waypoints() = waypoints;
  _spot.SetWidth(width);
  _spot.Checkpoint() = checkpoint;

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
    std::cerr << "[ParkingSpot::Addwaypoint() Invalid waypoint Id ["
              << _newWaypoint.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the waypoint already exists.
  if (std::find(this->dataPtr->waypoints.begin(),
        this->dataPtr->waypoints.end(), _newWaypoint) !=
          this->dataPtr->waypoints.end())
  {
    std::cerr << "[ParkingSpot::AddStop() error: Existing waypoint"
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
bool ParkingSpot::Valid() const
{
  return this->Id() > 0;
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

//////////////////////////////////////////////////
double ParkingSpot::Width() const
{
  return this->dataPtr->width;
}

//////////////////////////////////////////////////
bool ParkingSpot::SetWidth(const double _newWidth)
{
  bool valid = _newWidth > 0;
  if (!valid)
  {
    std::cerr << "ParkingSpot::SetWidth() Invalid spot width [" << _newWidth
              << "]" << std::endl;
    return false;
  }

  this->dataPtr->width = _newWidth;
  return true;
}

//////////////////////////////////////////////////
Checkpoint &ParkingSpot::Checkpoint()
{
  return this->dataPtr->checkpoint;
}

//////////////////////////////////////////////////
const Checkpoint &ParkingSpot::Checkpoint() const
{
  return this->dataPtr->checkpoint;
}

//////////////////////////////////////////////////
bool ParkingSpot::ParseHeader(std::ifstream &_rndfFile, const int _zoneId,
  const int _spotId, int &_width, rndf::Checkpoint &_checkpoint,
  int &_lineNumber)
{
  _width = 0;

  bool checkpointFound = false;
  bool widthFound = false;
  std::regex rgxHeader("^((spot_width) kRgxPositive)|("
    "(checkpoint) " + std::to_string(_zoneId) + "\\." +
    std::to_string(_spotId) + "\\." + kRgxPositive + " " + kRgxPositive + ")$");
  std::regex rgxWaypointId("^" + std::to_string(_zoneId) + "\\." +
    std::to_string(_spotId) + "\\." + kRgxPositive + " " + kRgxDouble + " " +
    kRgxDouble + "$");

  for (auto i = 0; i < 2; ++i)
  {
    auto oldPos = _rndfFile.tellg();
    int oldLineNumber = _lineNumber;

    std::string lineread;
    if (!nextRealLine(_rndfFile, lineread, _lineNumber))
      return false;

    // Check if we found the "waypoint" element.
    // If this is the case we should leave.
    if (std::regex_match(lineread, rgxWaypointId))
    {
      // Restore the file position and line number.
      // ParseHeader() shouldn't have any effect.
      _rndfFile.seekg(oldPos);
      _lineNumber = oldLineNumber;
      return true;
    }

    std::smatch result;
    std::regex_search(lineread, result, rgxHeader);
    if ((result.size() <= 3) ||
        (result[1] == "spot_width" && widthFound) ||
        (result[1] == "checkpoint" && checkpointFound))
    {
      // Invalid or repeated header element.
      std::cerr << "[Line " << _lineNumber << "]: Unable to parse spot header "
                << "element." << std::endl;
      std::cerr << " \"" << lineread << "\"" << std::endl;
      return false;
    }

    std::string::size_type sz;
    if (result[1] == "spot_width")
    {
      // Save the width in meters (from feet).
      _width = std::stoi(result[2], &sz) * 0.3048;
      widthFound = true;
    }
    else
    {
      _checkpoint.SetWaypointId(std::stoi(result[2], &sz));
      _checkpoint.SetCheckpointId(std::stoi(result[5], &sz));
      checkpointFound = true;
    }
  }

  return true;
}
