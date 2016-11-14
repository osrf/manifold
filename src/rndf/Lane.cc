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
#include "manifold/rndf/Exit.hh"
#include "manifold/rndf/Lane.hh"
#include "manifold/rndf/ParserUtils.hh"
#include "manifold/rndf/Waypoint.hh"

using namespace manifold;
using namespace rndf;

namespace manifold
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for Lane class.
    class LanePrivate
    {
      /// \brief Constructor.
      /// \param[in] _id Lane Id.
      public: explicit LanePrivate(const int _id)
        : id(_id),
          width(0.0),
          leftBoundary(Lane::Marking::UNDEFINED),
          rightBoundary(Lane::Marking::UNDEFINED)
      {
      }

      /// \brief Destructor.
      public: virtual ~LanePrivate() = default;

      /// \brief Lane identifier. E.g.: 1
      public: int id = 0;

      /// \brief Collection of waypoints.
      public: std::vector<Waypoint> waypoints;

      /// Below are the optional lane header members.

      /// \brief Lane width in meters (non-negative).
      double width = 0.0;

      /// \brief Left boundary type.
      Lane::Marking leftBoundary = Lane::Marking::UNDEFINED;

      /// \brief Right boundary type.
      Lane::Marking rightBoundary = Lane::Marking::UNDEFINED;

      /// \brief Collection of waypoints that are checkpoints.
      std::vector<Checkpoint> checkpoints;

      /// \brief Collection of waypoints that are stops. We just store the
      /// waypoint Id.
      std::vector<int> stops;

      /// \brief Collection of exits.
      std::vector<Exit> exits;
    };
  }
}

//////////////////////////////////////////////////
Lane::Lane()
  : Lane(0)
{
}

//////////////////////////////////////////////////
Lane::Lane(const int _id)
{
  int id = _id;
  if (_id <= 0)
  {
    std::cerr << "[Lane()] Invalid lane Id[" << _id << "]" << std::endl;
    id = 0;
  }

  this->dataPtr.reset(new LanePrivate(id));
}

//////////////////////////////////////////////////
Lane::Lane(const Lane &_other)
  : Lane(_other.Id())
{
  *this = _other;
}

//////////////////////////////////////////////////
Lane::~Lane()
{
}

//////////////////////////////////////////////////
bool Lane::Parse(std::ifstream &_rndfFile, const int _segmentId,
  rndf::Lane &_lane, int &_lineNumber)
{
  std::smatch result;
  std::string lineread;

  if (!nextRealLine(_rndfFile, lineread, _lineNumber))
    return false;

  // Parse the "lane ID" .
  std::regex rgxLaneId("^lane " + std::to_string(_segmentId) + "\\." +
    kRgxPositive + "$");
  std::regex_search(lineread, result, rgxLaneId);
  if (result.size() < 2)
  {
    std::cerr << "[Line " << _lineNumber << "]: Unable to parse lane element"
              << std::endl;
    std::cerr << " \"" << lineread << "\"" << std::endl;
    return false;
  }
  std::string::size_type sz;
  int laneId = std::stoi(result[1], &sz);

  // Parse "num_waypoints".
  int numWaypoints;
  if (!parsePositive(_rndfFile, "num_waypoints", numWaypoints, _lineNumber))
    return false;

  // Parse optional lane header.
  int width;
  Lane::Marking leftBoundary;
  Lane::Marking rightBoundary;
  std::vector<rndf::Checkpoint> checkpoints;
  std::vector<rndf::Waypoint> stops;
  std::vector<rndf::Exit> exits;
  if (!this->ParseHeader(_rndfFile, _segmentId, laneId, width, leftBoundary,
    rightBoundary, checkpoints, stops, exits, _lineNumber))
  {
    return false;
  }

  std::vector<rndf::Waypoint> waypoints;
  for (auto i = 0; i < numWaypoints; ++i)
  {
    // Parse a waypoint.
    rndf::Waypoint waypoint;
    if (!waypoint.Parse(_rndfFile, _segmentId, laneId, waypoint, _lineNumber))
      return false;

    if (!waypoint.Valid())
    {
      std::cerr << "Invalid waypoint" << std::endl;
      return false;
    }

    waypoints.push_back(waypoint);
  }

  // Parse "end_lane".
  if (!parseDelimiter(_rndfFile, "end_lane", _lineNumber))
    return false;

  // Populate the lane.
  _lane.SetId(laneId);
  _lane.Waypoints() = waypoints;

  return true;
}

//////////////////////////////////////////////////
int Lane::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool Lane::SetId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->dataPtr->id = _id;
  return valid;
}

//////////////////////////////////////////////////
unsigned int Lane::NumWaypoints() const
{
  return this->dataPtr->waypoints.size();
}

//////////////////////////////////////////////////
std::vector<rndf::Waypoint> &Lane::Waypoints()
{
  return this->dataPtr->waypoints;
}

//////////////////////////////////////////////////
const std::vector<rndf::Waypoint> &Lane::Waypoints() const
{
  return this->dataPtr->waypoints;
}

//////////////////////////////////////////////////
bool Lane::Waypoint(const int _wpId, rndf::Waypoint &_wp) const
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
bool Lane::UpdateWaypoint(const rndf::Waypoint &_wp)
{
  auto it = std::find(this->dataPtr->waypoints.begin(),
    this->dataPtr->waypoints.end(), _wp);

  bool found = it != this->dataPtr->waypoints.end();
  if (found)
    *it = _wp;

  return found;
}

//////////////////////////////////////////////////
bool Lane::AddWaypoint(const rndf::Waypoint &_newWaypoint)
{
  // Validate the waypoint.
  if (!_newWaypoint.Valid())
  {
    std::cerr << "[Lane::Addwaypoint() Invalid waypoint Id ["
              << _newWaypoint.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the waypoint already exists.
  if (std::find(this->dataPtr->waypoints.begin(),
        this->dataPtr->waypoints.end(), _newWaypoint) !=
          this->dataPtr->waypoints.end())
  {
    std::cerr << "[Lane::AddWaypoint() error: Existing waypoint" << std::endl;
    return false;
  }

  this->dataPtr->waypoints.push_back(_newWaypoint);
  assert(this->NumWaypoints() == this->dataPtr->waypoints.size());
  return true;
}

//////////////////////////////////////////////////
bool Lane::RemoveWaypoint(const int _wpId)
{
  rndf::Waypoint wp(_wpId, ignition::math::SphericalCoordinates());
  return (this->dataPtr->waypoints.erase(std::remove(
    this->dataPtr->waypoints.begin(), this->dataPtr->waypoints.end(), wp),
      this->dataPtr->waypoints.end()) != this->dataPtr->waypoints.end());
}

//////////////////////////////////////////////////
bool Lane::Valid() const
{
  bool valid = this->Id() > 0 && this->NumWaypoints() > 0;
  for (auto &wp : this->Waypoints())
    valid = valid && wp.Valid();
  return valid;
}

//////////////////////////////////////////////////
double Lane::Width() const
{
  return this->dataPtr->width;
}

//////////////////////////////////////////////////
bool Lane::SetWidth(const double _newWidth)
{
  bool valid = _newWidth >= 0;
  if (!valid)
  {
    std::cerr << "Lane::SetWidth() Invalid lane width [" << _newWidth << "]"
              << std::endl;
    return false;
  }

  this->dataPtr->width = _newWidth;
  return true;
}

//////////////////////////////////////////////////
Lane::Marking Lane::LeftBoundary() const
{
  return this->dataPtr->leftBoundary;
}

//////////////////////////////////////////////////
void Lane::SetLeftBoundary(const Lane::Marking &_boundary)
{
  this->dataPtr->leftBoundary = _boundary;
}

//////////////////////////////////////////////////
Lane::Marking Lane::RightBoundary() const
{
  return this->dataPtr->rightBoundary;
}

//////////////////////////////////////////////////
void Lane::SetRightBoundary(const Lane::Marking &_boundary)
{
  this->dataPtr->rightBoundary = _boundary;
}

//////////////////////////////////////////////////
unsigned int Lane::NumCheckpoints() const
{
  return this->dataPtr->checkpoints.size();
}

//////////////////////////////////////////////////
std::vector<rndf::Checkpoint> &Lane::Checkpoints()
{
  return this->dataPtr->checkpoints;
}

//////////////////////////////////////////////////
const std::vector<rndf::Checkpoint> &Lane::Checkpoints() const
{
  return this->dataPtr->checkpoints;
}

//////////////////////////////////////////////////
bool Lane::Checkpoint(const int _cpId, rndf::Checkpoint &_cp) const
{
  auto it = std::find_if(this->dataPtr->checkpoints.begin(),
    this->dataPtr->checkpoints.end(),
    [_cpId](const rndf::Checkpoint &_checkpoint)
    {
      return _checkpoint.CheckpointId() == _cpId;
    });

  bool found = it != this->dataPtr->checkpoints.end();
  if (found)
    _cp = *it;

  return found;
}

//////////////////////////////////////////////////
bool Lane::UpdateCheckpoint(const rndf::Checkpoint &_cp)
{
  auto it = std::find(this->dataPtr->checkpoints.begin(),
    this->dataPtr->checkpoints.end(), _cp);

  bool found = it != this->dataPtr->checkpoints.end();
  if (found)
    *it = _cp;

  return found;
}

//////////////////////////////////////////////////
bool Lane::AddCheckpoint(const rndf::Checkpoint &_newCheckpoint)
{
  // Validate the checkpoint.
  if (!_newCheckpoint.Valid())
  {
    std::cerr << "[Lane::AddCheckpoint() Invalid checkpoint: "
              << "checkpointId [" << _newCheckpoint.CheckpointId() << "], "
              << "waypointId [" << _newCheckpoint.WaypointId() << "]"
              << std::endl;
    return false;
  }

  // Check whether the checkpoint already exists.
  if (std::find(this->dataPtr->checkpoints.begin(),
        this->dataPtr->checkpoints.end(), _newCheckpoint) !=
          this->dataPtr->checkpoints.end())
  {
    std::cerr << "[Lane::AddCheckpoint() error: Existing checkpoint"
              << std::endl;
    return false;
  }

  this->dataPtr->checkpoints.push_back(_newCheckpoint);
  assert(this->NumCheckpoints() == this->dataPtr->checkpoints.size());
  return true;
}

//////////////////////////////////////////////////
bool Lane::RemoveCheckpoint(const int _cpId)
{
  rndf::Checkpoint cp(_cpId, 0);
  return (this->dataPtr->checkpoints.erase(std::remove(
    this->dataPtr->checkpoints.begin(), this->dataPtr->checkpoints.end(), cp),
      this->dataPtr->checkpoints.end()) != this->dataPtr->checkpoints.end());
}

//////////////////////////////////////////////////
unsigned int Lane::NumStops() const
{
  return this->dataPtr->stops.size();
}

//////////////////////////////////////////////////
std::vector<int> &Lane::Stops()
{
  return this->dataPtr->stops;
}

//////////////////////////////////////////////////
const std::vector<int> &Lane::Stops() const
{
  return this->dataPtr->stops;
}

//////////////////////////////////////////////////
bool Lane::AddStop(const int _waypointId)
{
  // Validate the waypoint Id.
  if (_waypointId <= 0)
  {
    std::cerr << "[Lane::AddStop() Invalid waypoint Id: [" << _waypointId
              << "]" << std::endl;
    return false;
  }

  // Check whether the stop already exists.
  if (std::find(this->dataPtr->stops.begin(),
        this->dataPtr->stops.end(), _waypointId) != this->dataPtr->stops.end())
  {
    std::cerr << "[Lane::AddStop() error: Existing waypoint" << std::endl;
    return false;
  }

  this->dataPtr->stops.push_back(_waypointId);
  assert(this->NumStops() == this->dataPtr->stops.size());
  return true;
}

//////////////////////////////////////////////////
bool Lane::RemoveStop(const int _waypointId)
{
  return (this->dataPtr->stops.erase(std::remove(
    this->dataPtr->stops.begin(), this->dataPtr->stops.end(), _waypointId),
      this->dataPtr->stops.end()) != this->dataPtr->stops.end());
}

//////////////////////////////////////////////////
unsigned int Lane::NumExits() const
{
  return this->dataPtr->exits.size();
}

//////////////////////////////////////////////////
std::vector<Exit> &Lane::Exits()
{
  return this->dataPtr->exits;
}

//////////////////////////////////////////////////
const std::vector<Exit> &Lane::Exits() const
{
  return this->dataPtr->exits;
}

//////////////////////////////////////////////////
bool Lane::AddExit(const Exit &_newExit)
{
  // Validate the exit unique Id.
  if (!_newExit.ExitId().Valid())
  {
    std::cerr << "[Lane::AddExit() Invalid exit Id: [" << _newExit.ExitId()
              << "]" << std::endl;
    return false;
  }

  // Validate the entry unique Id.
  if (!_newExit.EntryId().Valid())
  {
    std::cerr << "[Lane::AddExit() Invalid entry Id: [" << _newExit.EntryId()
              << "]" << std::endl;
    return false;
  }

  // Check whether the exit already exists.
  if (std::find(this->dataPtr->exits.begin(),
        this->dataPtr->exits.end(), _newExit) != this->dataPtr->exits.end())
  {
    std::cerr << "[Lane::AddExit() error: Existing exit" << std::endl;
    return false;
  }

  this->dataPtr->exits.push_back(_newExit);
  assert(this->NumExits() == this->dataPtr->exits.size());
  return true;
}

//////////////////////////////////////////////////
bool Lane::RemoveExit(const Exit &_exit)
{
  return (this->dataPtr->exits.erase(std::remove(
    this->dataPtr->exits.begin(), this->dataPtr->exits.end(), _exit),
      this->dataPtr->exits.end()) != this->dataPtr->exits.end());
}

//////////////////////////////////////////////////
bool Lane::operator==(const Lane &_other) const
{
  return this->Id() == _other.Id();
}

//////////////////////////////////////////////////
bool Lane::operator!=(const Lane &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
Lane &Lane::operator=(const Lane &_other)
{
  this->SetId(_other.Id());
  this->Waypoints() = _other.Waypoints();
  this->SetWidth(_other.Width());
  this->SetLeftBoundary(_other.LeftBoundary());
  this->SetRightBoundary(_other.RightBoundary());
  this->Checkpoints() = _other.Checkpoints();
  this->Stops() = _other.Stops();
  this->Exits() = _other.Exits();
  return *this;
}

//////////////////////////////////////////////////
bool Lane::ParseHeader(std::ifstream &_rndfFile, const int _segmentId,
  const int _laneId, int &_width, Lane::Marking &_leftBoundary,
  Lane::Marking &_rightBoundary, std::vector<rndf::Checkpoint> &_checkpoints,
  std::vector<rndf::Waypoint> &_stops, std::vector<rndf::Exit> &_exits,
  int &_lineNumber)
{
  auto oldPos = _rndfFile.tellg();
  int oldLineNumber = _lineNumber;

  std::string lineread;
  if (!nextRealLine(_rndfFile, lineread, _lineNumber))
    return false;

  // Check if we found the "waypoint" element.
  // If this is the case we should leave.
  std::regex rgxWaypointId("^" + std::to_string(_segmentId) + "\\." +
    std::to_string(_laneId) + "\\." + kRgxPositive + " " + kRgxDouble + " " +
    kRgxDouble + "$");
  if (std::regex_match(lineread, rgxWaypointId))
  {
    // Restore the file position and line number.
    // ParseHeader() shouldn't have any effect.
    _rndfFile.seekg(oldPos);
    _lineNumber = oldLineNumber;
    return true;
  }

  std::cout << "Header parsing is not supported yet" << std::endl;

  return true;
}
