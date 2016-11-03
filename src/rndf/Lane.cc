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
#include <vector>
#include <ignition/math/SphericalCoordinates.hh>

#include "manifold/rndf/Checkpoint.hh"
#include "manifold/rndf/Lane.hh"
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

      /// \brief Unique lane identifier. E.g.: 1
      public: int id;

      /// \brief Collection of waypoints.
      public: std::vector<Waypoint> waypoints;

      /// Bellow are the ptional lane header members.

      /// \brief Lane width in meters (non-negative).
      double width = 0.0;

      /// \brief Left boundary type.
      Lane::Marking leftBoundary = Lane::Marking::UNDEFINED;

      /// \brief Right boundary type.
      Lane::Marking rightBoundary = Lane::Marking::UNDEFINED;

      /// \brief Collection of checkpoints.
      std::vector<Checkpoint> checkpoints;

      /// \brief Collection of stop signs.
      // std::vector<Stop> stops;

      /// \brief Collection of exists.
      // std::vector<Exit> exits;
    };
  }
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
Lane::~Lane()
{
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
