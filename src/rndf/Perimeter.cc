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

#include "manifold/rndf/Exit.hh"
#include "manifold/rndf/Perimeter.hh"
#include "manifold/rndf/Waypoint.hh"

using namespace manifold;
using namespace rndf;

namespace manifold
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for Perimeter class.
    class PerimeterPrivate
    {
      /// \brief Constructor.
      public: PerimeterPrivate() = default;

      /// \brief Destructor.
      public: virtual ~PerimeterPrivate() = default;

      /// \brief Collection of points.
      public: std::vector<Waypoint> points;

      /// Below are the optional perimeter header members.

      /// \brief Collection of exits.
      public: std::vector<Exit> exits;
    };
  }
}

//////////////////////////////////////////////////
Perimeter::Perimeter()
  : dataPtr(new PerimeterPrivate())
{
}

//////////////////////////////////////////////////
Perimeter::Perimeter(const Perimeter &_other)
  : Perimeter()
{
  *this = _other;
}

//////////////////////////////////////////////////
Perimeter::~Perimeter()
{
}

//////////////////////////////////////////////////
unsigned int Perimeter::NumPoints() const
{
  return this->dataPtr->points.size();
}

//////////////////////////////////////////////////
std::vector<rndf::Waypoint> &Perimeter::Points()
{
  return this->dataPtr->points;
}

//////////////////////////////////////////////////
const std::vector<rndf::Waypoint> &Perimeter::Points() const
{
  return this->dataPtr->points;
}

//////////////////////////////////////////////////
bool Perimeter::Point(const int _wpId, rndf::Waypoint &_wp) const
{
  auto it = std::find_if(this->dataPtr->points.begin(),
    this->dataPtr->points.end(),
    [_wpId](const rndf::Waypoint &_waypoint)
    {
      return _waypoint.Id() == _wpId;
    });

  bool found = it != this->dataPtr->points.end();
  if (found)
    _wp = *it;

  return found;
}

//////////////////////////////////////////////////
bool Perimeter::UpdatePoint(const rndf::Waypoint &_wp)
{
  auto it = std::find(this->dataPtr->points.begin(),
    this->dataPtr->points.end(), _wp);

  bool found = it != this->dataPtr->points.end();
  if (found)
    *it = _wp;

  return found;
}

//////////////////////////////////////////////////
bool Perimeter::AddPoint(const rndf::Waypoint &_newWaypoint)
{
  // Validate the waypoint.
  if (!_newWaypoint.Valid())
  {
    std::cerr << "[Perimeter::AddPoint() Invalid point Id ["
              << _newWaypoint.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the point already exists.
  if (std::find(this->dataPtr->points.begin(),
        this->dataPtr->points.end(), _newWaypoint) !=
          this->dataPtr->points.end())
  {
    std::cerr << "[Perimeter::AddPoint() error: Existing point" << std::endl;
    return false;
  }

  this->dataPtr->points.push_back(_newWaypoint);
  assert(this->NumPoints() == this->dataPtr->points.size());
  return true;
}

//////////////////////////////////////////////////
bool Perimeter::RemovePoint(const int _wpId)
{
  rndf::Waypoint wp(_wpId, ignition::math::SphericalCoordinates());
  return (this->dataPtr->points.erase(std::remove(
    this->dataPtr->points.begin(), this->dataPtr->points.end(), wp),
      this->dataPtr->points.end()) != this->dataPtr->points.end());
}

//////////////////////////////////////////////////
unsigned int Perimeter::NumExits() const
{
  return this->dataPtr->exits.size();
}

//////////////////////////////////////////////////
std::vector<Exit> &Perimeter::Exits()
{
  return this->dataPtr->exits;
}

//////////////////////////////////////////////////
const std::vector<Exit> &Perimeter::Exits() const
{
  return this->dataPtr->exits;
}

//////////////////////////////////////////////////
bool Perimeter::AddExit(const Exit &_newExit)
{
  // Validate the exit unique Id.
  if (!_newExit.ExitId().Valid())
  {
    std::cerr << "[Perimeter::AddExit() Invalid exit Id: ["
              << _newExit.ExitId() << "]" << std::endl;
    return false;
  }

  // Validate the entry unique Id.
  if (!_newExit.EntryId().Valid())
  {
    std::cerr << "[Perimeter::AddExit() Invalid entry Id: ["
              << _newExit.EntryId() << "]" << std::endl;
    return false;
  }

  // Check whether the exit already exists.
  if (std::find(this->dataPtr->exits.begin(),
        this->dataPtr->exits.end(), _newExit) != this->dataPtr->exits.end())
  {
    std::cerr << "[Perimeter::AddExit() error: Existing exit" << std::endl;
    return false;
  }

  this->dataPtr->exits.push_back(_newExit);
  assert(this->NumExits() == this->dataPtr->exits.size());
  return true;
}

//////////////////////////////////////////////////
bool Perimeter::RemoveExit(const Exit &_exit)
{
  return (this->dataPtr->exits.erase(std::remove(
    this->dataPtr->exits.begin(), this->dataPtr->exits.end(), _exit),
      this->dataPtr->exits.end()) != this->dataPtr->exits.end());
}

//////////////////////////////////////////////////
bool Perimeter::Valid() const
{
  return this->NumPoints() > 0;;
}

//////////////////////////////////////////////////
bool Perimeter::operator==(const Perimeter &_other) const
{
  if ((this->Points().size() != _other.Points().size()) ||
      (this->Exits().size() != _other.Exits().size()))
  {
    return false;
  }

  for (auto const &point : this->Points())
  {
    if (std::find(_other.Points().begin(), _other.Points().end(), point) ==
          _other.Points().end())
    {
      return false;
    }
  }

  for (auto const &exit : this->Exits())
  {
    if (std::find(_other.Exits().begin(), _other.Exits().end(), exit) ==
          _other.Exits().end())
    {
      return false;
    }
  }

  return true;
}

//////////////////////////////////////////////////
bool Perimeter::operator!=(const Perimeter &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
Perimeter &Perimeter::operator=(const Perimeter &_other)
{
  this->Points() = _other.Points();
  this->Exits() = _other.Exits();
  return *this;
}
