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
#include <iostream>
#include <regex>
#include <string>
#include <vector>
#include <ignition/math/SphericalCoordinates.hh>

#include "manifold/rndf/Lane.hh"
#include "manifold/rndf/Waypoint.hh"

using namespace manifold;
using namespace rndf;

namespace manifold
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for LaneHeader class.
    class LaneHeaderPrivate
    {
      /// \brief Constructor.
      public: LaneHeaderPrivate() = default;

      /// \brief Destructor.
      public: virtual ~LaneHeaderPrivate() = default;
    };

    /// \internal
    /// \brief Private data for Lane class.
    class LanePrivate
    {
      /// \brief Constructor.
      /// \param[in] _id Lane Id.
      public: explicit LanePrivate(const std::string &_id)
        : id(_id)
      {
      }

      /// \brief Destructor.
      public: virtual ~LanePrivate() = default;

      /// \brief Unique lane identifier. E.g.: 1.2
      public: std::string id;

      /// \brief Collection of waypoints.
      public: std::vector<Waypoint> waypoints;
    };
  }
}

//////////////////////////////////////////////////
Lane::Lane(const std::string &_id)
{
  std::string id = _id;
  if (!valid(_id))
  {
    std::cerr << "[Lane()] Invalid lane Id[" << _id << "]" << std::endl;
    id = "";
  }

  this->dataPtr.reset(new LanePrivate(id));
}

//////////////////////////////////////////////////
Lane::~Lane()
{
}

//////////////////////////////////////////////////
std::string Lane::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool Lane::SetId(const std::string &_id)
{
  bool isValid = valid(_id);
  if (isValid)
    this->dataPtr->id = _id;
  return isValid;
}

//////////////////////////////////////////////////
unsigned int Lane::NumWaypoints() const
{
  return this->dataPtr->waypoints.size();
}

//////////////////////////////////////////////////
bool Lane::Waypoint(const unsigned int _idx, rndf::Waypoint &_waypoint)
{
  if (_idx >= this->NumWaypoints())
    return false;

  // ToDo: Check that this is a mutable rederence and not a copy.
  _waypoint = this->dataPtr->waypoints.at(_idx);
  return true;
}

//////////////////////////////////////////////////
bool Lane::Waypoint(const std::string &_wpId, rndf::Waypoint &_waypoint)
{
  auto it = std::find_if(this->dataPtr->waypoints.begin(),
    this->dataPtr->waypoints.end(),
    [&_wpId](const rndf::Waypoint &_wp)
    {
      return _wp.Id() == _wpId;
    });

  bool found = it != this->dataPtr->waypoints.end();
  if (found)
    _waypoint = *it;

  return found;
}

//////////////////////////////////////////////////
bool Lane::AddWaypoint(const rndf::Waypoint &_newWaypoint)
{
  // Validate the waypoint.
  if (!Waypoint::valid(_newWaypoint.Id()))
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
  return true;
}

//////////////////////////////////////////////////
bool Lane::RemoveWaypoint(const std::string &_wpId)
{
  rndf::Waypoint wp(_wpId, ignition::math::SphericalCoordinates());
  return (this->dataPtr->waypoints.erase(std::remove(
    this->dataPtr->waypoints.begin(), this->dataPtr->waypoints.end(), wp),
      this->dataPtr->waypoints.end()) != this->dataPtr->waypoints.end());
}

//////////////////////////////////////////////////
bool Lane::valid(const std::string &_id)
{
  const std::regex rgx("^[1-9][[:d:]]*\\.[1-9][[:d:]]*$");
  return std::regex_match(_id, rgx);
}
