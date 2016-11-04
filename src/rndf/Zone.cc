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
#include <string>
#include <vector>

#include "manifold/rndf/ParkingSpot.hh"
#include "manifold/rndf/Perimeter.hh"
#include "manifold/rndf/Zone.hh"

using namespace manifold;
using namespace rndf;

namespace manifold
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for Zone class.
    class ZonePrivate
    {
      /// \brief Constructor.
      /// \param[in] _id Zone Id.
      public: explicit ZonePrivate(const int _id)
        : id(_id)
      {
      }

      /// \brief Destructor.
      public: virtual ~ZonePrivate() = default;

      /// \brief Unique zone identifier. E.g.: 1
      public: int id = 0;

      /// \brief Optional parking spots.
      public: std::vector<ParkingSpot> spots;

      /// \brief Perimeter of points.
      public: Perimeter perimeter;

      /// Below are the optional zone header members.

      /// \brief Zone name.
      public: std::string name;
    };
  }
}

//////////////////////////////////////////////////
Zone::Zone(const int _id)
{
  int id = _id;
  if (_id <= 0)
  {
    std::cerr << "[Zone()] Invalid lane Id[" << _id << "]" << std::endl;
    id = 0;
  }

  this->dataPtr.reset(new ZonePrivate(id));
}

//////////////////////////////////////////////////
Zone::~Zone()
{
}

//////////////////////////////////////////////////
int Zone::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool Zone::SetId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->dataPtr->id = _id;
  return valid;
}

//////////////////////////////////////////////////
unsigned int Zone::NumSpots() const
{
  return this->dataPtr->spots.size();
}

//////////////////////////////////////////////////
std::vector<ParkingSpot> &Zone::Spots()
{
  return this->dataPtr->spots;
}

//////////////////////////////////////////////////
const std::vector<ParkingSpot> &Zone::Spots() const
{
  return this->dataPtr->spots;
}

//////////////////////////////////////////////////
bool Zone::Spot(const int _psId, ParkingSpot &_ps) const
{
  auto it = std::find_if(this->dataPtr->spots.begin(),
    this->dataPtr->spots.end(),
    [_psId](const ParkingSpot &_spot)
    {
      return _spot.Id() == _psId;
    });

  bool found = it != this->dataPtr->spots.end();
  if (found)
    _ps = *it;

  return found;
}

//////////////////////////////////////////////////
bool Zone::UpdateSpot(const ParkingSpot &_ps)
{
  auto it = std::find(this->dataPtr->spots.begin(),
    this->dataPtr->spots.end(), _ps);

  bool found = it != this->dataPtr->spots.end();
  if (found)
    *it = _ps;

  return found;
}

//////////////////////////////////////////////////
bool Zone::AddSpot(const ParkingSpot &_newSpot)
{
  // Validate the parking spot.
  if (!_newSpot.Valid())
  {
    std::cerr << "[Zone::AddSpot() Invalid parking spot Id ["
              << _newSpot.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the parking spot already exists.
  if (std::find(this->dataPtr->spots.begin(),
        this->dataPtr->spots.end(), _newSpot) != this->dataPtr->spots.end())
  {
    std::cerr << "[Zone::AddSpot() error: Existing spot" << std::endl;
    return false;
  }

  this->dataPtr->spots.push_back(_newSpot);
  assert(this->NumSpots() == this->dataPtr->spots.size());
  return true;
}

//////////////////////////////////////////////////
bool Zone::RemoveSpot(const int _psId)
{
  ParkingSpot ps(_psId);
  return (this->dataPtr->spots.erase(std::remove(
    this->dataPtr->spots.begin(), this->dataPtr->spots.end(), ps),
      this->dataPtr->spots.end()) != this->dataPtr->spots.end());
}

//////////////////////////////////////////////////
rndf::Perimeter &Zone::Perimeter()
{
  return this->dataPtr->perimeter;
}

//////////////////////////////////////////////////
const rndf::Perimeter &Zone::Perimeter() const
{
  return this->dataPtr->perimeter;
}

//////////////////////////////////////////////////
std::string Zone::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void Zone::Name(const std::string &_name) const
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
bool Zone::Valid() const
{
  return this->Id() > 0;
}
