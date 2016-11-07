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

#include "manifold/rndf/Lane.hh"
#include "manifold/rndf/Segment.hh"

using namespace manifold;
using namespace rndf;

namespace manifold
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for Segment class.
    class SegmentPrivate
    {
      /// \brief Constructor.
      /// \param[in] _id Segment Id.
      public: explicit SegmentPrivate(const int _id)
        : id(_id)
      {
      }

      /// \brief Destructor.
      public: virtual ~SegmentPrivate() = default;

      /// \brief Zone identifier. E.g.: 1
      public: int id = 0;

      /// \brief Collection of lanes.
      public: std::vector<rndf::Lane> lanes;

      /// Below are the optional segment header members.

      /// \brief Segment name.
      public: std::string name;
    };
  }
}

//////////////////////////////////////////////////
Segment::Segment()
  : Segment(0)
{
}

//////////////////////////////////////////////////
Segment::Segment(const int _id)
{
  int id = _id;
  if (_id <= 0)
  {
    std::cerr << "[Segment()] Invalid segment Id [" << _id << "]" << std::endl;
    id = 0;
  }

  this->dataPtr.reset(new SegmentPrivate(id));
}

//////////////////////////////////////////////////
Segment::Segment(const Segment &_other)
  : Segment(_other.Id())
{
  *this = _other;
}

//////////////////////////////////////////////////
Segment::~Segment()
{
}

//////////////////////////////////////////////////
int Segment::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool Segment::SetId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->dataPtr->id = _id;
  return valid;
}

//////////////////////////////////////////////////
unsigned int Segment::NumLanes() const
{
  return this->dataPtr->lanes.size();
}

//////////////////////////////////////////////////
std::vector<Lane> &Segment::Lanes()
{
  return this->dataPtr->lanes;
}

//////////////////////////////////////////////////
const std::vector<Lane> &Segment::Lanes() const
{
  return this->dataPtr->lanes;
}

//////////////////////////////////////////////////
bool Segment::Lane(const int _laneId, rndf::Lane &_lane) const
{
  auto it = std::find_if(this->dataPtr->lanes.begin(),
    this->dataPtr->lanes.end(),
    [_laneId](const rndf::Lane &_aLane)
    {
      return _aLane.Id() == _laneId;
    });

  bool found = it != this->dataPtr->lanes.end();
  if (found)
    _lane = *it;

  return found;
}

//////////////////////////////////////////////////
bool Segment::UpdateLane(const rndf::Lane &_lane)
{
  auto it = std::find(this->dataPtr->lanes.begin(),
    this->dataPtr->lanes.end(), _lane);

  bool found = it != this->dataPtr->lanes.end();
  if (found)
    *it = _lane;

  return found;
}

//////////////////////////////////////////////////
bool Segment::AddLane(const rndf::Lane &_newLane)
{
  // Validate the lane.
  if (!_newLane.Valid())
  {
    std::cerr << "[Segment::AddLane() Invalid lane Id ["
              << _newLane.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the lane already exists.
  if (std::find(this->dataPtr->lanes.begin(), this->dataPtr->lanes.end(),
    _newLane) != this->dataPtr->lanes.end())
  {
    std::cerr << "[Segment::AddLane() error: Existing lane" << std::endl;
    return false;
  }

  this->dataPtr->lanes.push_back(_newLane);
  assert(this->NumLanes() == this->dataPtr->lanes.size());
  return true;
}

//////////////////////////////////////////////////
bool Segment::RemoveLane(const int _laneId)
{
  rndf::Lane lane(_laneId);
  return (this->dataPtr->lanes.erase(std::remove(
    this->dataPtr->lanes.begin(), this->dataPtr->lanes.end(), lane),
      this->dataPtr->lanes.end()) != this->dataPtr->lanes.end());
}

//////////////////////////////////////////////////
std::string Segment::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void Segment::SetName(const std::string &_name) const
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
bool Segment::Valid() const
{
  bool valid = this->Id() > 0 && this->Lanes().size() > 0;
  for (auto const &lane : this->Lanes())
    valid = valid && lane.Valid();

  return valid;
}

//////////////////////////////////////////////////
bool Segment::operator==(const Segment &_other) const
{
  return this->Id() == _other.Id();
}

//////////////////////////////////////////////////
bool Segment::operator!=(const Segment &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
Segment &Segment::operator=(const Segment &_other)
{
  this->SetId(_other.Id());
  this->Lanes() = _other.Lanes();
  this->SetName(_other.Name());
  return *this;
}
