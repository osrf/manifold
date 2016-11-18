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
#include "manifold/rndf/UniqueId.hh"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
UniqueId::UniqueId()
  : UniqueId(0, 0, 0)
{
}

//////////////////////////////////////////////////
UniqueId::UniqueId(const int _segmentId, const int _laneId,
    const int _waypointId)
{
  int sId = _segmentId;
  int lId = _laneId;
  int wId = _waypointId;

  if (_segmentId <= 0)
  {
    std::cerr << "[UniqueId()] Invalid segment Id[" << _segmentId
              << "]" << std::endl;
    sId = -1;
  }
  if (_laneId < 0)
  {
    std::cerr << "[UniqueId()] Invalid lane Id[" << _laneId
              << "]" << std::endl;
    lId = -1;
  }
  if (_waypointId <= 0)
  {
    std::cerr << "[UniqueId()] Invalid waypoint Id[" << _waypointId
              << "]" << std::endl;
    wId = -1;
  }

  this->segmentId = sId;
  this->laneId = lId;
  this->waypointId = wId;
}

//////////////////////////////////////////////////
UniqueId::UniqueId(const UniqueId &_other)
  : UniqueId(_other.SegmentId(), _other.LaneId(), _other.WaypointId())
{
}

//////////////////////////////////////////////////
UniqueId::~UniqueId()
{
}

//////////////////////////////////////////////////
int UniqueId::SegmentId() const
{
  return this->segmentId;
}

//////////////////////////////////////////////////
bool UniqueId::SetSegmentId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->segmentId = _id;
  return valid;
}

//////////////////////////////////////////////////
int UniqueId::LaneId() const
{
  return this->laneId;
}

//////////////////////////////////////////////////
bool UniqueId::SetLaneId(const int _id)
{
  // We allow 0 here because a perimter Id is 0.
  bool valid = _id >= 0;
  if (valid)
    this->laneId = _id;
  return valid;
}

//////////////////////////////////////////////////
int UniqueId::WaypointId() const
{
  return this->waypointId;
}

//////////////////////////////////////////////////
bool UniqueId::SetWaypointId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->waypointId = _id;
  return valid;
}

//////////////////////////////////////////////////
bool UniqueId::Valid() const
{
  return this->SegmentId() > 0 && this->LaneId() >= 0 && this->WaypointId() > 0;
}

//////////////////////////////////////////////////
bool UniqueId::operator==(const UniqueId &_other) const
{
  return this->SegmentId()  == _other.SegmentId()  &&
         this->LaneId()     == _other.LaneId()     &&
         this->WaypointId() == _other.WaypointId();
}

//////////////////////////////////////////////////
bool UniqueId::operator!=(const UniqueId &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
UniqueId &UniqueId::operator=(const UniqueId &_other)
{
  this->SetSegmentId(_other.SegmentId());
  this->SetLaneId(_other.LaneId());
  this->SetWaypointId(_other.WaypointId());
  return *this;
}
