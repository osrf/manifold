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
#include "manifold/rndf/Checkpoint.hh"

using namespace manifold;
using namespace rndf;

namespace manifold
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for Checkpoint class.
    class CheckpointPrivate
    {
      /// \brief Constructor.
      /// \param[in] _checkpointId Checkpoint Id.
      /// \param[in] _waypointId Waypoint Id.
      public: explicit CheckpointPrivate(const int _checkpointId,
                                         const int _waypointId)
        : checkpointId(_checkpointId),
          waypointId(_waypointId)
      {
      }

      /// \brief Destructor.
      public: virtual ~CheckpointPrivate() = default;

      /// \brief Unique checkpoint identifier. E.g.: 1
      public: int checkpointId;

      /// \brief Unique waypoint identifier. E.g.: 1
      public: int waypointId;
    };
  }
}

//////////////////////////////////////////////////
Checkpoint::Checkpoint()
  : Checkpoint(0, 0)
{
}

//////////////////////////////////////////////////
Checkpoint::Checkpoint(const int _checkpointId, const int _waypointId)
{
  int checkpointId = _checkpointId;
  int waypointId = _waypointId;
  if (_checkpointId <= 0)
  {
    std::cerr << "[Checkpoint()] Invalid checkpoint Id[" << _checkpointId
              << "]" << std::endl;
    checkpointId = 0;
  }
  if (_waypointId <= 0)
  {
    std::cerr << "[Checkpoint()] Invalid waypoint Id[" << _waypointId
              << "]" << std::endl;
    waypointId = 0;
  }

  this->dataPtr.reset(new CheckpointPrivate(checkpointId, waypointId));
}

//////////////////////////////////////////////////
Checkpoint::Checkpoint(const Checkpoint &_other)
  : Checkpoint(_other.CheckpointId(), _other.WaypointId())
{
}

//////////////////////////////////////////////////
Checkpoint::~Checkpoint()
{
}

//////////////////////////////////////////////////
int Checkpoint::CheckpointId() const
{
  return this->dataPtr->checkpointId;
}

//////////////////////////////////////////////////
bool Checkpoint::SetCheckpointId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->dataPtr->checkpointId = _id;
  return valid;
}

//////////////////////////////////////////////////
int Checkpoint::WaypointId() const
{
  return this->dataPtr->waypointId;
}

//////////////////////////////////////////////////
bool Checkpoint::SetWaypointId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->dataPtr->waypointId = _id;
  return valid;
}

//////////////////////////////////////////////////
bool Checkpoint::Valid() const
{
  return this->CheckpointId() > 0 && this->WaypointId() > 0;
}

//////////////////////////////////////////////////
bool Checkpoint::operator==(const Checkpoint &_other) const
{
  return this->CheckpointId() == _other.CheckpointId();
}

//////////////////////////////////////////////////
bool Checkpoint::operator!=(const Checkpoint &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
Checkpoint &Checkpoint::operator=(const Checkpoint &_other)
{
  this->SetCheckpointId(_other.CheckpointId());
  this->SetWaypointId(_other.WaypointId());
  return *this;
}
