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

#include "manifold/rndf/RNDF.hh"
#include "manifold/rndf/Segment.hh"
#include "manifold/rndf/Zone.hh"

using namespace manifold;
using namespace rndf;

namespace manifold
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for RNDF class.
    class RNDFPrivate
    {
      /// \brief Constructor.
      public: RNDFPrivate() = default;

      /// \brief Destructor.
      public: virtual ~RNDFPrivate() = default;

      /// \brief RNDF name.
      public: std::string name;

      /// \brief The collection of segments.
      public: std::vector<rndf::Segment> segments;

      /// \brief The collection of zones.
      public: std::vector<rndf::Zone> zones;

      /// Below are the optional segment header members.

      /// \brief Format version.
      public: std::string version;

      /// \brief Creation date.
      public: std::string date;
    };
  }
}

//////////////////////////////////////////////////
RNDF::RNDF()
  : dataPtr(new RNDFPrivate())
{
}

//////////////////////////////////////////////////
RNDF::~RNDF()
{
}

//////////////////////////////////////////////////
std::string RNDF::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void RNDF::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
unsigned int RNDF::NumSegments() const
{
  return this->dataPtr->segments.size();
}

//////////////////////////////////////////////////
std::vector<Segment> &RNDF::Segments()
{
  return this->dataPtr->segments;
}

//////////////////////////////////////////////////
const std::vector<Segment> &RNDF::Segments() const
{
  return this->dataPtr->segments;
}

//////////////////////////////////////////////////
bool RNDF::Segment(const int _segmentId, rndf::Segment &_segment) const
{
  auto it = std::find_if(this->dataPtr->segments.begin(),
    this->dataPtr->segments.end(),
    [_segmentId](const rndf::Segment &_aSegment)
    {
      return _aSegment.Id() == _segmentId;
    });

  bool found = it != this->dataPtr->segments.end();
  if (found)
    _segment = *it;

  return found;
}

//////////////////////////////////////////////////
bool RNDF::UpdateSegment(const rndf::Segment &_segment)
{
  auto it = std::find(this->dataPtr->segments.begin(),
    this->dataPtr->segments.end(), _segment);

  bool found = it != this->dataPtr->segments.end();
  if (found)
    *it = _segment;

  return found;
}

//////////////////////////////////////////////////
bool RNDF::AddSegment(const rndf::Segment &_newSegment)
{
  // Validate the segment.
  if (!_newSegment.Valid())
  {
    std::cerr << "[RNDF::AddSegment() Invalid segment Id ["
              << _newSegment.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the segment already exists.
  if (std::find(this->dataPtr->segments.begin(), this->dataPtr->segments.end(),
    _newSegment) != this->dataPtr->segments.end())
  {
    std::cerr << "[RNDF::AddSegment() error: Existing segment" << std::endl;
    return false;
  }

  this->dataPtr->segments.push_back(_newSegment);
  assert(this->NumSegments() == this->dataPtr->segments.size());
  return true;
}

//////////////////////////////////////////////////
bool RNDF::RemoveSegment(const int _segmentId)
{
  rndf::Segment segment(_segmentId);
  return (this->dataPtr->segments.erase(std::remove(
    this->dataPtr->segments.begin(), this->dataPtr->segments.end(), segment),
      this->dataPtr->segments.end()) != this->dataPtr->segments.end());
}

//////////////////////////////////////////////////
unsigned int RNDF::NumZones() const
{
  return this->dataPtr->zones.size();
}

//////////////////////////////////////////////////
std::vector<Zone> &RNDF::Zones()
{
  return this->dataPtr->zones;
}

//////////////////////////////////////////////////
const std::vector<Zone> &RNDF::Zones() const
{
  return this->dataPtr->zones;
}

//////////////////////////////////////////////////
bool RNDF::Zone(const int _zoneId, rndf::Zone &_zone) const
{
  auto it = std::find_if(this->dataPtr->zones.begin(),
    this->dataPtr->zones.end(),
    [_zoneId](const rndf::Zone &_aZone)
    {
      return _aZone.Id() == _zoneId;
    });

  bool found = it != this->dataPtr->zones.end();
  if (found)
    _zone = *it;

  return found;
}

//////////////////////////////////////////////////
bool RNDF::UpdateZone(const rndf::Zone &_zone)
{
  auto it = std::find(this->dataPtr->zones.begin(),
    this->dataPtr->zones.end(), _zone);

  bool found = it != this->dataPtr->zones.end();
  if (found)
    *it = _zone;

  return found;
}

//////////////////////////////////////////////////
bool RNDF::AddZone(const rndf::Zone &_newZone)
{
  // Validate the zone.
  if (!_newZone.Valid())
  {
    std::cerr << "[RNDF::AddZone() Invalid zone ["
              << _newZone.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the zone already exists.
  if (std::find(this->dataPtr->zones.begin(), this->dataPtr->zones.end(),
    _newZone) != this->dataPtr->zones.end())
  {
    std::cerr << "[RNDF::AddZone() error: Existing zone" << std::endl;
    return false;
  }

  this->dataPtr->zones.push_back(_newZone);
  assert(this->NumZones() == this->dataPtr->zones.size());
  return true;
}

//////////////////////////////////////////////////
bool RNDF::RemoveZone(const int _zoneId)
{
  rndf::Zone zone(_zoneId);
  return (this->dataPtr->zones.erase(std::remove(
    this->dataPtr->zones.begin(), this->dataPtr->zones.end(), zone),
      this->dataPtr->zones.end()) != this->dataPtr->zones.end());
}

//////////////////////////////////////////////////
std::string RNDF::Version() const
{
  return this->dataPtr->version;
}

//////////////////////////////////////////////////
void RNDF::SetVersion(const std::string &_version) const
{
  this->dataPtr->version = _version;
}

//////////////////////////////////////////////////
std::string RNDF::Date() const
{
  return this->dataPtr->date;
}

//////////////////////////////////////////////////
void RNDF::SetDate(const std::string &_newDate) const
{
  this->dataPtr->date = _newDate;
}

//////////////////////////////////////////////////
bool RNDF::Valid() const
{
  bool valid = true;
  for (auto const &segment : this->Segments())
    valid = valid && segment.Valid();

  for (auto const &zone : this->Zones())
    valid = valid && zone.Valid();

  return valid;
}
