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
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

#include "manifold/rndf/ParkingSpot.hh"
#include "manifold/rndf/ParserUtils.hh"
#include "manifold/rndf/Perimeter.hh"
#include "manifold/rndf/Zone.hh"

using namespace manifold;
using namespace rndf;

namespace manifold
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for ZoneHeader class.
    class ZoneHeaderPrivate
    {
      /// \brief Default constructor.
      public: ZoneHeaderPrivate() = default;

      /// \brief Destructor.
      public: virtual ~ZoneHeaderPrivate() = default;

      /// \brief Zone name.
      public: std::string name = "";
    };

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

      /// \brief Zone identifier. E.g.: 1
      public: int id = -1;

      /// \brief Optional parking spots.
      public: std::vector<ParkingSpot> spots;

      /// \brief Perimeter of points.
      public: Perimeter perimeter;

      /// Below are the optional zone header members.
      public: ZoneHeader header;
    };
  }
}

//////////////////////////////////////////////////
ZoneHeader::ZoneHeader()
{
  this->dataPtr.reset(new ZoneHeaderPrivate());
}

//////////////////////////////////////////////////
bool ZoneHeader::Load(std::ifstream &_rndfFile, const int _zoneId,
  int &_lineNumber)
{
  auto oldPos = _rndfFile.tellg();
  int oldLineNumber = _lineNumber;

  std::string lineread;
  if (!nextRealLine(_rndfFile, lineread, _lineNumber))
    return false;

  std::regex rgxPerimeterId("^perimeter\\s+" + std::to_string(_zoneId) +
    "\\.0\\s*(" + kRgxComment + ")?\\s*$");

  // Check if we found the "perimeter" element.
  // If this is the case we should leave.
  if (std::regex_match(lineread, rgxPerimeterId))
  {
    // Restore the file position and line number.
    // ParseHeader() shouldn't have any effect.
    _rndfFile.seekg(oldPos);
    _lineNumber = oldLineNumber;
    return true;
  }

  std::regex rgxHeader("^zone_name\\s+(" + kRgxString +
    ")\\s*(" + kRgxComment + ")?$");
  std::smatch result;

  std::regex_search(lineread, result, rgxHeader);
  if (result.size() <= 2)
  {
    // Invalid or header element.
    std::cerr << "[Line " << _lineNumber << "]: Unable to parse zone header "
              << "element" << std::endl;
    std::cerr << " \"" << lineread << "\"" << std::endl;
    return false;
  }

  assert(result.size() > 2);

  this->SetName(result[1]);
  return true;
}

//////////////////////////////////////////////////
std::string ZoneHeader::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void ZoneHeader::SetName(const std::string &_name) const
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
Zone::Zone()
{
  this->dataPtr.reset(new ZonePrivate(-1));
}

//////////////////////////////////////////////////
Zone::Zone(const int _id)
  : Zone()
{
  if (_id <= 0)
    return;

  this->SetId(_id);
}

//////////////////////////////////////////////////
Zone::Zone(const Zone &_other)
  : Zone()
{
  *this = _other;
}

//////////////////////////////////////////////////
Zone::~Zone()
{
}

//////////////////////////////////////////////////
bool Zone::Load(std::ifstream &_rndfFile, int &_lineNumber)
{
  int zoneId;
  if (!parsePositive(_rndfFile, "zone", zoneId, _lineNumber))
    return false;

  int numSpots;
  if (!parseNonNegative(_rndfFile, "num_spots", numSpots, _lineNumber))
    return false;

  // Parse the optional zone header.
  ZoneHeader header;
  if (!header.Load(_rndfFile, zoneId, _lineNumber))
    return false;

  // Parse the perimeter.
  rndf::Perimeter perimeter;
  if (!perimeter.Load(_rndfFile, zoneId, _lineNumber))
    return false;

  // Parse parking spots.
  std::vector<rndf::ParkingSpot> spots;
  for (auto i = 0; i < numSpots; ++i)
  {
    rndf::ParkingSpot spot;
    if (!spot.Load(_rndfFile, zoneId, _lineNumber))
      return false;

    spots.push_back(spot);
  }

  // Parse "end_zone".
  if (!parseDelimiter(_rndfFile, "end_zone", _lineNumber))
    return false;

  // Populate the zone.
  this->SetId(zoneId);
  this->Spots() = spots;
  this->Perimeter() = perimeter;
  this->SetName(header.Name());

  return true;
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
  return this->dataPtr->header.Name();
}

//////////////////////////////////////////////////
void Zone::SetName(const std::string &_name) const
{
  this->dataPtr->header.SetName(_name);
}

//////////////////////////////////////////////////
bool Zone::Valid() const
{
  return this->Id() > 0            &&
         this->Perimeter().Valid() &&
         this->Perimeter().Valid();
}

//////////////////////////////////////////////////
bool Zone::operator==(const Zone &_other) const
{
  bool valid = this->Id() == _other.Id();
  for (auto const &s : this->Spots())
    valid = valid && s.Valid();

  return valid;
}

//////////////////////////////////////////////////
bool Zone::operator!=(const Zone &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
Zone &Zone::operator=(const Zone &_other)
{
  this->SetId(_other.Id());
  this->Spots() = _other.Spots();
  this->Perimeter() = _other.Perimeter();
  this->SetName(_other.Name());
  return *this;
}
