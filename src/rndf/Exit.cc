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
#include <regex>
#include "manifold/rndf/Exit.hh"
#include "manifold/rndf/ParserUtils.hh"
#include "manifold/rndf/UniqueId.hh"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
Exit::Exit()
  : exit(0, 0, 0),
    entry(0, 0, 0)
{
}

//////////////////////////////////////////////////
Exit::Exit(const UniqueId &_exit, const UniqueId &_entry)
{
  UniqueId ex = _exit;
  UniqueId en = _entry;

  if (!_exit.Valid())
  {
    std::cerr << "[Exit()] Invalid exit Id[" << _exit << "]" << std::endl;
    ex = UniqueId();
  }
  if (!_entry.Valid())
  {
    std::cerr << "[Exit()] Invalid entry Id[" << _entry << "]" << std::endl;
    en = UniqueId();
  }

  this->exit = ex;
  this->entry = en;
}


//////////////////////////////////////////////////
Exit::Exit(const Exit &_other)
  : Exit(_other.ExitId(), _other.EntryId())
{
}

//////////////////////////////////////////////////
Exit::~Exit()
{
}

//////////////////////////////////////////////////
bool Exit::Parse(std::ifstream &_rndfFile, const int _major, const int _minor,
  rndf::Exit &_exit, int &_lineNumber)
{
  std::smatch result;
  std::string lineread;

  if (!nextRealLine(_rndfFile, lineread, _lineNumber))
    return false;

  // Parse the "exit" .
  std::regex rgxLaneId("^exit\\s+" + std::to_string(_major) + "\\." +
    std::to_string(_minor) + "\\." + kRgxPositive + "\\s+" + kRgxPositive +
    "\\." + kRgxPositive + "\\." + kRgxPositive + "\\s*(" + kRgxComment +
    ")?\\s*$");
  std::regex_search(lineread, result, rgxLaneId);
  if (result.size() < 5)
    return false;

  std::string::size_type sz;
  int id = std::stoi(result[1], &sz);
  int entrySegmentId = std::stoi(result[2], &sz);
  int entryLaneId = std::stoi(result[3], &sz);
  int entryWaypointId = std::stoi(result[4], &sz);

  // Populate the exit.
  UniqueId exitId(_major, _minor, id);
  UniqueId entryId(entrySegmentId, entryLaneId, entryWaypointId);
  _exit.ExitId() = exitId;
  _exit.EntryId() = entryId;

  return true;
}

//////////////////////////////////////////////////
const UniqueId &Exit::ExitId() const
{
  return this->exit;
}

//////////////////////////////////////////////////
UniqueId &Exit::ExitId()
{
  return this->exit;
}

//////////////////////////////////////////////////
const UniqueId &Exit::EntryId() const
{
  return this->entry;
}

//////////////////////////////////////////////////
UniqueId &Exit::EntryId()
{
  return this->entry;
}

//////////////////////////////////////////////////
bool Exit::Valid() const
{
  return this->ExitId().Valid() && this->EntryId().Valid();
}

//////////////////////////////////////////////////
bool Exit::operator==(const Exit &_other) const
{
  return this->ExitId()  == _other.ExitId()  &&
         this->EntryId() == _other.EntryId();
}

//////////////////////////////////////////////////
bool Exit::operator!=(const Exit &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
Exit &Exit::operator=(const Exit &_other)
{
  this->ExitId() = _other.ExitId();
  this->EntryId() = _other.EntryId();
  return *this;
}
