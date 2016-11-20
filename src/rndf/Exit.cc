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

#include <cassert>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include "manifold/rndf/Exit.hh"
#include "manifold/rndf/ParserUtils.hh"
#include "manifold/rndf/UniqueId.hh"

using namespace manifold;
using namespace rndf;

//////////////////////////////////////////////////
Exit::Exit(const UniqueId &_exit, const UniqueId &_entry)
{
  if (!_exit.Valid() || !_entry.Valid())
    return;

  this->exit = _exit;
  this->entry = _entry;
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
bool Exit::Load(std::ifstream &_rndfFile, const int _x, const int _y,
  int &_lineNumber)
{
  std::smatch result;
  std::string lineread;

  if (!nextRealLine(_rndfFile, lineread, _lineNumber))
    return false;

  // Parse the "exit" .
  std::regex rgxLaneId("^exit\\s+" + std::to_string(_x) + "\\." +
    std::to_string(_y) + "\\." + kRgxPositive + "\\s+" + kRgxPositive +
    "\\." + kRgxPositive + "\\." + kRgxPositive + "\\s*(" + kRgxComment +
    ")?\\s*$");
  std::regex_search(lineread, result, rgxLaneId);
  if (result.size() < 5)
    return false;

  assert(result.size() >= 5);

  std::string::size_type sz;
  int id = std::stoi(result[1], &sz);
  int entrySegmentId = std::stoi(result[2], &sz);
  int entryLaneId = std::stoi(result[3], &sz);
  int entryWaypointId = std::stoi(result[4], &sz);

  // Populate the exit.
  UniqueId exitId(_x, _y, id);
  UniqueId entryId(entrySegmentId, entryLaneId, entryWaypointId);
  this->ExitId() = exitId;
  this->EntryId() = entryId;

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
