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

#include "manifold/rndf/Checkpoint.hh"
#include "manifold/rndf/Exit.hh"
#include "manifold/rndf/ParserUtils.hh"
#include "manifold/rndf/UniqueId.hh"
#include "manifold/rndf/Waypoint.hh"

namespace manifold
{
  namespace rndf
  {
    //////////////////////////////////////////////////
    bool nextRealLine(std::ifstream &_rndfFile, std::string &_line,
      int &_lineNumber)
    {
      if (_rndfFile.eof())
        return false;

      while (std::getline(_rndfFile, _line))
      {
        ++_lineNumber;

        // Ignore blank lines or lines that only contains comments.
        std::regex rgxIgnore("^\\s*(" + kRgxComment + ")?\\s*$");
        if (!std::regex_match(_line, rgxIgnore))
          break;
      }

      return true;
    }

    //////////////////////////////////////////////////
    bool parseString(std::ifstream &_rndfFile, const std::string &_delimiter,
      std::string &_value, int &_lineNumber)
    {
      std::string lineread;
      if (!nextRealLine(_rndfFile, lineread, _lineNumber))
        return false;

      std::regex rgxName("^" + _delimiter + "\\s+(" + kRgxString + ")\\s*(" +
        kRgxComment +  ")?\\s*$");
      std::smatch result;
      std::regex_search(lineread, result, rgxName);
      if (result.size() < 2)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << _delimiter << " element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      assert(result.size() >= 2);
      _value = result[1];
      return true;
    }

    //////////////////////////////////////////////////
    bool parseDelimiter(std::ifstream &_rndfFile, const std::string &_delimiter,
      int &_lineNumber)
    {
      if (_rndfFile.eof())
        return false;

      std::string lineread;
      if (!nextRealLine(_rndfFile, lineread, _lineNumber))
        return false;

      std::regex rgxDelim("^" + _delimiter + "\\s*(" + kRgxComment + ")?\\s*$");
      if (!std::regex_match(lineread, rgxDelim))
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse delimiter ["
                  << _delimiter << "]" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      return true;
    }

    //////////////////////////////////////////////////
    bool parsePositive(std::ifstream &_rndfFile, const std::string &_delimiter,
      int &_value, int &_lineNumber)
    {
      std::string lineread;
      if (!nextRealLine(_rndfFile, lineread, _lineNumber))
        return false;

      std::regex rgxNumSegments("^" + _delimiter + "\\s+" + kRgxPositive +
        "\\s*(" + kRgxComment + ")?\\s*$");
      std::smatch result;
      std::regex_search(lineread, result, rgxNumSegments);
      if (result.size() < 2)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << _delimiter << " element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      assert(result.size() >= 2);
      std::string::size_type sz;
      _value = std::stoi(result[1], &sz);
      return true;
    }

    //////////////////////////////////////////////////
    bool parseNonNegative(std::ifstream &_rndfFile,
      const std::string &_delimiter, int &_value, int &_lineNumber)
    {
      std::string lineread;
      if (!nextRealLine(_rndfFile, lineread, _lineNumber))
        return false;

      std::regex rgxNumSegments("^" + _delimiter + "\\s+" +
        kRgxNonNegative + "\\s*(" + kRgxComment + ")?\\s*$");
      std::smatch result;
      std::regex_search(lineread, result, rgxNumSegments);
      if (result.size() < 2)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << _delimiter << " element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      assert(result.size() >= 2);
      std::string::size_type sz;
      _value = std::stoi(result[1], &sz);
      return true;
    }

    //////////////////////////////////////////////////
    bool parseNonNegative(const std::string &_input,
      const std::string &_delimiter, int &_value)
    {
      std::regex rgxLaneWidth("^" + _delimiter + "\\s+" + kRgxNonNegative +
        "\\s*(" + kRgxComment + ")?\\s*$");
      std::smatch result;
      std::regex_search(_input, result, rgxLaneWidth);
      if (result.size() < 2)
        return false;

      assert(result.size() >= 2);
      std::string::size_type sz;
      _value = std::stoi(result[1], &sz);
      return true;
    }

    //////////////////////////////////////////////////
    bool parseBoundary(const std::string &_input, Marking &_boundary)
    {
      _boundary = Marking::UNDEFINED;

      std::regex rgx("^(left|right)_boundary\\s+(double_yellow|solid_yellow|"
        "solid_white|broken_white)\\s*(" + kRgxComment + ")?\\s*$");
      std::smatch result;
      std::regex_search(_input, result, rgx);
      if (result.size() < 3)
        return false;

      assert(result.size() >= 3);
      std::string boundary = result[2];
      if (boundary == "double_yellow")
        _boundary = Marking::DOUBLE_YELLOW;
      else if (boundary == "solid_yellow")
        _boundary = Marking::SOLID_YELLOW;
      else if (boundary == "solid_white")
        _boundary = Marking::SOLID_WHITE;
      else if (boundary == "broken_white")
        _boundary = Marking::BROKEN_WHITE;
      else
        return false;

      return true;
    }

    //////////////////////////////////////////////////
    bool parseCheckpoint(const std::string &_input, const int _segmentId,
      const int _laneId, Checkpoint &_checkpoint)
    {
      std::regex rgx("^checkpoint\\s+" + std::to_string(_segmentId) + "\\." +
        std::to_string(_laneId) + "\\." + kRgxPositive + "\\s+" + kRgxPositive +
        "\\s*(" + kRgxComment + ")?\\s*$");
      std::smatch result;
      std::regex_search(_input, result, rgx);
      if (result.size() < 3)
        return false;

      assert(result.size() >= 3);
      std::string::size_type sz;
      _checkpoint.SetCheckpointId(std::stoi(result[2], &sz));
      _checkpoint.SetWaypointId(std::stoi(result[1], &sz));
      return true;
    }

    //////////////////////////////////////////////////
    bool parseStop(const std::string &_input, const int _segmentId,
      const int _laneId, UniqueId &_stop)
    {
      std::regex rgx("^stop\\s+" + std::to_string(_segmentId) + "\\." +
        std::to_string(_laneId) + "\\." + kRgxPositive + "\\s*(" + kRgxComment +
        ")?\\s*$");
      std::smatch result;
      std::regex_search(_input, result, rgx);
      if (result.size() < 2)
        return false;

      assert(result.size() >= 2);
      std::string::size_type sz;
      _stop.SetX(_segmentId);
      _stop.SetY(_laneId);
      _stop.SetZ(std::stoi(result[1], &sz));
      return true;
    }

    //////////////////////////////////////////////////
    bool parseExit(const std::string &_input, const int _segmentId,
      const int _laneId, Exit &_exit)
    {
      std::regex rgx("^exit\\s+" + std::to_string(_segmentId) + "\\." +
        std::to_string(_laneId) + "\\." + kRgxPositive + "\\s+" + kRgxUniqueId +
        "\\s*(" + kRgxComment + ")?\\s*$");
      std::smatch result;
      std::regex_search(_input, result, rgx);
      if (result.size() < 5)
        return false;

      assert(result.size() >= 5);
      std::string::size_type sz;
      UniqueId exitId(_segmentId, _laneId, std::stoi(result[1], &sz));
      UniqueId entryId(std::stoi(result[2], &sz), std::stoi(result[3], &sz),
        std::stoi(result[4], &sz));
      _exit.ExitId() = exitId;
      _exit.EntryId() = entryId;
      return true;
    }
  }
}
