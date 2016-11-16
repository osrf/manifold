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
      while (std::getline(_rndfFile, _line))
      {
        ++_lineNumber;

        // Ignore blank lines or lines that only contains comments.
        std::smatch result;
        std::regex rgxIgnore("^\\s*(" + kRgxComment +  ")?\\s*$");
        std::regex_search(_line, result, rgxIgnore);
        if (result.size() == 0)
          break;
      }

      return !_rndfFile.eof();
    }

    //////////////////////////////////////////////////
    bool parseString(std::ifstream &_rndfFile, const std::string &_delimiter,
      std::string &_value, int &_lineNumber)
    {
      std::string lineread;
      if (!nextRealLine(_rndfFile, lineread, _lineNumber))
        return false;

      std::regex rgxName("^" + _delimiter + " (" + kRgxString + ")\\s*(" +
        kRgxComment +  ")?$");
      std::smatch result;
      std::regex_search(lineread, result, rgxName);
      if (result.size() < 2)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << _delimiter << " element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

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
      nextRealLine(_rndfFile, lineread, _lineNumber);

      std::regex rgxDelim("^" + _delimiter + "$");
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

      std::regex rgxNumSegments("^" + _delimiter + " " + kRgxPositive + "$");
      std::smatch result;
      std::regex_search(lineread, result, rgxNumSegments);
      if (result.size() != 2)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << _delimiter << " element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

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

      std::regex rgxNumSegments("^" + _delimiter + " " + kRgxNonNegative + "$");
      std::smatch result;
      std::regex_search(lineread, result, rgxNumSegments);
      if (result.size() != 2)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << _delimiter << " element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      std::string::size_type sz;
      _value = std::stoi(result[1], &sz);
      return true;
    }

    //////////////////////////////////////////////////
    bool parseLaneWidth(const std::string &_input, int &_value,
      int &_lineNumber)
    {
      std::regex rgxLaneWidth("^lane_width " + kRgxNonNegative + "$");
      std::smatch result;
      std::regex_search(_input, result, rgxLaneWidth);
      if (result.size() < 2)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "lane width element" << std::endl;
        std::cerr << " \"" << _input << "\"" << std::endl;
        return false;
      }

      std::string::size_type sz;
      _value = std::stoi(result[1], &sz);
      return true;
    }

    //////////////////////////////////////////////////
    bool parseBoundary(const std::string &_input, Lane::Marking &_boundary,
      int &_lineNumber)
    {
      _boundary = Lane::Marking::UNDEFINED;

      std::regex rgx("^(left|right)_boundary (double_yellow|solid_yellow|"
        "solid_white|broken_white)$");
      std::smatch result;
      std::regex_search(_input, result, rgx);
      if (result.size() < 3)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "lane boundary element" << std::endl;
        std::cerr << " \"" << _input << "\"" << std::endl;
        return false;
      }

      std::string boundary = result[2];
      if (boundary == "double_yellow")
        _boundary = Lane::Marking::DOUBLE_YELLOW;
      else if (boundary == "solid_yellow")
        _boundary = Lane::Marking::SOLID_YELLOW;
      else if (boundary == "solid_white")
        _boundary = Lane::Marking::SOLID_WHITE;
      else if (boundary == "broken_white")
        _boundary = Lane::Marking::BROKEN_WHITE;
      else
        return false;

      return true;
    }

    //////////////////////////////////////////////////
    bool parseCheckpoint(const std::string &_input, const int _segmentId,
      const int _laneId, Checkpoint &_checkpoint, int &_lineNumber)
    {
      std::regex rgx("^checkpoint " + std::to_string(_segmentId) + "\\." +
        std::to_string(_laneId) + "\\." + kRgxPositive + " " + kRgxPositive +
        "$");
      std::smatch result;
      std::regex_search(_input, result, rgx);
      if (result.size() < 3)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "checkpoint element" << std::endl;
        std::cerr << " \"" << _input << "\"" << std::endl;
        return false;
      }

      std::string::size_type sz;
      _checkpoint.SetCheckpointId(std::stoi(result[2], &sz));
      _checkpoint.SetWaypointId(std::stoi(result[1], &sz));
      return true;
    }

    //////////////////////////////////////////////////
    bool parseStop(const std::string &_input, const int _segmentId,
      const int _laneId, UniqueId &_stop, int &_lineNumber)
    {
      std::regex rgx("^stop " + std::to_string(_segmentId) + "\\." +
        std::to_string(_laneId) + "\\." + kRgxPositive + "$");
      std::smatch result;
      std::regex_search(_input, result, rgx);
      if (result.size() < 2)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "stop element" << std::endl;
        std::cerr << " \"" << _input << "\"" << std::endl;
        return false;
      }

      std::string::size_type sz;
      _stop.SetSegmentId(_segmentId);
      _stop.SetLaneId(_laneId);
      _stop.SetWaypointId(std::stoi(result[1], &sz));
      return true;
    }

    //////////////////////////////////////////////////
    bool parseExit(const std::string &_input, const int _segmentId,
      const int _laneId, Exit &_exit, int &_lineNumber)
    {
      std::regex rgx("^exit " + std::to_string(_segmentId) + "\\." +
        std::to_string(_laneId) + "\\." + kRgxPositive + " " + kRgxUniqueId +
        "$");
      std::smatch result;
      std::regex_search(_input, result, rgx);
      if (result.size() < 5)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "exit element" << std::endl;
        std::cerr << " \"" << _input << "\"" << std::endl;
        return false;
      }

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
