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

#include "manifold/rndf/ParserUtils.hh"

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
  }
}
