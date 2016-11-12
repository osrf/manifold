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
  }
}
