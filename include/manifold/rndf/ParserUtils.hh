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

#ifndef MANIFOLD_RNDF_PARSERUTILS_HH_
#define MANIFOLD_RNDF_PARSERUTILS_HH_

#include <iosfwd>
#include <string>

#include "manifold/Helpers.hh"

namespace manifold
{
  namespace rndf
  {
    /// \brief Regular expression that captures a non-empty string with a
    /// maximum length of 128 characters without containing any spaces,
    /// backslashes or stars.
    static const std::string kRgxString = "[^[:space:]\\*\\\\]{1,128}";

    /// \brief ToDo.
    static const std::string kPositiveData =
     "[1-9]|"
     "[1-9][[:d:]]|"
     "[1-9][[:d:]][[:d:]]|"
     "[1-9][[:d:]][[:d:]][[:d:]]|"
     "[1-2][[:d:]][[:d:]][[:d:]][[:d:]]|"
     "3[0-1][[:d:]][[:d:]][[:d:]]|"
     "32[0-6][[:d:]][[:d:]]|327[0-5][[:d:]]|3276[0-8]";

    /// \brief Regular expresion that captures a positive integer with a
    /// maximum value of 32768.
    static const std::string kRgxPositive = "(" + kPositiveData + ")";

    /// \brief Regular expresion that captures a non-negative integer with a
    /// maximum value of 32768.
    static const std::string kRgxNonNegative = "(0|" + kPositiveData + ")";

    /// \brief A comment.
    static const std::string kRgxDouble = "(-?[0-9]*\\.?[0-9]+)";

    /// \brief A comment.
    static const std::string kRgxComment = "\\/\\*[^\\*\\/]*\\*\\/";

    /// \brief ToDo.
    MANIFOLD_VISIBLE
    bool nextRealLine(std::ifstream &_rndfFile,
                      std::string &_line,
                      int &_lineNumber);

    /// \brief ToDo.
    MANIFOLD_VISIBLE
    bool parseString(std::ifstream &_rndfFile,
                     const std::string &_delimiter,
                     std::string &_value,
                     int &_lineNumber);

    /// \brief ToDo.
    MANIFOLD_VISIBLE
    bool parseDelimiter(std::ifstream &_rndfFile,
                        const std::string &_delimiter,
                        int &_lineNumber);

    /// \brief ToDo.
    MANIFOLD_VISIBLE
    bool parsePositive(std::ifstream &_rndfFile,
                       const std::string &_delimiter,
                       int &_value,
                       int &_lineNumber);

    /// \brief ToDo.
    MANIFOLD_VISIBLE
    bool parseNonNegative(std::ifstream &_rndfFile,
                         const std::string &_delimiter,
                         int &_value,
                         int &_lineNumber);
  }
}
#endif
