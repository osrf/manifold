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
#include "manifold/rndf/Lane.hh"

namespace manifold
{
  namespace rndf
  {

    // Forward declarations.
    class Checkpoint;
    class Exit;
    class UniqueId;

    /// \brief Regular expression that captures a non-empty string with a
    /// maximum length of 128 characters without containing any spaces,
    /// backslashes or stars.
    static const std::string kRgxString = "[^[:space:]\\*\\\\]{1,128}";

    /// \brief Regular expression that captures a positive number with a maximum
    /// value of 32768.
    static const std::string kPositiveData =
     "[1-9]|"
     "[1-9][[:d:]]|"
     "[1-9][[:d:]][[:d:]]|"
     "[1-9][[:d:]][[:d:]][[:d:]]|"
     "[1-2][[:d:]][[:d:]][[:d:]][[:d:]]|"
     "3[0-1][[:d:]][[:d:]][[:d:]]|"
     "32[0-6][[:d:]][[:d:]]|327[0-5][[:d:]]|3276[0-8]";

    /// \brief Regular expression that captures a positive integer with a
    /// maximum value of 32768 (including parenthesis for regex grouping).
    static const std::string kRgxPositive = "(" + kPositiveData + ")";

    /// \brief Regular expression that captures a non-negative number with a
    /// maximum value of 32768.
    static const std::string kRgxNonNegative = "(0|" + kPositiveData + ")";

    /// \brief Regular expression that captures a floating point value.
    static const std::string kRgxDouble = "(-?[0-9]*\\.?[0-9]+)";

    /// \brief Regular expression that captures a unique Id "x.y.z", where
    /// 'x', 'y' and 'z' are positive numbers.
    static const std::string kRgxUniqueId = kRgxPositive + "\\." +
      kRgxPositive + "\\." + kRgxPositive;

    /// \brief Regular expression that captures a comment. A comment is
    /// delimited by "/*" and "*/" (C-style).
    static const std::string kRgxComment = "\\/\\*[^\\*\\/]*\\*\\/";

    /// \brief Consumes lines from an input stream coming from a text file.
    /// The function reads line by line until it finds a line containing
    /// parsable content. Blank lines or lines with just a comment are not
    /// considered parsable lines, so they will be consumed by this function.
    /// \param[in, out] _rndfFile Input file stream.
    /// \param[out] _line First line found with parsable content.
    /// \param[in, out] Line number pointed by the stream position indicator.
    /// \return True if a parsable line was found or false otherwise
    /// (e.g.: EoF was found).
    MANIFOLD_VISIBLE
    bool nextRealLine(std::ifstream &_rndfFile,
                      std::string &_line,
                      int &_lineNumber);

    /// \brief Checks if the next parsable line from an input stream coming from
    /// a text file matches the following expression:
    /// "<DELIMITER> <STRING> [<COMMENT>]".
    /// <DELIMITER> is a string such as "RNDF_name".
    /// <STRING> is a sequence of characters with a maximum length of 128, and
    /// do not contain any spaces, backslashes or '*'.
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in, out] _rndfFile Input file stream.
    /// \param[in] _delimiter The <DELIMITER>.
    /// \param[out] _value The parsed <STRING>.
    /// \return True if the next parsable line matched the expression or false
    /// otherwise.
    MANIFOLD_VISIBLE
    bool parseString(std::ifstream &_rndfFile,
                     const std::string &_delimiter,
                     std::string &_value,
                     int &_lineNumber);

    /// \brief Checks if the next parsable line from an input stream coming from
    /// a text file matches the following expression:
    /// "<DELIMITER> [<COMMENT>]".
    /// <DELIMITER> is a string such as "RNDF_name".
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in, out] _rndfFile Input file stream.
    /// \param[in] _delimiter The <DELIMITER>.
    /// \return True if the next parsable line matched the expression or false
    /// otherwise.
    MANIFOLD_VISIBLE
    bool parseDelimiter(std::ifstream &_rndfFile,
                        const std::string &_delimiter,
                        int &_lineNumber);

    /// \brief Checks if the next parsable line from an input stream coming from
    /// a text file matches the following expression:
    /// "<DELIMITER> <POSITIVE> [<COMMENT>]".
    /// <DELIMITER> is a string such as "RNDF_name".
    /// <POSITIVE> is an integer value between [1, 32768].
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in, out] _rndfFile Input file stream.
    /// \param[in] _delimiter The <DELIMITER>.
    /// \param[out] _value The parsed <POSITIVE>.
    /// \return True if the next parsable line matched the expression or false
    /// otherwise.
    MANIFOLD_VISIBLE
    bool parsePositive(std::ifstream &_rndfFile,
                       const std::string &_delimiter,
                       int &_value,
                       int &_lineNumber);

    /// \brief Checks if the next parsable line from an input stream coming from
    /// a text file matches the following expression:
    /// "<DELIMITER> <NON_NEGATIVE> [<COMMENT>]".
    /// <DELIMITER> is a string such as "RNDF_name".
    /// <NON_NEGATIVE> is an integer value between [0, 32768].
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in, out] _rndfFile Input file stream.
    /// \param[in] _delimiter The <DELIMITER>.
    /// \param[out] _value The parsed <NON_NEGATIVE>.
    /// \return True if the next parsable line matched the expression or false
    /// otherwise.
    MANIFOLD_VISIBLE
    bool parseNonNegative(std::ifstream &_rndfFile,
                         const std::string &_delimiter,
                         int &_value,
                         int &_lineNumber);

    /// \brief Checks if a string matches the following expression:
    /// "<DELIMITER> <NON_NEGATIVE> [<COMMENT>]".
    /// <DELIMITER> is a string such as "RNDF_name".
    /// <NON_NEGATIVE> is an integer value between [0, 32768].
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in] _input Input string.
    /// \param[out] _value The parsed <NON_NEGATIVE>.
    /// \return True if the input string matched the expression or false
    /// otherwise.
    MANIFOLD_VISIBLE
    bool parseNonNegative(const std::string &_input,
                          const std::string &_delimiter,
                          int &_value);

    /// \brief Checks if a string matches the following expression:
    /// "left_boundary <BOUNDARY> [<COMMENT>]" or.
    /// "right_boundary <BOUNDARY> [<COMMENT>]".
    /// <BOUNDARY> is a string with one of the following values (double_yellow,
    /// solid_yellow, solid_white, broken_white).
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in] _input Input string.
    /// \param[out] _boundary The parsed <BOUNDARY>.
    /// \return True if the input string matched the expression or false
    /// otherwise.
    MANIFOLD_VISIBLE
    bool parseBoundary(const std::string &_input,
                       Lane::Marking &_boundary);

    /// \brief Checks if a string matches the following expression:
    /// "checkpoint <WAYPOINT_ID> <CHECKPOINT_ID> [<COMMENT>]".
    /// <WAYPOINT_ID> is a "x.y.z" sequence, where 'x', 'y' and 'z' are positive
    /// numbers.
    /// <CHECKPOINT_ID> is a positive number.
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in] _input Input string.
    /// \param[in] _segmentId The expected segment Id (the "x").
    /// \param[in] _laneId The expected lane Id (the "y").
    /// \param[out] _checkpoint A Checkpoint object created parsing
    ///  <WAYPOINT_ID> and <CHECKPOINT_ID>.
    /// \return True if the input string matched the expression or false
    /// otherwise.
    MANIFOLD_VISIBLE
    bool parseCheckpoint(const std::string &_input,
                         const int _segmentId,
                         const int _laneId,
                         Checkpoint &_checkpoint);

    /// \brief Checks if a string matches the following expression:
    /// "stop <WAYPOINT_ID> [<COMMENT>]".
    /// <WAYPOINT_ID> is a "x.y.z" sequence, where 'x', 'y' and 'z' are positive
    /// numbers.
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in] _input Input string.
    /// \param[in] _segmentId The expected segment Id (the "x").
    /// \param[in] _laneId The expected lane Id (the "y").
    /// \param[out] _uniqueId An uniqueId object created parsing <WAYPOINT_ID>.
    /// \return True if the input string matched the expression or false
    /// otherwise.
    MANIFOLD_VISIBLE
    bool parseStop(const std::string &_input,
                   const int _segmentId,
                   const int _laneId,
                   UniqueId &_stop);

    /// \brief Checks if a string matches the following expression:
    /// "exit <EXIT_WAYPOINT_ID> <ENTRY_WAYPOINT_ID> [<COMMENT>]" or.
    /// "exit <EXIT_WAYPOINT_ID> <ENTRY_PERIMETERPOINT_ID> [<COMMENT>]".
    /// <EXIT_WAYPOINT_ID> is a "x.y.z" sequence, where 'x', 'y' and 'z' are
    /// positive numbers.
    /// <ENTRY_WAYPOINT_ID> and <ENTRY_PERIMETERPOINT_ID> are a "x.y.z"
    /// sequence, where 'x', 'y' and 'z' are positive numbers.
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in] _input Input string.
    /// \param[in] _segmentId The expected segment Id (the "x").
    /// \param[in] _laneId The expected lane Id (the "y").
    /// \param[out] _exit An Exit object created parsing <EXIT_WAYPOINT_ID> and
    /// <ENTRY_WAYPOINT_ID> or <ENTRY_PERIMETERPOINT_ID>.
    /// \return True if the input string matched the expression or false
    /// otherwise.
    MANIFOLD_VISIBLE
    bool parseExit(const std::string &_input,
                   const int _segmentId,
                   const int _laneId,
                   Exit &_exit);
  }
}
#endif
