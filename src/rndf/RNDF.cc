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

      /// brief Whether the file was succesfully loaded.
      public: bool successfullyLoaded = true;

      public: RNDF::ParserState state = RNDF::ParserState::UNKNOWN;

      public: RNDF::ParserState previousState = RNDF::ParserState::UNKNOWN;
    };
  }
}

//////////////////////////////////////////////////
RNDF::RNDF()
  : dataPtr(new RNDFPrivate())
{
}

//////////////////////////////////////////////////
RNDF::RNDF(const std::string &_filepath)
  : RNDF()
{
  std::ifstream rndfFile;
  rndfFile.open(_filepath);
  if (!rndfFile)
  {
    std::cerr << "Error opening RNDF [" << _filepath << "]" << std::endl;
    this->dataPtr->successfullyLoaded = false;
    return;
  }

  std::string lineread;
  int lineNumber = 0;

  // Read line by line
  while (std::getline(rndfFile, lineread))
  {
    ++lineNumber;
    uint realCharacters = 0;

    for (uint ind = 0; ind < lineread.length(); ++ind)
    {
      if (lineread[ind] != '\r' && lineread[ind] != '\t' &&
          lineread[ind] != ' ')
      {
        ++realCharacters;
      }
    }

    // Blank lines.
    if (realCharacters == 0)
    {
      // if (verbose)
      //   printf("%d: Blank Line\n", lineNumber);
      continue;
    }

    std::string token = lineread;

    if (this->dataPtr->state != RNDF::ParserState::COMMENT)
    {
      if (token.compare("RNDF_name") == 0)
        this->ChangeState(RNDF::ParserState::GENERAL);
      else if (token.compare("segment") == 0)
        this->ChangeState(RNDF::ParserState::SEGMENTS);
      else if (token.compare("lane") == 0)
        this->ChangeState(RNDF::ParserState::LANES);
      else if (token.compare("zone") == 0)
        this->ChangeState(RNDF::ParserState::ZONES);
      else if (token.compare("perimeter") == 0)
        this->ChangeState(RNDF::ParserState::PERIMETER);
      else if (token.compare("spot") == 0)
        this->ChangeState(RNDF::ParserState::PARKING_SPOT);
      else if (token.find("/*") != std::string::npos)
        this->ChangeState(RNDF::ParserState::COMMENT);
    }

    bool valid = true;
    switch (this->dataPtr->state)
    {
      case RNDF::ParserState::COMMENT:
        // if (verbose)
        //   printf("%d: COMMENT: %s\n", line_number, lineread.c_str());
        if (lineread.find("*/") != std::string::npos)
          this->dataPtr->state = this->dataPtr->previousState;
        break;

      case RNDF::ParserState::GENERAL:
        // RNDF_NAME
        if (token.compare("RNDF_name") == 0)
          auto filename = this->ParseString(lineread, "RNDF_name", valid);
        // NUM_SEGMENTS
        else if (token.compare("num_segments") == 0)
        {
          auto numSegments = this->ParseInteger(lineread, "num_segments",
            valid);
          if (numSegments <= 0)
            valid = false;
        }
        // NUM_ZONES
        else if (token.compare("num_zones") == 0)
        {
          auto numZones = this->ParseInteger(lineread, "num_zones", valid);
          if (numZones < 0)
            valid = false;
        }
        // FORMAT_VERSION
        else if (token.compare("format_version") == 0)
        {
          auto formatVersion = this->ParseString(lineread, "format_version",
            valid);
        }
        // CREATION_DATE
        else if (token.compare("creation_date") == 0)
        {
          auto creationDate = this->ParseString(lineread, "creation_date",
            valid);
        }
        // END_FILE
        else if (token.compare("end_file") == 0)
        {
          //if (!isvalid())
          //  valid = false;
         // else if (verbose)
         //   printf("%d: RNDF file has finished parsing\n", lineNumber);
        }
        // printf("Token: |%s|\n", token.c_str());
        else
        {
          printf("%d: Unexpected token\n", lineNumber);
          valid = false;
        }
        break;

      case RNDF::ParserState::SEGMENTS:
        // SEGMENT
        if (token.compare("segment") == 0)
        {
          auto segmentId = this->ParseInteger(lineread, "segment", valid);
          if (segmentId <= 0)
            valid = false;
        }
        // NUM_LANES
        else if (token.compare("num_lanes") == 0)
        {
          auto numLanes = this->ParseInteger(lineread, "num_lanes", valid);
          if (numLanes <= 0)
            valid = false;
        }
        // SEGMENT_NAME
        else if (token.compare("segment_name") == 0)
        {
          auto segmentName = this->ParseString(lineread, "segment_name", valid);
        }
        // END_SEGMENT
        else if (token.compare("end_segment") == 0)
        {
          // if (!temp_segment.isvalid())
          //   valid = false;
          // else
          // {
          //   segments.push_back(temp_segment);
          //   if (verbose)
          //     printf("%d: segment has ended\n", line_number);
          //   this->ChangeState(RNDF::ParserState::GENERAL);
          //   temp_segment.clear();
          // }
        }
        else
        {
          printf("%d: Unexpected token\n", lineNumber);
          valid = false;
        }
        break;

      /*
      case RNDF::ParserState::LANES:
        // LANE
        if (token.compare("lane") == 0)
        {
          sprintf(temp_char, "lane %d.%%d", temp_segment.segment_id);
          if (sscanf(lineread.c_str(), temp_char, &temp_lane.lane_id) == 1){
            if (verbose)
              printf("%d: Lane number is %d\n",
               line_number, temp_lane.lane_id);
          }
          else valid=false;
          if (temp_lane.lane_id <= 0) valid = false;
        }
        //NUM_WAYPOINTS
        else if(token.compare("num_waypoints") == 0){
          temp_lane.number_of_waypoints =
            parse_integer(lineread, std::string("num_waypoints"),
              line_number, valid, verbose);
          if (temp_lane.number_of_waypoints <= 0) valid = false;
        }
        //LANE_WIDTH
        else if(token.compare("lane_width") == 0){
          temp_lane.lane_width =
            parse_integer(lineread, std::string("lane_width"),
              line_number, valid, verbose);
          if (temp_lane.lane_width <= 0) valid = false;
        }
        //LEFT_BOUNDARY
        else if(token.compare("left_boundary") == 0){
          temp_lane.left_boundary =
            parse_boundary(lineread, valid);
          if (verbose)
            printf("%d: left boundary type is %d\n",
             line_number, temp_lane.left_boundary);
        }
        //RIGHT_BOUNDARY
        else if(token.compare("right_boundary") == 0){
          temp_lane.right_boundary =
            parse_boundary(lineread, valid);
          if (verbose)
            printf("%d: right boundary type is %d\n",
             line_number, temp_lane.right_boundary);
        }
        //CHECKPOINT
        else if(token.compare("checkpoint") == 0){
          Checkpoint checkpoint(lineread, temp_segment.segment_id,
              temp_lane.lane_id, line_number, valid, verbose);
          temp_lane.checkpoints.push_back(checkpoint);
        }
        //STOP
        else if(token.compare("stop") == 0){
          Stop stop(lineread, temp_segment.segment_id,
              temp_lane.lane_id, line_number, valid, verbose);
          temp_lane.stops.push_back(stop);
        }
        //EXIT
        else if(token.compare("exit") == 0){
          Exit exit(lineread, temp_segment.segment_id,
              temp_lane.lane_id, line_number, valid, verbose);
          temp_lane.exits.push_back(exit);
        }
        //END_LANE
        else if(token.compare("end_lane") == 0){
          if(temp_lane.number_of_waypoints != (int)temp_lane.waypoints.size())
            printf("Number of waypoints in lane does not match num_waypoints\n");
          if (!temp_lane.isvalid())
            valid = false;
          else{
            temp_segment.lanes.push_back(temp_lane);
            if (verbose)
              printf("%d: lane has ended\n", line_number);
            change_state(previous_state, state, SEGMENTS);
            temp_lane.clear();
          }
        }

        //NO TOKEN
        else{
          //WAYPOINT
          sprintf(temp_char, "%d.%d.", temp_segment.segment_id,
            temp_lane.lane_id);
          if(token.find(temp_char) != std::string::npos ){
            LL_Waypoint wp(lineread, temp_segment.segment_id,
               temp_lane.lane_id, line_number, valid, verbose);
            temp_lane.waypoints.push_back(wp);
          }
          else{
            printf("%d: Unexpected token\n", line_number);
            valid=false;
          }
        }
        break;

      case ZONES:
        //ZONE
        if(token.compare("zone") == 0){
          temp_zone.zone_id =
            parse_integer(lineread, std::string("zone"),
              line_number, valid, verbose);
          if (temp_zone.zone_id <= 0) valid = false;

        }
        //NUM_SPOTS
        else if(token.compare("num_spots") == 0){
          temp_zone.number_of_parking_spots =
            parse_integer(lineread, std::string("num_spots"),
              line_number, valid, verbose);
          if (temp_zone.number_of_parking_spots < 0) valid = false;
        }
        //ZONE_NAME
        else if(token.compare("zone_name") == 0)
          temp_zone.zone_name =
            parse_string(lineread, std::string("zone_name"),
             line_number, valid, verbose);
        //END_ZONE
        else if(token.compare("end_zone") == 0){
          if(!temp_zone.isvalid())
            valid = false;
          else{
            zones.push_back(temp_zone);
            if (verbose)
              printf("%d: zone has ended\n", line_number);
            change_state(previous_state, state, GENERAL);
            temp_zone.clear();
          }
        }
        else {
          printf("%d: Unexpected token\n", line_number);
          valid=false;
        }
        break;

      case PERIMETER:
        //PERIMETER
        if(token.compare("perimeter") == 0){
          sprintf(temp_char,"perimeter %d.%%d" , temp_zone.zone_id);
          if (sscanf(lineread.c_str(), temp_char,
               &temp_perimeter.perimeter_id) == 1){
            if (verbose)
              printf("%d: Perimeter id is %d\n",
               line_number, temp_perimeter.perimeter_id);
          }
          else valid=false;
          if (temp_perimeter.perimeter_id != 0)
            valid = false;
        }
        //NUM_PERIMETER_POINTS
        else if(token.compare("num_perimeterpoints") == 0){
          temp_perimeter.number_of_perimeterpoints =
            parse_integer(lineread, std::string("num_perimeterpoints"),
              line_number, valid, verbose);
          if (temp_perimeter.number_of_perimeterpoints <= 0)
            valid = false;
        }
        //EXIT
        else if(token.compare("exit") == 0){
          Exit exit(lineread, temp_zone.zone_id, 0, line_number, valid,
              verbose);
          temp_perimeter.exits_from_perimeter.push_back(exit);
        }
        //END_PERIMETER
        else if(token.compare("end_perimeter") == 0){
          if(!temp_perimeter.isvalid())
            valid = false;
          else{
            temp_zone.perimeter = temp_perimeter;
            if (verbose)
              printf("%d: perimeter has ended\n", line_number);
            change_state(previous_state, state, ZONES);
            temp_perimeter.clear();
          }
        }
        else{
          //WAYPOINT
          sprintf(temp_char, "%d.%d.", temp_zone.zone_id, 0);
          if(token.find(temp_char) != std::string::npos ){
            LL_Waypoint wp(lineread, temp_zone.zone_id, 0, line_number,
               valid, verbose);
            temp_perimeter.perimeterpoints.push_back(wp);
          }
          else valid=false;
        }
        break;

      case PARKING_SPOT:
        //SPOT
        if(token.compare("spot") == 0){
          sprintf(temp_char,"spot %d.%%d" , temp_zone.zone_id);
          if (sscanf(lineread.c_str(), temp_char, &temp_spot.spot_id) == 1){
            if (verbose)
              printf("%d: Spot id is %d\n", line_number, temp_spot.spot_id);
          }
          else valid=false;
          if (temp_spot.spot_id <= 0) valid = false;
        }
        //SPOT_WIDTH
        else if(token.compare("spot_width") == 0){
          temp_spot.spot_width =
            parse_integer(lineread, std::string("spot_width"),
              line_number, valid, verbose);
          if (temp_spot.spot_width <= 0) valid = false;
        }
        //CHECKPOINT
        else if(token.compare("checkpoint") == 0)
          temp_spot.checkpoint = Checkpoint(lineread, temp_zone.zone_id,
                    temp_spot.spot_id,
                    line_number, valid, verbose);
        //END_SPOT
        else if(token.compare("end_spot") == 0){
          if(!temp_spot.isvalid())
            valid = false;
          else{
            temp_zone.spots.push_back(temp_spot);
            if (verbose)
              printf("%d: spot has ended\n", line_number);
            change_state(previous_state, state, ZONES);
            temp_spot.clear();
          }
        }
        else{
          //WAYPOINT
          sprintf(temp_char, "%d.%d.", temp_zone.zone_id, temp_spot.spot_id);
          if(token.find(temp_char) != std::string::npos ){
            LL_Waypoint wp(lineread, temp_zone.zone_id,
               temp_spot.spot_id, line_number, valid, verbose);
            temp_spot.waypoints.push_back(wp);
          }
          else{
            printf("%d: Unexpected token\n", line_number);
            valid=false;
          }
        }
        break;

      */
      case RNDF::ParserState::UNKNOWN:
        printf("%d: Unexpected token\n", lineNumber);
        valid = false;

      default:
        break;
    };
  }
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
  bool valid = this->dataPtr->successfullyLoaded &&
               this->Segments().size() > 0u;
  for (auto const &segment : this->Segments())
    valid = valid && segment.Valid();

  for (auto const &zone : this->Zones())
    valid = valid && zone.Valid();

  return valid;
}

//////////////////////////////////////////////////
void RNDF::ChangeState(const ParserState &_newState)
{
  this->dataPtr->previousState = this->dataPtr->state;
  this->dataPtr->state = _newState;
}

//////////////////////////////////////////////////
std::string RNDF::ParseString(const std::string &_line,
  const std::string &_token, bool &_valid)
{
  char delim = ' ';
  _valid = false;

  // No whitespace or more than one.
  if (std::count(_line.begin(), _line.end(), delim) != 1u)
    return "";

  auto delimPos = _line.find(delim);
  // Nothing after the whitespace.
  if (delimPos == _line.size() - 1)
    return "";

  _valid = true;
  return _line.substr(delimPos + 1, _line.size() - delimPos);
}

//////////////////////////////////////////////////
int RNDF::ParseInteger(const std::string &_line,
  const std::string &_token, bool &_valid)
{
  char delim = ' ';
  _valid = false;

  // No whitespace or more than one.
  if (std::count(_line.begin(), _line.end(), delim) != 1u)
    return -1;

  auto delimPos = _line.find(delim);
  // Nothing after the whitespace.
  if (delimPos == _line.size() - 1)
    return -1;


  int intValue;
  auto strValue = _line.substr(delimPos + 1, _line.size() - delimPos);
  auto expectedSize = strValue.size();
  try
  {
    std::string::size_type sz;
    intValue = std::stoi(strValue, &sz);
    if ((sz != expectedSize) || (intValue < 0))
      return -1;
  }
  catch (const std::invalid_argument &_ia)
  {
    return -1;
  }
  catch (const std::out_of_range &_oor)
  {
    return -1;
  }

  _valid = true;
  return intValue;
}

//////////////////////////////////////////////////
int RNDF::ParseInteger(const std::string &_line, bool &_valid)
{
  _valid = false;

  int intValue;
  auto expectedSize = _line.size();
  try
  {
    std::string::size_type sz;
    intValue = std::stoi(_line, &sz);
    if ((sz != expectedSize) || (intValue < 0))
      return false;
  }
  catch (const std::invalid_argument &_ia)
  {
    return false;
  }
  catch (const std::out_of_range &_oor)
  {
    return false;
  }

  _valid = true;
  return intValue;
}
