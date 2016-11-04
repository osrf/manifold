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

#ifndef MANIFOLD_RNDF_ZONE_HH_
#define MANIFOLD_RNDF_ZONE_HH_

#include <memory>
#include <string>

#include "manifold/Helpers.hh"

namespace manifold
{
  namespace rndf
  {
    // Forward declarations.
    class ParkingSpot;
    class Perimeter;
    class ZonePrivate;

    /// \brief ToDo.
    class MANIFOLD_VISIBLE Zone
    {
      /// \brief Constructor.
      /// \param[in] _id Zone Id (a positive number).
      /// \sa valid.
      public: Zone(const int _id);

      /// \brief Destructor.
      public: virtual ~Zone();

      ///////
      /// Id
      ///////

      /// \brief Get the unique identifier of the zone.
      /// \return The zone Id.
      public: int Id() const;

      /// \brief Set the identifier of the zone.
      /// \param[in] _id New unique Id.
      /// \return True if the operation succeed or false otherwise
      /// (e.g.: if theid is not valid).
      /// \sa valid.
      public: bool SetId(const int _id);

      /////////////////
      /// Parking spots
      /////////////////

      /// \brief Get the number of parking spots stored.
      /// \return The number of parking spots in the current zone.
      public: unsigned int NumSpots() const;

      /// \brief Get a mutable reference to the vector of parking spots.
      /// \return A mutable reference to the vector of parking spots.
      public: std::vector<ParkingSpot> &Spots();

      /// \brief Get the vector of parking spots.
      /// \return \return The vector of parking spots.
      public: const std::vector<ParkingSpot> &Spots() const;

      /// \brief Get the details of one of the parking spots with Id _psId.
      /// \param[in] _psId The parking spot Id.
      /// \param[out] _ps The parking spot requested.
      /// \return True if the parking spot was found or false otherwise.
      public: bool Spot(const int _psId, ParkingSpot &_ps) const;

      /// \brief Update an existing parking spot.
      /// \param[in] _ps The updated parking spot.
      /// \return True if the spot was found and updated or false otherwise.
      public: bool UpdateSpot(const ParkingSpot &_ps);

      /// \brief Add a new parking spot.
      /// \param[in] _newSpot A new spot to be added.
      /// \return True when the parking spot was successfully added to the list
      /// or false otherwise (e.g. if the Id of the spot was already existing
      /// or invalid).
      public: bool AddSpot(const ParkingSpot &_newSpot);

      /// \brief Remove an existing parking spot.
      /// \param[in] _psId The parking spot Id to be removed.
      /// \return True when the sopt was successfully deleted from the list
      /// or false otherwise (e.g. if the Id of the waypoint was not found
      /// or invalid).
      public: bool RemoveSpot(const int _psId);

      /////////////
      /// Perimeter
      /////////////

      /// \brief Get a mutable reference to the perimeter.
      /// \return A mutable reference to the perimeter.
      public: rndf::Perimeter &Perimeter();

      /// \brief Get the perimeter.
      /// \return The perimeter.
      public: const rndf::Perimeter &Perimeter() const;

      ////////
      /// Name
      ////////

      /// \brief Get the zone name. E.g.: "North_parking_lot".
      /// \return The zone name.
      public: std::string Name() const;

      /// \brief Set the zone name. E.g.: "North_parking_lot".
      /// \param[in] _name The new name.
      public: void Name(const std::string &_name) const;

      //////////////
      /// Validation
      //////////////

      /// \return True if the zone is valid.
      public: bool Valid() const;

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<ZonePrivate> dataPtr;
    };
  }
}
#endif
