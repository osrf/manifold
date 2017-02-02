/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef MANIFOLD_OPENDRIVE_IMPORTER_HH_
#define MANIFOLD_OPENDRIVE_IMPORTER_HH_

#include <memory>
#include <string>

#include "manifold/Helpers.hh"

namespace manifold
{
  namespace opendrive
  {
    // Forward declarations.
    class ImporterPrivate;

    /// \brief A class to import an OpenDrive file and load it in memory.
    /// Please, refer to the specification for more details.
    /// \reference http://www.opendrive.org/download.html
    class MANIFOLD_VISIBLE Importer
    {
      /// \brief Default constructor.
      public: Importer();

      /// \brief Constructor.
      /// \param[in] _filepath Path to an existing xodr (openDRIVE) file.
      public: explicit Importer(const std::string &_filepath);

      /// \brief Destructor.
      public: virtual ~Importer();

      ///////////
      /// Parsing
      ///////////

      /// \brief Load a xodr file from an input stream coming from a text file.
      /// The expected format is the one specified on the OpenDrive spec.
      /// \param[in, out] _filePath Path to xodr file.
      /// \return True if the entire xodr file was correctly parsed or false
      /// otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(const std::string &_filePath);

      //////////////
      /// Validation
      //////////////

      /// \brief Whether the current OpenDRIVE object is valid or not.
      /// \return True if the OpenDRIVE representation is valid.
      public: bool Valid() const;

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<ImporterPrivate> dataPtr;
    };
  }
}
#endif
