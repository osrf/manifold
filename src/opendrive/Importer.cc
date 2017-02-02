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

#include <fstream>
#include <iostream>
#include <string>
#include <tinyxml2.h>

#include "manifold/opendrive/Importer.hh"

using namespace manifold;
using namespace opendrive;

namespace manifold
{
  namespace opendrive
  {
    /// \internal
    /// \brief Private data for Importer class.
    class ImporterPrivate
    {
      /// \brief Constructor.
      public: ImporterPrivate() = default;

      /// \brief Destructor.
      public: virtual ~ImporterPrivate() = default;

      /// \brief The XML .xodr file.
      public: tinyxml2::XMLDocument xmlDoc;
    };
  }
}

//////////////////////////////////////////////////
Importer::Importer()
  : dataPtr(new ImporterPrivate())
{
}

//////////////////////////////////////////////////
Importer::Importer(const std::string &_filepath)
  : Importer()
{
  this->Load(_filepath);
}

//////////////////////////////////////////////////
Importer::~Importer()
{
}

//////////////////////////////////////////////////
bool Importer::Load(const std::string &_filePath)
{
  // Flag use to indicate if a parser failure has occurred
  if (this->dataPtr->xmlDoc.LoadFile(_filePath.c_str()) !=
      tinyxml2::XML_SUCCESS)
  {
    std::cerr << "Error opening OpenDrive [" << _filePath << "]" << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool Importer::Valid() const
{
  return true;
}
