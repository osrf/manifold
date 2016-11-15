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

#ifndef MANIFOLD_RNDF_EXIT_HH_
#define MANIFOLD_RNDF_EXIT_HH_

#include "manifold/Helpers.hh"
#include "manifold/rndf/UniqueId.hh"

namespace manifold
{
  namespace rndf
  {
    /// \brief An exit clas that shows how to go from an exit waypoint to
    /// an entry waypoint. The waypoints are represented with their unique Id.
    class MANIFOLD_VISIBLE Exit
    {
      public: Exit();

      /// \brief Constructor.
      /// \param[in] _exit The unique ID of the exit waypoint.
      /// \param[in] _entry The unique ID of the entry waypoint.
      /// \sa Valid.
      public: explicit Exit(const UniqueId &_exit,
                            const UniqueId &_entry);

      /// \brief Copy constructor.
      /// \param[in] _other Other Exit.
      public: Exit(const Exit &_other);

      /// \brief Destructor.
      public: virtual ~Exit();

      /// \brief ToDo.
      public: bool Parse(std::ifstream &_rndfFile,
                         const int major,
                         const int minor,
                         rndf::Exit &_exit,
                         int &_lineNumber);

      /// \brief Get the unique Id of the exit waypoint.
      /// \return The unique Id of the exit waypoint.
      public: const UniqueId &ExitId() const;

      /// \brief Get a mutable reference to the unique Id of the exit waypoint.
      /// \return A mutable reference to the unique Id of the exit waypoint.
      public: UniqueId &ExitId();

      /// \brief Get the unique Id of the entry waypoint.
      /// \return The unique Id of the entry waypoint.
      public: const UniqueId &EntryId() const;

      /// \brief Get a mutable reference to the unique Id of the entry waypoint.
      /// \return A mutable reference to the unique Id of the entry waypoint.
      public: UniqueId &EntryId();

      /// \return True if the exit is valid.
      public: bool Valid() const;

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other Exit to check for equality
      /// \return true if this == _other
      public: bool operator==(const Exit &_other) const;

      /// \brief Inequality.
      /// \param[in] _other Exit to check for inequality.
      /// \return true if this != _other
      public: bool operator!=(const Exit &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new Exit.
      /// \return A reference to this instance.
      public: Exit &operator=(const Exit &_other);

      /// \brief The unique Id of the exit waypoint.
      private: UniqueId exit;

      /// \brief The unique Id of the entry waypoint.
      private: UniqueId entry;
    };
  }
}
#endif
