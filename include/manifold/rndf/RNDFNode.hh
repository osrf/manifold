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

#ifndef MANIFOLD_RNDF_RNDFNODE_HH_
#define MANIFOLD_RNDF_RNDFNODE_HH_

#include <memory>

#include "manifold/Helpers.hh"

namespace manifold
{
  namespace rndf
  {
    // Forward declarations.
    class Lane;
    class RNDFNodePrivate;
    class Segment;
    class UniqueId;
    class Zone;

    // \internal
    /// \brief An RNDF node class. Stores all the information associated with a
    /// given a unique Id .
    class MANIFOLD_VISIBLE RNDFNode
    {
      /// \brief Default constructor.
      /// \sa Valid.
      public: RNDFNode();

      /// \brief Default constructor.
      /// \param[in] _id A unique Id.
      public: RNDFNode(const rndf::UniqueId &_id);

      /// \brief Copy constructor.
      /// \param[in] _other Other RNDFNode.
      public: explicit RNDFNode(const RNDFNode &_other);

      /// \brief Destructor.
      public: virtual ~RNDFNode();

      public: rndf::UniqueId &UniqueId() const;

      public: rndf::Segment *Segment() const;

      public: rndf::Lane *Lane() const;

      public: rndf::Zone *Zone() const;

      public: void SetUniqueId(const rndf::UniqueId &_id);

      public: void SetSegment(rndf::Segment *_segment);

      public: void SetLane(rndf::Lane *_lane);

      public: void SetZone(rndf::Zone *_zone);

      /////////////
      /// Operators
      /////////////

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other RNDFNode to check for equality.
      /// \return true if this == _other
      public: bool operator==(const RNDFNode &_other) const;

      /// \brief Inequality.
      /// \param[in] _other RNDFNode to check for inequality.
      /// \return true if this != _other
      public: bool operator!=(const RNDFNode &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new RNDFNode.
      /// \return A reference to this instance.
      public: RNDFNode &operator=(const RNDFNode &_other);

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<RNDFNodePrivate> dataPtr;
    };
  }
}
#endif
