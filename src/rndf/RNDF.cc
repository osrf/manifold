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

#include "manifold/rndf/RNDF.hh"

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
    };
  }
}

//////////////////////////////////////////////////
RNDF::RNDF()
  : dataPtr(new RNDFPrivate())
{
}