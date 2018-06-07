/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef IGNITION_RENDERING_BASE_BASEOBJECT_HH_
#define IGNITION_RENDERING_BASE_BASEOBJECT_HH_

#include <string>
#include "ignition/rendering/Object.hh"

namespace ignition
{
  namespace rendering
  {
    class IGNITION_RENDERING_VISIBLE BaseObject :
      public virtual std::enable_shared_from_this<BaseObject>,
      public virtual Object
    {
      protected: BaseObject();

      public: virtual ~BaseObject();

      public: virtual unsigned int Id() const;

      public: virtual std::string Name() const;

      // Documentation inherited
      public: virtual void PreRender();

      // Documentation inherited
      public: virtual void PostRender();

      public: virtual void Destroy();

              // TODO: make pure virtual
      protected: virtual void Load();

              // TODO: make pure virtual
      protected: virtual void Init();

      protected: unsigned int id;

      protected: std::string name;
    };
  }
}
#endif
