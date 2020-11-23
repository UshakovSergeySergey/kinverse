#pragma once

#include "exports.h"

namespace kinverse {
  namespace visualization {

    using Color = std::tuple<unsigned char, unsigned char, unsigned char, unsigned char>;

    class IGizmoImpl;

    class KINVERSE_VISUALIZATION_API IGizmo {
     public:
      using Ptr = std::shared_ptr<IGizmo>;
      using ConstPtr = std::shared_ptr<const IGizmo>;

      explicit IGizmo(const IGizmo* parentGizmo);
      virtual ~IGizmo() = default;

      virtual void draw(void* renderer);
      virtual void hide(void* renderer);
      void update();

     protected:
      std::shared_ptr<IGizmoImpl> m_pImpl{ nullptr };
      const IGizmo* m_parentGizmo{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
