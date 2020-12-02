#pragma once

#include "exports.h"
#include "i_gizmo.h"
#include "cylinder_gizmo.h"
#include "coordinate_frame_gizmo.h"

namespace kinverse {
  namespace visualization {

    class KINVERSE_VISUALIZATION_API RevoluteJointGizmo : public IGizmo {
     public:
      using Ptr = std::shared_ptr<RevoluteJointGizmo>;
      using ConstPtr = std::shared_ptr<const RevoluteJointGizmo>;

      explicit RevoluteJointGizmo(const IGizmo* parentGizmo = nullptr,
                                  const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                                  unsigned int jointIndex = 0);
      virtual ~RevoluteJointGizmo() = default;

      void setTransform(const Eigen::Affine3d& transform);
      Eigen::Affine3d getTransform() const;

      void setJointIndex(unsigned int jointIndex);
      unsigned int getJointIndex() const;

      void show(void* renderer) override;

     private:
      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };
      unsigned int m_jointIndex{ 0 };

      CylinderGizmo::Ptr m_cylinder{ nullptr };
      CoordinateFrameGizmo::Ptr m_coordinateFrame{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
