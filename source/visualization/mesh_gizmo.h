#pragma once

#include "exports.h"
#include "i_gizmo.h"
#include <core/mesh.h>

namespace kinverse {
  namespace visualization {

    class KINVERSE_VISUALIZATION_API MeshGizmo : public IGizmo {
     public:
      using Ptr = std::shared_ptr<MeshGizmo>;
      using ConstPtr = std::shared_ptr<const MeshGizmo>;

      explicit MeshGizmo(const IGizmo* parentGizmo = nullptr,
                         core::Mesh::ConstPtr mesh = nullptr,
                         const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                         const Color& color = { 255, 255, 255, 255 });
      virtual ~MeshGizmo() = default;

      void setMesh(core::Mesh::ConstPtr mesh);
      core::Mesh::ConstPtr getMesh() const;

      void setTransform(const Eigen::Affine3d& transform);
      Eigen::Affine3d getTransform() const;

      void setColor(const Color& color);
      Color getColor() const;

     private:
      core::Mesh::ConstPtr m_mesh{ nullptr };
      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };
      Color m_color{ 255, 255, 255, 255 };
    };

  }  // namespace visualization
}  // namespace kinverse
