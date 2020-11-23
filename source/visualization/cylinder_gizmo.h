#pragma once

#include "exports.h"
#include "i_gizmo.h"

namespace kinverse {
  namespace visualization {

    class KINVERSE_VISUALIZATION_API CylinderGizmo : public IGizmo {
     public:
      using Ptr = std::shared_ptr<CylinderGizmo>;
      using ConstPtr = std::shared_ptr<const CylinderGizmo>;

      explicit CylinderGizmo(const IGizmo* parentGizmo = nullptr,
                             const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                             double radius = 37.5,
                             double height = 150.0,
                             const Color& color = { 255, 255, 255, 255 });
      virtual ~CylinderGizmo() = default;

      void setTransform(const Eigen::Affine3d& transform);
      Eigen::Affine3d getTransform() const;

      void setRadius(double radius);
      double getRadius() const;

      void setHeight(double height);
      double getHeight() const;

      void setColor(const Color& color);
      Color getColor() const;

     private:
      void updateGeometry();

      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };
      double m_radius{ 37.5 };
      double m_height{ 150.0 };
      Color m_color{ 255, 255, 255, 255 };
    };

  }  // namespace visualization
}  // namespace kinverse
