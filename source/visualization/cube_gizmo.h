#pragma once

#include "exports.h"
#include "i_gizmo.h"

namespace kinverse {
  namespace visualization {

    class KINVERSE_VISUALIZATION_API CubeGizmo : public IGizmo {
     public:
      using Ptr = std::shared_ptr<CubeGizmo>;
      using ConstPtr = std::shared_ptr<const CubeGizmo>;

      explicit CubeGizmo(const IGizmo* parentGizmo = nullptr,
                         const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                         double width = 300.0,
                         double height = 300.0,
                         double depth = 300.0,
                         const Color& color = { 255, 255, 255, 255 });
      virtual ~CubeGizmo() = default;

      void setTransform(const Eigen::Affine3d& transform);
      Eigen::Affine3d getTransform() const;

      void setColor(const Color& color);
      Color getColor() const;

      void setWidth(double width);
      double getWidth() const;

      void setDepth(double depth);
      double getDepth() const;

      void setHeight(double height);
      double getHeight() const;

     private:
      void updateGeometry();

      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };
      Color m_color{ 255, 255, 255, 255 };
      double m_width{ 300.0 };
      double m_depth{ 300.0 };
      double m_height{ 300.0 };
    };

  }  // namespace visualization
}  // namespace kinverse
