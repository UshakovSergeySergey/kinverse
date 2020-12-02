#pragma once

#include "exports.h"
#include "i_gizmo.h"
#include "text_3d_gizmo.h"

namespace kinverse {
  namespace visualization {

    // by default axes lengths are 1000 mm
    class KINVERSE_VISUALIZATION_API CoordinateFrameGizmo : public IGizmo {
     public:
      using Ptr = std::shared_ptr<CoordinateFrameGizmo>;
      using ConstPtr = std::shared_ptr<const CoordinateFrameGizmo>;

      explicit CoordinateFrameGizmo(const IGizmo* parentGizmo = nullptr,
                                    const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                                    const std::string& caption = "",
                                    double scale = 1.0,
                                    const std::array<std::string, 3>& axesLabels = { "X", "Y", "Z" });
      virtual ~CoordinateFrameGizmo() = default;

      void setCaption(const std::string& caption);
      std::string getCaption() const;

      void setTransform(const Eigen::Affine3d& transform);
      Eigen::Affine3d getTransform() const;

      void setAxesLabels(const std::array<std::string, 3>& axesLabels);
      std::array<std::string, 3> getAxesLabels() const;

      void setScale(double scale);
      double getScale() const;

      void show(void* renderer) override;

     private:
      void updateTransform();
      void scaleLabels(double scale);

      std::array<std::string, 3> m_axesLabels{ "X", "Y", "Z" };
      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };
      double m_scale{ 1.0 };
      Text3DGizmo::Ptr m_captionGizmo{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
