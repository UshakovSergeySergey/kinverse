#pragma once

#include "exports.h"
#include "i_gizmo.h"

namespace kinverse {
  namespace visualization {

    class KINVERSE_VISUALIZATION_API Text3DGizmo : public IGizmo {
     public:
      using Ptr = std::shared_ptr<Text3DGizmo>;
      using ConstPtr = std::shared_ptr<const Text3DGizmo>;

      explicit Text3DGizmo(const IGizmo* parentGizmo = nullptr,
                           const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                           const std::string& text = "Text3DGizmo",
                           int fontSize = 100.0,
                           const Color& color = Color{ 255, 255, 255, 255 },
                           bool faceTowardsCamera = true,
                           bool viewportConstScale = true);
      virtual ~Text3DGizmo() = default;

      void setTransform(const Eigen::Affine3d& transform);
      Eigen::Affine3d getTransform() const;

      void setText(const std::string& text);
      std::string getText() const;

      void setFontSize(int fontSize);
      int getFontSize() const;

      void setColor(const Color& color);
      Color getColor() const;

      void setFaceTowardsCamera(bool faceTowardsCamera);
      bool getFaceTowardsCamera() const;

      void setViewportConstScale(bool viewportConstScale);
      bool getViewportConstScale() const;

      void show(void* renderer) override;

     private:
      void updateSubscriptionForCameraEvents();
      void updateTextOrientationAndScale(bool needsToBeRerendered);

      bool getCameraPositionAndOrientation(Eigen::Vector3d& cameraPosition, Eigen::Vector3d& cameraUpVector) const;

      // these methods only update 3d view
      // rerender only if setTransform. if this method called from subscription, then there is no need to rerender, because the caller will do everything
      void updateTransform(const Eigen::Affine3d& transform, bool needsToBeRerendered);

      std::string m_text;
      Eigen::Affine3d m_transform;
      Color m_color;
      int m_fontSize;
      bool m_faceTowardsCamera;
      bool m_viewportConstScale;
    };

  }  // namespace visualization
}  // namespace kinverse
