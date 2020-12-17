/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Sergey Ushakov
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "exports.h"
#include "i_gizmo.h"

namespace kinverse {
  namespace visualization {

    /**
     * @class Text3DGizmo
     * @brief This gizmo allows to draw text in 3d space.
     * This gizmo also allows to make text to have constant scale
     * relative to viewport, or face text towards camera.
     */
    class KINVERSE_VISUALIZATION_API Text3DGizmo : public IGizmo {
     public:
      /**
       * @brief Smart pointer to @p Text3DGizmo
       */
      using Ptr = std::shared_ptr<Text3DGizmo>;

      /**
       * @brief Smart pointer to const @p Text3DGizmo
       */
      using ConstPtr = std::shared_ptr<const Text3DGizmo>;

      /**
       * @brief Simple constructor that allows to set text and its properties.
       * @param[in] parentGizmo - parent gizmo
       * @param[in] transform - position and orientation of the text
       * @param[in] text - text to display
       * @param[in] fontSize - font size
       * @param[in] color - text color
       * @param[in] faceTowardsCamera - if set to true, then text will always face towards camera
       * @param[in] viewportConstScale - if set to true, then text will always has constant scale relative to viewport
       */
      explicit Text3DGizmo(const IGizmo* parentGizmo = nullptr,
                           const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                           const std::string& text = "Text3DGizmo",
                           int fontSize = 100,
                           const Color& color = Color{ 255, 255, 255, 255 },
                           bool faceTowardsCamera = true,
                           bool viewportConstScale = true);

      /**
       * @brief Sets text position and orientation. If @p faceTowardsCamera set to true,
       * then only text position is taken into account.
       * @param[in] transform - desired mesh transform
       */
      void setTransform(const Eigen::Affine3d& transform);

      /**
       * @brief Returns text transform.
       */
      Eigen::Affine3d getTransform() const;

      /**
       * @brief Sets text that need to be displayed.
       * @param[in] text - text to display
       */
      void setText(const std::string& text);

      /**
       * @brief Returns text that is displayed with gizmo.
       */
      std::string getText() const;

      /**
       * @brief Sets font size. This parameter affects quality of the rendered text.
       * @param[in] fontSize - font size
       */
      void setFontSize(int fontSize);

      /**
       * @brief Returns font size.
       */
      int getFontSize() const;

      /**
       * @brief Sets up text color.
       * @param[in] color - desired color of the text
       */
      void setColor(const Color& color);

      /**
       * @brief Returns text color.
       */
      Color getColor() const;

      /**
       * @brief This method sets the flag that tells whether text must always face towards camera.
       * @param[in] faceTowardsCamera - if set to true, then text will always face towards camera
       */
      void setFaceTowardsCamera(bool faceTowardsCamera);

      /**
       * @brief Returns flag that tells if text is always facing towards camera.
       */
      bool getFaceTowardsCamera() const;

      /**
       * @brief This method sets the flag that tells whether text must always have constant scale relative to viewport.
       * @param[in] viewportConstScale - if set to true, then text will always have constant scale relative to viewport
       */
      void setViewportConstScale(bool viewportConstScale);

      /**
       * @brief Returns flag that tells if text is always have constant scale
       */
      bool getViewportConstScale() const;

     protected:
      /**
       * @brief In order to implement faceTowardsCamera and viewportConstScale functionality
       * Text3DGizmo needs subscription for camera events.
       * Here we are overriding default @p show method in order to make subscription.
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      void show(void* renderer) override;

      /**
       * @brief In order to implement faceTowardsCamera and viewportConstScale functionality
       * Text3DGizmo needs subscription for camera events.
       * Here we are overriding default @p hide method in order to make unsubscription.
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      void hide(void* renderer) override;

     private:
      /**
       * @brief This is a helper method that updates subscription for camera events.
       * It subscribes/unsubscribes for camera events if needed.
       */
      void updateSubscriptionForCameraEvents();

      /**
       * @brief This helper method computes text transform taking into account
       * @p m_faceTowardsCamera and @p m_viewportConstScale parameters.
       * @param[in] needsToBeRerendered - tells if need to initiate render call after computation complete
       */
      void updateTextOrientationAndScale(bool needsToBeRerendered);

      /**
       * @brief This helper method acquires camera for position and orientation.
       * @param[in] cameraPosition - camera position
       * @param[in] cameraUpVector - camera orientation represented with camera up vector
       */
      bool getCameraPositionAndOrientation(Eigen::Vector3d& cameraPosition, Eigen::Vector3d& cameraUpVector) const;

      /**
       * @brief This method applies given transform and initiates render call if needed.
       * @param[in] transform - text transform to apply
       * @param[in] needsToBeRerendered - tells if need to initiate render call
       */
      void updateTransform(const Eigen::Affine3d& transform, bool needsToBeRerendered) const;

      /**
       * @brief Text to display.
       */
      std::string m_text{ "Text3DGizmo" };

      /**
       * @brief Stores position and orientation of the text.
       */
      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };

      /**
       * @brief Color to use when rendering text.
       */
      Color m_color{ 255, 255, 255, 255 };

      /**
       * @brief Font size of the text.
       */
      int m_fontSize{ 100 };

      /**
       * @brief This flag tells whether text must always face towards camera.
       */
      bool m_faceTowardsCamera{ true };

      /**
       * @brief This flag tells whether text must always have constant scale relative to viewport.
       */
      bool m_viewportConstScale{ true };
    };

  }  // namespace visualization
}  // namespace kinverse
