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
#include "text_3d_gizmo.h"

namespace kinverse {
  namespace visualization {

    /**
     * @class CoordinateFrameGizmo
     * @brief This gizmo represents coordinate frame. By default axes lengths are 1000 mm.
     * Note that kinverse uses right handed coordinate frames.
     */
    class KINVERSE_VISUALIZATION_API CoordinateFrameGizmo : public IGizmo {
     public:
      /**
       * @brief Smart pointer to @p CoordinateFrameGizmo
       */
      using Ptr = std::shared_ptr<CoordinateFrameGizmo>;

      /**
       * @brief Smart pointer to const @p CoordinateFrameGizmo
       */
      using ConstPtr = std::shared_ptr<const CoordinateFrameGizmo>;

      /**
       * @brief Simple constructor that allows to set coordinate frame parameters.
       * @param[in] parentGizmo - parent gizmo
       * @param[in] transform - position and orientation of the coordinate frame
       * @param[in] caption - caption drawn near origin of the coordinate frame
       * @param[in] axesLength - this parameter is responsible for axes length (by default length is 1000 mm)
       * @param[in] axesLabels - array of strings used for labeling axes.
       */
      explicit CoordinateFrameGizmo(const IGizmo* parentGizmo = nullptr,
                                    const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                                    const std::string& caption = "",
                                    double axesLength = 1000.0,
                                    const std::array<std::string, 3>& axesLabels = { "X", "Y", "Z" });

      /**
       * @brief Sets caption for coordinate frame. It will be drawn near the coordinates frame origin.
       * @param[in] caption - coordinate frame caption
       */
      void setCaption(const std::string& caption) const;

      /**
       * @brief Returns coordinate frame caption.
       */
      std::string getCaption() const;

      /**
       * @brief Sets up coordinate frame transform.
       * @param[in] transform - desired coordinate frame transform
       */
      void setTransform(const Eigen::Affine3d& transform);

      /**
       * @brief Returns transform of the coordinate frame.
       */
      Eigen::Affine3d getTransform() const;

      /**
       * @brief Sets array of labels. These labels will be used for labeling coordinate frame axes.
       * @param[in] axesLabels - axes labels
       */
      void setAxesLabels(const std::array<std::string, 3>& axesLabels);

      /**
       * @brief Returns axes labels.
       */
      std::array<std::string, 3> getAxesLabels() const;

      /**
       * @brief Sets axes length, this is used when rendering coordinate frame.
       * @param[in] axesLength - this parameter is responsible for axes length
       */
      void setAxesLength(double axesLength);

      /**
       * @brief Returns axes length used when rendering coordinate frame.
       */
      double getAxesLength() const;

     protected:
      /**
       * @brief As soon as this is a compound gizmo, we need to override default @p show method.
       * This method simply tells how to render this complex gizmo.
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      void show(void* renderer) override;

     private:
      /**
       * @brief This is a helper method. It updates geometry transform each time axes length or coordinate frame transform changes.
       */
      void updateTransform();

      /**
       * @brief This method scales x, y and z axes labels.
       * @param[in] scale - scale of the axes labels
       */
      void scaleLabels(double scale) const;

      /**
       * @brief Stores labels for each axis of the coordinate frame.
       */
      std::array<std::string, 3> m_axesLabels{ "X", "Y", "Z" };

      /**
       * @brief Stores position and orientation of the coordinate frame.
       */
      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };

      /**
       * @brief Stores length of the axes.
       */
      double m_axesLength{ 1000.0 };

      /**
       * @brief Child gizmo for drawing coordinate frame caption.
       */
      Text3DGizmo::Ptr m_captionGizmo{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
