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
     * @class CylinderGizmo
     * @brief This gizmo allows to easily draw a cylinder.
     * It allows to set up cylinders radius, height, color and position.
     */
    class KINVERSE_VISUALIZATION_API CylinderGizmo : public IGizmo {
     public:
      /**
       * @brief Smart pointer to @p CylinderGizmo
       */
      using Ptr = std::shared_ptr<CylinderGizmo>;

      /**
       * @brief Smart pointer to const @p CylinderGizmo
       */
      using ConstPtr = std::shared_ptr<const CylinderGizmo>;

      /**
       * @brief Simple constructor that allows to set cylinders parameters.
       * @param[in] parentGizmo - parent gizmo
       * @param[in] transform - cylinders transform
       * @param[in] radius - radius of the cylinder
       * @param[in] height - height of the cylinder
       * @param[in] color - cylinders color
       */
      explicit CylinderGizmo(const IGizmo* parentGizmo = nullptr,
                             const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                             double radius = 31.25,
                             double height = 125.0,
                             const Color& color = { 255, 255, 255, 255 });

      /**
       * @brief Sets up cylinders transform.
       * @param[in] transform - desired cylinders transform
       */
      void setTransform(const Eigen::Affine3d& transform);

      /**
       * @brief Returns cylinders transform.
       */
      Eigen::Affine3d getTransform() const;

      /**
       * @brief Sets up cylinders color.
       * @param[in] color - desired color of the cylinder
       */
      void setColor(const Color& color);

      /**
       * @brief Returns cylinders color.
       */
      Color getColor() const;

      /**
       * @brief Allows to set cylinders radius.
       * @param[in] radius - desired radius of the cylinder
       */
      void setRadius(double radius);

      /**
       * @brief Returns radius of the cylinder.
       */
      double getRadius() const;

      /**
       * @brief Allows to set cylinders height.
       * @param[in] height - desired height of the cylinder
       */
      void setHeight(double height);

      /**
       * @brief Returns height of the cylinder.
       */
      double getHeight() const;

     private:
      /**
       * @brief This is a helper method. It updates geometry each time cylinders radius or height changes.
       */
      void updateGeometry() const;

      /**
       * @brief Stores cylinders position and orientation.
       */
      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };

      /**
       * @brief Color to use when rendering cylinder.
       */
      Color m_color{ 255, 255, 255, 255 };

      /**
       * @brief Stores cylinders radius.
       */
      double m_radius{ 31.25 };

      /**
       * @brief Stores cylinders height.
       */
      double m_height{ 125.0 };
    };

  }  // namespace visualization
}  // namespace kinverse
