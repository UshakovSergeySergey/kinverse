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
     * @class CubeGizmo
     * @brief This gizmo allows to easily draw a cube.
     * Actually it's not a cube, it is a parallelepiped, but it is
     * much easier to write and pronounce CubeGizmo instead of ParallelepipedGizmo.
     * It also allows to set up cubes width, height, depth, color and position.
     */
    class KINVERSE_VISUALIZATION_API CubeGizmo : public IGizmo {
     public:
      /**
       * @brief Smart pointer to @p CubeGizmo
       */
      using Ptr = std::shared_ptr<CubeGizmo>;

      /**
       * @brief Smart pointer to const @p CubeGizmo
       */
      using ConstPtr = std::shared_ptr<const CubeGizmo>;

      /**
       * @brief Simple constructor that allows to set cubes parameters.
       * @param[in] parentGizmo - parent gizmo
       * @param[in] transform - cubes transform
       * @param[in] width - width of the cube
       * @param[in] height - height of the cube
       * @param[in] depth - cubes depth
       * @param[in] color - cubes color
       */
      explicit CubeGizmo(const IGizmo* parentGizmo = nullptr,
                         const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                         double width = 100.0,
                         double height = 100.0,
                         double depth = 100.0,
                         const Color& color = { 255, 255, 255, 255 });

      /**
       * @brief Sets up cubes transform.
       * @param[in] transform - desired cubes transform
       */
      void setTransform(const Eigen::Affine3d& transform);

      /**
       * @brief Returns cubes transform.
       */
      Eigen::Affine3d getTransform() const;

      /**
       * @brief Sets up color of the cube.
       * @param[in] color - desired color of the cube
       */
      void setColor(const Color& color);

      /**
       * @brief Returns cubes color.
       */
      Color getColor() const;

      /**
       * @brief Allows to set cubes width.
       * @param[in] width - desired width of the cube
       */
      void setWidth(double width);

      /**
       * @brief Returns width of the cube.
       */
      double getWidth() const;

      /**
       * @brief Allows to set cubes depth.
       * @param[in] depth - desired depth of the cube
       */
      void setDepth(double depth);

      /**
       * @brief Returns depth of the cube.
       */
      double getDepth() const;

      /**
       * @brief Allows to set cubes height.
       * @param[in] height - desired height of the cube
       */
      void setHeight(double height);

      /**
       * @brief Returns height of the cube.
       */
      double getHeight() const;

     private:
      /**
       * @brief This is a helper method. It updates geometry each time cubes width, height or depth changes.
       */
      void updateGeometry() const;

      /**
       * @brief Stores cubes position and orientation.
       */
      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };

      /**
       * @brief Color to use when rendering cube.
       */
      Color m_color{ 255, 255, 255, 255 };

      /**
       * @brief Stores cubes width.
       */
      double m_width{ 300.0 };

      /**
       * @brief Stores cubes depth.
       */
      double m_depth{ 300.0 };

      /**
       * @brief Stores cubes height.
       */
      double m_height{ 300.0 };
    };

  }  // namespace visualization
}  // namespace kinverse
