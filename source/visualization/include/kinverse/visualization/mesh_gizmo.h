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
#include <kinverse/core/mesh.h>

namespace kinverse {
  namespace visualization {

    /**
     * @class MeshGizmo
     * @brief This gizmo allows to draw meshes.
     * It allows to set up color and position of the given mesh.
     */
    class KINVERSE_VISUALIZATION_API MeshGizmo : public IGizmo {
     public:
      /**
       * @brief Smart pointer to @p MeshGizmo
       */
      using Ptr = std::shared_ptr<MeshGizmo>;

      /**
       * @brief Smart pointer to const @p MeshGizmo
       */
      using ConstPtr = std::shared_ptr<const MeshGizmo>;

      /**
       * @brief Simple constructor that allows to set mesh and its rendering parameters.
       * @param[in] parentGizmo - parent gizmo
       * @param[in] mesh - mesh to draw
       * @param[in] transform - transform of the mesh
       * @param[in] color - mesh color
       */
      explicit MeshGizmo(const IGizmo* parentGizmo = nullptr,
                         core::Mesh::ConstPtr mesh = nullptr,
                         const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                         const Color& color = { 255, 255, 255, 255 });

      /**
       * @brief Sets mesh that will be drawn.
       * @param[in] mesh - mesh to be drawn
       */
      void setMesh(core::Mesh::ConstPtr mesh);

      /**
       * @brief Returns mesh used for drawing
       */
      core::Mesh::ConstPtr getMesh() const;

      /**
       * @brief Sets up mesh transform.
       * @param[in] transform - desired mesh transform
       */
      void setTransform(const Eigen::Affine3d& transform);

      /**
       * @brief Returns mesh transform.
       */
      Eigen::Affine3d getTransform() const;

      /**
       * @brief Sets up mesh color.
       * @param[in] color - desired color of the mesh
       */
      void setColor(const Color& color);

      /**
       * @brief Returns mesh color.
       */
      Color getColor() const;

     private:
      /**
       * @brief Mesh used for rendering
       */
      core::Mesh::ConstPtr m_mesh{ nullptr };

      /**
       * @brief Stores mesh position and orientation.
       */
      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };

      /**
       * @brief Color to use when rendering mesh.
       */
      Color m_color{ 255, 255, 255, 255 };
    };

  }  // namespace visualization
}  // namespace kinverse
