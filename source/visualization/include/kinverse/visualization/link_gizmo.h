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
#include "cylinder_gizmo.h"
#include <kinverse/core/denavit_hartenberg_parameters.h>

namespace kinverse {
  namespace visualization {

    /**
     * @class LinkGizmo
     * @brief This gizmo represents link between two joints. This link is drawn with to lines (cylinders):
     * displacement along Z axis and displacement along X axis.
     * This gizmo is very handy when drawing kinematic diagram of the robot.
     */
    class KINVERSE_VISUALIZATION_API LinkGizmo : public IGizmo {
     public:
      /**
       * @brief Smart pointer to @p LinkGizmo
       */
      using Ptr = std::shared_ptr<LinkGizmo>;

      /**
       * @brief Smart pointer to const @p LinkGizmo
       */
      using ConstPtr = std::shared_ptr<const LinkGizmo>;

      /**
       * @brief Simple constructor for settings up link between two joints.
       * As long as DH parameters describe transform between two joints,
       * we can use this information in order to assemble link between these joints.
       * @param[in] parentGizmo - parent gizmo
       * @param[in] dhParameters - Denavit-Hartenberg parameters
       * @param[in] transform - position and orientation of the link
       */
      explicit LinkGizmo(const IGizmo* parentGizmo = nullptr,
                         const core::DenavitHartenbergParameters& dhParameters = {},
                         const Eigen::Affine3d& transform = Eigen::Affine3d::Identity());

      /**
       * @brief Sets up link transform. Link gizmo represents transform between two joints
       * (initial and the next one). But in order to position this link in world space we still
       * need to know position of the initial joint (which depends on robot configuration).
       * @param[in] transform - desired link transform
       */
      void setTransform(const Eigen::Affine3d& transform);

      /**
       * @brief Returns transform of the link.
       */
      Eigen::Affine3d getTransform() const;

      /**
       * @brief Sets DH parameters that describe transform between two joints.
       * @param[in] dhParameters - Denavit-Hartenberg parameters
       */
      void setDHParameters(const core::DenavitHartenbergParameters& dhParameters);

      /**
       * @brief Returns DH parameters that describe transform between two joints.
       */
      core::DenavitHartenbergParameters getDHParameters() const;

     protected:
      /**
       * @brief LinkGizmo is compound gizmo, so we need to override default @p show method.
       * This method simply tells how to render this complex gizmo.
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      void show(void* renderer = nullptr) override;

      /**
       * @brief LinkGizmo is compound gizmo, so we need to override default @p hide method.
       * This method simply tells how to hide this complex gizmo.
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      void hide(void* renderer = nullptr) override;

     private:
      /**
       * @brief This is a helper method. It updates geometry transform each time transform or DH parameters changes.
       */
      void updateTransform() const;

      /**
       * @brief Stores position and orientation of the joint, the place it would be drawn.
       */
      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };

      /**
       * @brief Stores DH parameters that describe transform between two joints.
       */
      core::DenavitHartenbergParameters m_dhParameters{};

      /**
       * @brief Stores line (cylinder gizmo) that represents displacement along Z axis.
       */
      CylinderGizmo::Ptr m_zAxisDisplacementGizmo{ nullptr };

      /**
       * @brief Stores line (cylinder gizmo) that represents displacement along X axis.
       */
      CylinderGizmo::Ptr m_xAxisDisplacementGizmo{ nullptr };

      /**
       * @brief Stores transform that moves cylinder from the origin to link space.
       * In other words it places Z displacement line in link local space.
       */
      Eigen::Affine3d m_zInitialTransform{ Eigen::Affine3d::Identity() };

      /**
       * @brief Stores transform that moves cylinder from the origin to link space.
       * In other words it places X displacement line in link local space.
       */
      Eigen::Affine3d m_xInitialTransform{ Eigen::Affine3d::Identity() };
    };

  }  // namespace visualization
}  // namespace kinverse
