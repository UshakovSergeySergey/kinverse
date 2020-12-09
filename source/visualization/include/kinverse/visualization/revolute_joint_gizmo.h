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
#include "coordinate_frame_gizmo.h"

namespace kinverse {
  namespace visualization {

    /**
     * @class RevoluteJointGizmo
     * @brief This gizmo is used for rendering revolute joints.
     * It comes handy when you want to draw kinematic diagram of the robot.
     */
    class KINVERSE_VISUALIZATION_API RevoluteJointGizmo : public IGizmo {
     public:
      /**
       * @brief Smart pointer to @p RevoluteJointGizmo
       */
      using Ptr = std::shared_ptr<RevoluteJointGizmo>;

      /**
       * @brief Smart pointer to const @p RevoluteJointGizmo
       */
      using ConstPtr = std::shared_ptr<const RevoluteJointGizmo>;

      /**
       * @brief Simple constructor that allows to set joint parameters.
       * @param[in] parentGizmo - parent gizmo
       * @param[in] transform - position and orientation of the joint
       * @param[in] jointIndex - this index is used for labeling. Each joint has label in the format 'frame' + jointIndex (e.g. frame1)
       */
      explicit RevoluteJointGizmo(const IGizmo* parentGizmo = nullptr,
                                  const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                                  unsigned int jointIndex = 0);

      /**
       * @brief Sets up joints transform.
       * @param[in] transform - desired joint transform
       */
      void setTransform(const Eigen::Affine3d& transform);

      /**
       * @brief Returns transform of the revolute joint.
       */
      Eigen::Affine3d getTransform() const;

      /**
       * @brief Sets joint index. This index is used for labeling joint in the format 'frame' + jointIndex (e.g. frame5).
       * @param[in] jointIndex - joint index
       */
      void setJointIndex(unsigned int jointIndex);

      /**
       * @brief Returns joint index used for labeling
       */
      unsigned int getJointIndex() const;

     protected:
      /**
       * @brief Revolute joint is compound gizmo, so we need to override default @p show method.
       * This method simply tells how to render this complex gizmo.
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      void show(void* renderer) override;

     private:
      /**
       * @brief Stores position and orientation of the revolute joint, the place it would be drawn.
       */
      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };

      /**
       * @brief Stores joint index which is used for labeling.
       */
      unsigned int m_jointIndex{ 0 };

      /**
       * @brief Revolute joints usually drawn with cylinders on the kinematic diagrams.
       * So we store a cylinder here.
       */
      CylinderGizmo::Ptr m_cylinder{ nullptr };

      /**
       * @brief Stores coordinate frame of a joint. Z axis is the axis of rotation.
       */
      CoordinateFrameGizmo::Ptr m_coordinateFrame{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
