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
#include "joint_gizmo.h"
#include "link_gizmo.h"
#include <kinverse/core/robot.h>

namespace kinverse {
  namespace visualization {

    /**
     * @class KinematicDiagramGizmo
     * @brief This class allows to easily visualize kinematic diagram of a robot.
     */
    class KINVERSE_VISUALIZATION_API KinematicDiagramGizmo : public IGizmo {
     public:
      /**
       * @brief Smart pointer to @p KinematicDiagramGizmo
       */
      using Ptr = std::shared_ptr<KinematicDiagramGizmo>;

      /**
       * @brief Smart pointer to const @p KinematicDiagramGizmo
       */
      using ConstPtr = std::shared_ptr<const KinematicDiagramGizmo>;

      /**
       * @brief Simple constructor that allows to set robot for which
       * kinematic diagram needs to be drawn.
       * @param[in] parentGizmo - parent gizmo
       * @param[in] robot - robot
       */
      explicit KinematicDiagramGizmo(const IGizmo* parentGizmo = nullptr, core::Robot::ConstPtr robot = nullptr);

      /**
       * @brief Sets robot for which kinematic diagram must be drawn.
       * @param[in] robot - robot
       */
      void setRobot(core::Robot::ConstPtr robot);

      /**
       * @brief Returns robot for which kinematic diagram is drawn.
       */
      core::Robot::ConstPtr getRobot() const;

      /**
       * @brief If robot changes its configuration, kinematic diagram is not updated immediately.
       * In order to update diagram users must call this method. It extracts configuration from
       * the provided robot and updates kinematic diagram configuration.
       */
      void updateRobotConfiguration();

     protected:
      /**
       * @brief KinematicDiagramGizmo is compound gizmo, so we need to override default @p show method.
       * This method simply tells how to render this complex gizmo.
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      void show(void* renderer = nullptr) override;

      /**
       * @brief KinematicDiagramGizmo is compound gizmo, so we need to override default @p hide method.
       * This method simply tells how to hide this complex gizmo.
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      void hide(void* renderer = nullptr) override;

     private:
      /**
       * @brief This is a helper method which updates kinematic diagram whenever user calls @p setRobot method.
       */
      void updateRobotStructure();

      /**
       * @brief Stores robot for which kinematic diagram is drawn.
       */
      core::Robot::ConstPtr m_robot{ nullptr };

      /**
       * @brief Stores gizmos which represent joints on the kinematic diagram.
       */
      std::vector<JointGizmo::Ptr> m_jointGizmos{};

      /**
       * @brief Stores gizmos which represent links between joints.
       * Links are drawn as lines (thick cylinders).
       */
      std::vector<LinkGizmo::Ptr> m_linkGizmos{};

      /**
       * @brief Stores coordinate frame gizmo of the end effector.
       */
      CoordinateFrameGizmo::Ptr m_endEffectorGizmo{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
