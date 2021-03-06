﻿/**
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
#include "mesh_gizmo.h"
#include "coordinate_frame_gizmo.h"
#include <kinverse/core/robot.h>

namespace kinverse {
  namespace visualization {

    class KINVERSE_VISUALIZATION_API RobotGizmo : public IGizmo {
     public:
      using Ptr = std::shared_ptr<RobotGizmo>;
      using ConstPtr = std::shared_ptr<const RobotGizmo>;

      explicit RobotGizmo(const IGizmo* parentGizmo = nullptr, core::Robot::ConstPtr robot = nullptr);

      void setRobot(core::Robot::ConstPtr robot);
      core::Robot::ConstPtr getRobot() const;

      void updateRobotConfiguration();

     protected:
      void show(void* renderer = nullptr) override;
      void hide(void* renderer = nullptr) override;

     private:
      void updateRobotStructure();

      core::Robot::ConstPtr m_robot{ nullptr };
      std::vector<core::Mesh::ConstPtr> m_meshes{};
      std::vector<MeshGizmo::Ptr> m_meshGizmos{};
      CoordinateFrameGizmo::Ptr m_endEffectorGizmo{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
