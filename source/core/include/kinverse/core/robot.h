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
#include "denavit_hartenberg_parameters.h"
#include "joint_constraints.h"
#include "mesh.h"

namespace kinverse {
  namespace core {

    class KINVERSE_CORE_API Robot {
     public:
      /**
       * @brief Smart pointer to @p IRobot
       */
      using Ptr = std::shared_ptr<Robot>;

      /**
       * @brief Smart pointer to const @p IRobot
       */
      using ConstPtr = std::shared_ptr<const Robot>;

      /**
       * @brief Sets Denavit-Hartenberg table describing kinematic chain of the robot.
       * @param[in] dhTable - Denavit-Hartenberg table
       */
      void setDHTable(const std::vector<DenavitHartenbergParameters>& dhTable);

      /**
       * @brief Returns Denavit-Hartenberg table of the robot.
       */
      std::vector<DenavitHartenbergParameters> getDHTable() const;

      /**
       * @brief Sets robots axes configuration.
       * @param[in] configuration - axes configuration
       */
      void setConfiguration(const std::vector<double>& configuration);

      /**
       * @brief Returns current robot axes configuration.
       */
      std::vector<double> getConfiguration() const;

      void setJointConstraints(const std::vector<JointConstraints>& constraints);
      std::vector<JointConstraints> getJointConstraints() const;

      /**
       * For example KUKA KR5 Arc has its first axis pointing down, if you'll build kinematic diagram for KUKA you'll see
       * that the whole robot seems to be upside down (end effector has negative z values).
       * But in real life KUKA's end effector has positive z values. It is because base transform rotates robot about X axis 180 degrees
       * In order to fix this it comes handy to add some base transform.
       * Do not confuse base transform with robot transform.
       * Base transform tells how we need to rotate and translate robot (its kinematic diagram) in order to get robots local coordinate frame.
       * Whether robot transform tells how is robot positioned and oriented in the world.
       */
      void setBaseTransform(const Eigen::Affine3d& transform);
      Eigen::Affine3d getBaseTransform() const;

      void setTransform(const Eigen::Affine3d& transform);
      Eigen::Affine3d getTransform() const;

      void setMeshes(const std::vector<Mesh::ConstPtr>& meshes);
      std::vector<Mesh::ConstPtr> getMeshes() const;

      unsigned int getNumberOfJoints() const;
      unsigned int getNumberOfLinks() const;

      std::vector<Eigen::Affine3d> getJointCoordinateFrames() const;
      std::vector<Eigen::Affine3d> getLinkCoordinateFrames() const;

     private:
      std::vector<double> getAxisValues(const std::vector<double>& axisValues) const;

      Eigen::Affine3d m_baseTransform{ Eigen::Affine3d::Identity() };

      std::vector<DenavitHartenbergParameters> m_dhTable{};
      std::vector<JointConstraints> m_constraints{};
      std::vector<double> m_configuration{};
      std::vector<Mesh::ConstPtr> m_meshes{};
    };

  }  // namespace core
}  // namespace kinverse
