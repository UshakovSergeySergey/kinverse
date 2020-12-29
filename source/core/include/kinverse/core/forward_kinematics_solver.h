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
#include "robot.h"

namespace kinverse {
  namespace core {

    /**
     * @class ForwardKinematicsSolver
     * @brief This class solves end effector position for the given
     * robot and its configuration.
     */
    class KINVERSE_CORE_API ForwardKinematicsSolver {
     public:
      /**
       * @brief Smart pointer to @p ForwardKinematicsSolver
       */
      using Ptr = std::shared_ptr<ForwardKinematicsSolver>;

      /**
       * @brief Smart pointer to const @p ForwardKinematicsSolver
       */
      using ConstPtr = std::shared_ptr<const ForwardKinematicsSolver>;

      /**
       * @brief Simple constructor.
       * @param[in] robot - robot for FK solving
       */
      ForwardKinematicsSolver(Robot::ConstPtr robot = nullptr);

      /**
       * @brief Sets robot for which FK must be solved.
       * @param[in] robot - robot for FK solving
       */
      void setRobot(Robot::ConstPtr robot);

      /**
       * @brief Returns robot for which FK is solved.
       */
      Robot::ConstPtr getRobot() const;

      /**
       * @brief This method solves inverse kinematics. Note that if robot is nullptr,
       * configuration contains NaNs or if number of values in configuration doesn't match
       * number of robots joints, then this method will throw domain error. This method also
       * stores intermediate transforms which can be obtained using @p getCoordinateFrames method.
       * @param[in] configuration - robots joint configuration for which forward kinematics must be solved
       * @return Returns end effector transform.
       */
      Eigen::Affine3d solve(const Eigen::VectorXd& configuration);

      /**
       * @brief After @p solve method is called, this method will return intermediate results.
       * It returns list of transforms from local to world coordinate space of the next coordinate frames
       * {BASE, A1, ..., AN, FLANGE, TCP}
       */
      std::vector<Eigen::Affine3d> getLinkCoordinateFrames() const;

     private:
      /**
       * @brief Stores robot for which forward kinematics is solved.
       */
      Robot::ConstPtr m_robot{ nullptr };

      /**
       * @brief Stores intermediate transforms obtained during finding solution for forward kinematics.
       * Array contains transforms from local to world coordinate space of the next coordinate frames
       * {BASE, A1, ..., AN, FLANGE, TCP}
       */
      std::vector<Eigen::Affine3d> m_linkCoordinateFrames{};
    };

  }  // namespace core
}  // namespace kinverse
