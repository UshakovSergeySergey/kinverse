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

    class KINVERSE_CORE_API AnalyticalSolver {
     public:
      using Ptr = std::shared_ptr<AnalyticalSolver>;
      using ConstPtr = std::shared_ptr<const AnalyticalSolver>;

      explicit AnalyticalSolver(Robot::ConstPtr robot);
      std::vector<std::vector<double>> solve(const Eigen::Affine3d& endEffectorTransform);

     private:
      double computeGamma() const;
      double getDistanceBetweenSecondAndThirdJoints() const;
      double getDistanceBetweenThirdJointAndWristPosition() const;
      Eigen::Affine3d convertWorldToLocal(const Eigen::Affine3d& transform) const;
      Eigen::Vector3d computeWristPosition(const Eigen::Affine3d& targetTransform) const;
      std::vector<std::vector<double>> solveForPosition() const;
      Eigen::Vector3d getFirstJointZAxis() const;
      std::vector<std::vector<double>> solvePosition(double theta1, bool facingForward) const;
      void getTriangle(double theta1, double& a, double& b, double& c) const;
      Eigen::Vector3d computeSecondJointPosition(double theta1) const;
      std::vector<std::vector<double>> solveForOrientation(const std::vector<double>& configuration) const;
      Eigen::Matrix3d computeWristOrientation(double theta1, double theta2, double theta3) const;

      Robot::ConstPtr m_robot{ nullptr };
      /**
       * @brief Stores angle between robot's Z4 axis and direction from third joint to wrist position.
       */
      double m_gamma{ 0.0 };
      double m_distanceFromSecondToThirdJoint{ 0.0 };
      double m_distanceFromThirdJointToWrist{ 0.0 };
      Eigen::Affine3d m_targetTransform{ Eigen::Affine3d::Identity() };
      Eigen::Vector3d m_wristPosition{ Eigen::Vector3d::Zero() };
    };

  }  // namespace core
}  // namespace kinverse
