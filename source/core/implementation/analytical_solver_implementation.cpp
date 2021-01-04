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

#include "stdafx.h"
#include "analytical_solver_implementation.h"

kinverse::core::AnalyticalSolverImplementation::AnalyticalSolverImplementation(Robot::ConstPtr robot) {
  if (!robot)
    throw std::domain_error("Failed to instantiate AnalyticalSolverImplementation! Robot cannot be nullptr!");

  if (robot->getNumberOfJoints() != 6)
    throw std::domain_error("Failed to instantiate AnalyticalSolverImplementation! Robot must have exactly 6 joints, but it has " +
                            std::to_string(robot->getNumberOfJoints()) + "!");

  m_robot = robot;

  m_gamma = computeGamma(robot);
  m_distanceFromA3ToA2 = std::abs(robot->getDHTable()[1].getXAxisDisplacement());
  m_distanceFromA3ToWrist = (robot->getDHTable()[2].getTransform() * robot->getDHTable()[3].getTransform()).translation().norm();
  m_worldToA1LocalTransform = (robot->getTransform() * robot->getBaseTransform()).inverse();
}

double kinverse::core::AnalyticalSolverImplementation::getGamma() const {
  return m_gamma;
}

double kinverse::core::AnalyticalSolverImplementation::getDistanceFromA3ToA2() const {
  return m_distanceFromA3ToA2;
}

double kinverse::core::AnalyticalSolverImplementation::getDistanceFromA3ToWrist() const {
  return m_distanceFromA3ToWrist;
}

Eigen::Vector3d kinverse::core::AnalyticalSolverImplementation::getA1ZAxis() const {
  return Eigen::Vector3d::UnitZ();
}

double kinverse::core::AnalyticalSolverImplementation::computeGamma(Robot::ConstPtr robot) const {
  const Eigen::Vector3d fourthJointPosition = robot->getDHTable()[2].getTransform().translation();
  const Eigen::Vector3d wristPosition = (robot->getDHTable()[2].getTransform() * robot->getDHTable()[3].getTransform()).translation();

  const double a = (wristPosition - fourthJointPosition).norm();
  double c = wristPosition.norm();

  if (c < std::numeric_limits<double>::epsilon())
    c = 1.0;

  double angle = acos(a / c);
  return angle;
}

Eigen::Affine3d kinverse::core::AnalyticalSolverImplementation::convertWorldToA1Local(const Eigen::Affine3d& transform) const {
  return m_worldToA1LocalTransform * transform;
}

Eigen::Affine3d kinverse::core::AnalyticalSolverImplementation::convertTCPToFlange(const Eigen::Affine3d& transform) const {
  return transform * m_robot->getInverseTCPTransform();
}
