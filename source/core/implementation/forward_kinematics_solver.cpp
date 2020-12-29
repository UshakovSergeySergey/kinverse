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
#include "../include/kinverse/core/forward_kinematics_solver.h"
#include "../include/kinverse/core/robot.h"

kinverse::core::ForwardKinematicsSolver::ForwardKinematicsSolver(Robot::ConstPtr robot) {
  setRobot(robot);
}

void kinverse::core::ForwardKinematicsSolver::setRobot(Robot::ConstPtr robot) {
  m_robot = robot;
}

kinverse::core::Robot::ConstPtr kinverse::core::ForwardKinematicsSolver::getRobot() const {
  return m_robot;
}

Eigen::Affine3d kinverse::core::ForwardKinematicsSolver::solve(const Eigen::VectorXd& configuration) {
  if (!m_robot)
    throw std::domain_error("Failed to solve forward kinematics! Robot is nullptr!");

  if (!configuration.allFinite())
    throw std::domain_error("Failed to solve forward kinematics! Robot configuration must be finite vector!");

  const auto numberOfJoints = m_robot->getNumberOfJoints();
  if (configuration.rows() != numberOfJoints)
    throw std::domain_error("Failed to solve forward kinematics! Number of axis values in configuration doesn't match number of robots joints!");

  const auto dhTable = m_robot->getDHTable();

  const auto numberOfLinks = 3 + numberOfJoints;
  if (m_linkCoordinateFrames.size() != numberOfLinks)
    m_linkCoordinateFrames.resize(numberOfLinks);

  Eigen::Affine3d endEffectorTransform = m_robot->getTransform() * m_robot->getBaseTransform();

  // set link to base coordinate frame
  m_linkCoordinateFrames[0] = m_robot->getTransform();

  for (auto jointCounter = 0u; jointCounter < numberOfJoints; ++jointCounter) {
    const auto& dhParameters = dhTable[jointCounter];
    const double axisValue = configuration(jointCounter);

    // set link to joint coordinate frame
    m_linkCoordinateFrames[jointCounter + 1] = endEffectorTransform;

    endEffectorTransform = endEffectorTransform * dhParameters.getTransform(axisValue);
  }

  // set link to flange coordinate frame
  m_linkCoordinateFrames[numberOfLinks - 2] = endEffectorTransform;

  endEffectorTransform = endEffectorTransform * m_robot->getTCPTransform();

  // set link to tcp coordinate frame
  m_linkCoordinateFrames[numberOfLinks - 1] = endEffectorTransform;

  return endEffectorTransform;
}

std::vector<Eigen::Affine3d> kinverse::core::ForwardKinematicsSolver::getLinkCoordinateFrames() const {
  return m_linkCoordinateFrames;
}
