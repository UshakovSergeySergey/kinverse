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
#include "../include/kinverse/visualization/robot_gizmo.h"

kinverse::visualization::RobotGizmo::RobotGizmo(const IGizmo* parentGizmo, core::Robot::ConstPtr robot) : IGizmo{ parentGizmo } {
  setRobot(robot);
}

void kinverse::visualization::RobotGizmo::setRobot(core::Robot::ConstPtr robot) {
  m_robot = robot;

  updateRobotStructure();
  updateRobotConfiguration();
}

kinverse::core::Robot::ConstPtr kinverse::visualization::RobotGizmo::getRobot() const {
  return m_robot;
}

void kinverse::visualization::RobotGizmo::updateRobotStructure() {
  hide();

  m_meshes.clear();
  m_meshGizmos.clear();
  m_endEffectorGizmo = nullptr;

  if (!m_robot)
    return;

  // create meshes
  const auto meshes = m_robot->getMeshes();
  for (const auto& mesh : meshes) {
    auto meshGizmo = std::make_shared<MeshGizmo>(this, mesh);
    m_meshGizmos.push_back(meshGizmo);
  }

  // create end effector
  m_endEffectorGizmo = std::make_shared<CoordinateFrameGizmo>(this);
  m_endEffectorGizmo->setAxesLength(100.0);

  show();
}

void kinverse::visualization::RobotGizmo::updateRobotConfiguration() {
  if (!m_robot)
    return;

  const auto linkTransforms = m_robot->getLinkCoordinateFrames();
  const auto dhTable = m_robot->getDHTable();
  const auto robotConfiguration = m_robot->getConfiguration();

  // update base link mesh
  m_meshGizmos.front()->setTransform(linkTransforms.front());

  // update joint meshes
  for (auto jointCounter = 0u; jointCounter < m_robot->getNumberOfJoints(); ++jointCounter) {
    const Eigen::Affine3d jointTransform = dhTable[jointCounter].getJointTransform(robotConfiguration[jointCounter]);
    const Eigen::Affine3d transform = linkTransforms[jointCounter + 1] * jointTransform;
    m_meshGizmos[jointCounter + 1]->setTransform(transform);
  }

  // update end effector
  const Eigen::Affine3d endEffectorTransform = m_robot->getEndEffectorTransform();
  m_endEffectorGizmo->setTransform(endEffectorTransform);

  render();
}

void kinverse::visualization::RobotGizmo::show(void* renderer) {
  if (m_endEffectorGizmo)
    IGizmo::show(m_endEffectorGizmo, renderer);
  for (const auto& mesh : m_meshGizmos)
    IGizmo::show(mesh, renderer);
  IGizmo::show(renderer);
}

void kinverse::visualization::RobotGizmo::hide(void* renderer) {
  if (m_endEffectorGizmo)
    IGizmo::hide(m_endEffectorGizmo, renderer);
  for (const auto& mesh : m_meshGizmos)
    IGizmo::hide(mesh, renderer);
  IGizmo::hide(renderer);
}
