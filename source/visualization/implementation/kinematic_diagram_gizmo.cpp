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
#include "../include/kinverse/visualization/kinematic_diagram_gizmo.h"

kinverse::visualization::KinematicDiagramGizmo::KinematicDiagramGizmo(const IGizmo* parentGizmo, core::Robot::ConstPtr robot) : IGizmo{ parentGizmo } {
  setRobot(robot);
}

void kinverse::visualization::KinematicDiagramGizmo::setRobot(core::Robot::ConstPtr robot) {
  m_robot = robot;

  updateRobotStructure();
  updateRobotConfiguration();
}

kinverse::core::Robot::ConstPtr kinverse::visualization::KinematicDiagramGizmo::getRobot() const {
  return m_robot;
}

void kinverse::visualization::KinematicDiagramGizmo::updateRobotStructure() {
  hide();

  m_jointGizmos.clear();
  m_endEffectorGizmo = nullptr;

  if (!m_robot)
    return;

  const auto dhTable = m_robot->getDHTable();
  for (auto jointCounter = 0u; jointCounter < dhTable.size(); ++jointCounter) {
    // create joints
    auto jointGizmo = std::make_shared<JointGizmo>(this, dhTable[jointCounter].getJointType(), Eigen::Affine3d::Identity(), jointCounter);
    m_jointGizmos.push_back(jointGizmo);

    // create link between current joint and the next one
    auto linkGizmo = std::make_shared<LinkGizmo>(this, dhTable[jointCounter]);
    m_linkGizmos.push_back(linkGizmo);
  }

  // create end effector
  m_endEffectorGizmo = std::make_shared<CoordinateFrameGizmo>(this);
  m_endEffectorGizmo->setAxesLength(100.0);

  show();
}

void kinverse::visualization::KinematicDiagramGizmo::updateRobotConfiguration() {
  if (!m_robot)
    return;

  const auto joints = m_robot->getJointCoordinateFrames();
  for (auto jointCounter = 0u; jointCounter < joints.size(); ++jointCounter) {
    // update joints
    m_jointGizmos[jointCounter]->setTransform(joints[jointCounter]);

    // update link between current joint and the next one
    m_linkGizmos[jointCounter]->setTransform(joints[jointCounter]);
  }

  // update end effector
  const Eigen::Affine3d endEffectorTransform = m_robot->getLinkCoordinateFrames().back();
  m_endEffectorGizmo->setTransform(endEffectorTransform);

  render();
}

void kinverse::visualization::KinematicDiagramGizmo::show(void* renderer) {
  if (m_endEffectorGizmo)
    IGizmo::show(m_endEffectorGizmo, renderer);
  for (const auto& joint : m_jointGizmos)
    IGizmo::show(joint, renderer);
  for (const auto& joint : m_linkGizmos)
    IGizmo::show(joint, renderer);
  IGizmo::show(renderer);
}

void kinverse::visualization::KinematicDiagramGizmo::hide(void* renderer) {
  if (m_endEffectorGizmo)
    IGizmo::hide(m_endEffectorGizmo, renderer);
  for (const auto& joint : m_jointGizmos)
    IGizmo::hide(joint, renderer);
  for (const auto& joint : m_linkGizmos)
    IGizmo::hide(joint, renderer);
  IGizmo::hide(renderer);

  //@todo we need to show/hide childs before parent, because parent initiates render call
}
