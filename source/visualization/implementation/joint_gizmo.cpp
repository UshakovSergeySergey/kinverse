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
#include "../include/kinverse/visualization/cube_gizmo.h"
#include "../include/kinverse/visualization/cylinder_gizmo.h"
#include "../include/kinverse/visualization/joint_gizmo.h"

kinverse::visualization::JointGizmo::JointGizmo(const IGizmo* parentGizmo,
                                                core::JointType jointType,
                                                const Eigen::Affine3d& transform,
                                                unsigned int jointIndex) :
    IGizmo{ parentGizmo } {
  m_coordinateFrame = std::make_shared<CoordinateFrameGizmo>(this);
  m_coordinateFrame->setAxesLength(100.0);
  m_jointMesh = std::make_shared<CylinderGizmo>(this);

  setJointType(jointType);
  setTransform(transform);
  setJointIndex(jointIndex);
}

void kinverse::visualization::JointGizmo::show(void* renderer) {
  IGizmo::show(m_coordinateFrame, renderer);
  IGizmo::show(m_jointMesh, renderer);
  IGizmo::show(renderer);
}

void kinverse::visualization::JointGizmo::hide(void* renderer) {
  IGizmo::hide(m_coordinateFrame, renderer);
  IGizmo::hide(m_jointMesh, renderer);
  IGizmo::hide(renderer);
}

void kinverse::visualization::JointGizmo::setJointType(core::JointType jointType) {
  if (m_jointType == jointType)
    return;
  m_jointType = jointType;

  IGizmo::hide(m_jointMesh);

  if (m_jointType == core::JointType::Revolute)
    m_jointMesh = std::make_shared<CylinderGizmo>(this, m_transform);
  else
    m_jointMesh = std::make_shared<CubeGizmo>(this, m_transform);

  IGizmo::show(m_jointMesh);
}

kinverse::core::JointType kinverse::visualization::JointGizmo::getJointType() const {
  return m_jointType;
}

void kinverse::visualization::JointGizmo::setTransform(const Eigen::Affine3d& transform) {
  m_transform = transform;
  m_coordinateFrame->setTransform(transform);

  if (m_jointType == core::JointType::Revolute)
    std::dynamic_pointer_cast<CylinderGizmo>(m_jointMesh)->setTransform(transform);
  else
    std::dynamic_pointer_cast<CubeGizmo>(m_jointMesh)->setTransform(transform);
}

Eigen::Affine3d kinverse::visualization::JointGizmo::getTransform() const {
  return m_transform;
}

void kinverse::visualization::JointGizmo::setJointIndex(unsigned int jointIndex) {
  m_jointIndex = jointIndex;

  const auto index = std::to_string(m_jointIndex);

  const std::array<std::string, 3> axisLabels{ "X" + index, "Y" + index, "Z" + index };
  m_coordinateFrame->setAxesLabels(axisLabels);
}

unsigned int kinverse::visualization::JointGizmo::getJointIndex() const {
  return m_jointIndex;
}
