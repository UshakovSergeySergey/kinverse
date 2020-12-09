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
#include "../include/kinverse/visualization/revolute_joint_gizmo.h"
#include "i_gizmo_impl.h"

kinverse::visualization::RobotGizmo::RobotGizmo(const IGizmo* parentGizmo, core::Robot::ConstPtr robot) : IGizmo{ parentGizmo } {
  m_endEffectorGizmo = std::make_shared<CoordinateFrameGizmo>(this);
  //  m_endEffectorGizmo->setCaption("end effector");
  m_endEffectorGizmo->setAxesLength(100.0);

  setRobot(robot);
}

void kinverse::visualization::RobotGizmo::setRobot(core::Robot::ConstPtr robot) {
  m_robot = robot;
  robotStructureChanged();
}

kinverse::core::Robot::ConstPtr kinverse::visualization::RobotGizmo::getRobot() const {
  return m_robot;
}

void kinverse::visualization::RobotGizmo::setConfiguration(const std::vector<double>& axisValues) {
  m_axisValues = axisValues;
  robotConfigurationChanged();
  render();
}

std::vector<double> kinverse::visualization::RobotGizmo::getConfiguration() const {
  return m_axisValues;
}

void kinverse::visualization::RobotGizmo::robotStructureChanged() {
  // remove old robot gizmos
  {
    for (auto gizmo : m_jointGizmos) {
      IGizmo::hide(gizmo, m_pImpl->getRenderer());
    }
    for (auto gizmo : m_linkGizmos) {
      IGizmo::hide(gizmo, m_pImpl->getRenderer());
    }
    for (auto gizmo : m_jointMeshGizmos) {
      IGizmo::hide(gizmo, m_pImpl->getRenderer());
    }
    m_jointGizmos.clear();
    m_linkGizmos.clear();
    m_jointMeshGizmos.clear();
  }

  // add new robot gizmos
  {
    const auto joints = m_robot->getJointCoordinateFrames();
    const auto numberOfJoints = joints.size();
    for (auto jointCounter = 0u; jointCounter < numberOfJoints; ++jointCounter) {
      auto jointGizmo = std::make_shared<RevoluteJointGizmo>(this, joints[jointCounter], jointCounter);
      m_jointGizmos.push_back(jointGizmo);

      auto jointMeshGizmo = std::make_shared<MeshGizmo>(this);
      jointMeshGizmo->setTransform(joints[jointCounter]);
      m_jointMeshGizmos.push_back(jointMeshGizmo);
    }

    m_endEffectorGizmo->setTransform(m_robot->getLinkCoordinateFrames().back());

    // const auto links = m_robot->getLinkCoordinateFrames();
    // const auto numberOfLinks = links.size();
    // for (auto linkCounter = 0u; linkCounter < numberOfLinks; ++linkCounter) {
    //  auto linkGizmo = std::make_shared<CoordinateFrameGizmo>(this, links[linkCounter], "link " + std::to_string(linkCounter));
    //  linkGizmo->setScale(0.1);
    //  m_linkGizmos.push_back(linkGizmo);
    //}

    for (auto jointCounter = 0u; jointCounter < m_meshes.size(); ++jointCounter) {
      m_jointMeshGizmos[jointCounter]->setMesh(m_meshes[jointCounter]);
      m_jointMeshGizmos[jointCounter]->setTransform(joints[jointCounter]);
    }
  }

  // draw if renderer is not nullptr
  {
    if (m_pImpl->getRenderer()) {
      for (auto gizmo : m_jointGizmos) {
        IGizmo::show(gizmo, m_pImpl->getRenderer());
      }
      for (auto gizmo : m_linkGizmos) {
        IGizmo::show(gizmo, m_pImpl->getRenderer());
      }
      IGizmo::show(m_endEffectorGizmo, m_pImpl->getRenderer());
    }
  }
}

void kinverse::visualization::RobotGizmo::robotConfigurationChanged() {
  {
    const auto joints = m_robot->getJointCoordinateFrames();
    const auto numberOfJoints = joints.size();
    for (auto jointCounter = 0u; jointCounter < numberOfJoints; ++jointCounter) {
      m_jointGizmos[jointCounter]->setTransform(joints[jointCounter]);
    }

    m_endEffectorGizmo->setTransform(m_robot->getLinkCoordinateFrames().back());

    // const auto links = m_robot->getLinkCoordinateFrames();
    // const auto numberOfLinks = links.size();
    // for (auto linkCounter = 0u; linkCounter < numberOfLinks; ++linkCounter) {
    //  m_linkGizmos[linkCounter]->setTransform(links[linkCounter]);
    //}

    for (auto jointCounter = 0u; jointCounter < m_meshes.size(); ++jointCounter) {
      m_jointMeshGizmos[jointCounter]->setMesh(m_meshes[jointCounter]);
      m_jointMeshGizmos[jointCounter]->setTransform(joints[jointCounter]);
    }
  }
}

void kinverse::visualization::RobotGizmo::show(void* renderer) {
  IGizmo::show(renderer);
  for (auto gizmo : m_jointGizmos) {
    IGizmo::show(gizmo, renderer);
  }
  for (auto gizmo : m_linkGizmos) {
    IGizmo::show(gizmo, renderer);
  }
  for (auto gizmo : m_jointMeshGizmos) {
    IGizmo::show(gizmo, renderer);
  }
  IGizmo::show(m_endEffectorGizmo, renderer);
}

void kinverse::visualization::RobotGizmo::setMeshes(const std::vector<core::Mesh::ConstPtr>& jointMeshes) {
  m_meshes = jointMeshes;
  robotConfigurationChanged();
}

std::vector<kinverse::core::Mesh::ConstPtr> kinverse::visualization::RobotGizmo::getMeshes() const {
  return m_meshes;
}
