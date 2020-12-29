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
#include "../include/kinverse/core/robot.h"
#include "../include/kinverse/core/forward_kinematics_solver.h"

void kinverse::core::Robot::setDHTable(const std::vector<DenavitHartenbergParameters>& dhTable) {
  m_dhTable = dhTable;
}

std::vector<kinverse::core::DenavitHartenbergParameters> kinverse::core::Robot::getDHTable() const {
  return m_dhTable;
}

void kinverse::core::Robot::setJointConstraints(const std::vector<JointConstraints>& constraints) {
  m_constraints = constraints;
}

std::vector<kinverse::core::JointConstraints> kinverse::core::Robot::getJointConstraints() const {
  return m_constraints;
}

void kinverse::core::Robot::setMeshes(const std::vector<Mesh::ConstPtr>& meshes) {
  m_meshes = meshes;
}

std::vector<kinverse::core::Mesh::ConstPtr> kinverse::core::Robot::getMeshes() const {
  return m_meshes;
}

void kinverse::core::Robot::setTransform(const Eigen::Affine3d& transform) {
  if (!transform.matrix().allFinite())
    throw std::domain_error("Failed to set transform! All values of the transform matrix must be finite numbers!");
  m_transform = transform;
}

Eigen::Affine3d kinverse::core::Robot::getTransform() const {
  return m_transform;
}

void kinverse::core::Robot::setBaseTransform(const Eigen::Affine3d& transform) {
  if (!transform.matrix().allFinite())
    throw std::domain_error("Failed to set base transform! All values of the transform matrix must be finite numbers!");
  m_baseTransform = transform;
}

Eigen::Affine3d kinverse::core::Robot::getBaseTransform() const {
  return m_baseTransform;
}

void kinverse::core::Robot::setTCPTransform(const Eigen::Affine3d& transform) {
  if (!transform.matrix().allFinite())
    throw std::domain_error("Failed to set tcp transform! All values of the transform matrix must be finite numbers!");
  m_tcpTransform = transform;
}

Eigen::Affine3d kinverse::core::Robot::getTCPTransform() const {
  return m_tcpTransform;
}

void kinverse::core::Robot::setConfiguration(const Eigen::VectorXd& configuration) {
  if (configuration.rows() != getNumberOfJoints())
    throw std::domain_error("Failed to set robot configuration! Configuration doesn't match number of joints! Number of joints is " +
                            std::to_string(getNumberOfJoints()) + ", but configuration has only " + std::to_string(configuration.rows()) + " values!");

  if (configuration.allFinite() == false)
    throw std::domain_error("Failed to set robot configuration! All values of the configuration vector must be finite numbers!");

  m_configuration = configuration;

  if (!m_fkSolver) {
    m_fkSolver = std::make_shared<ForwardKinematicsSolver>();
    m_fkSolver->setRobot(shared_from_this());
  }

  m_endEffectorTransform = m_fkSolver->solve(configuration);
  m_linkCoordinateFrames = m_fkSolver->getLinkCoordinateFrames();
}

Eigen::VectorXd kinverse::core::Robot::getConfiguration() const {
  return m_configuration;
}

unsigned int kinverse::core::Robot::getNumberOfJoints() const {
  return static_cast<unsigned int>(m_dhTable.size());
}

Eigen::Affine3d kinverse::core::Robot::getEndEffectorTransform() const {
  return m_endEffectorTransform;
}

std::vector<Eigen::Affine3d> kinverse::core::Robot::getLinkCoordinateFrames() const {
  return m_linkCoordinateFrames;
}
