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

void kinverse::core::Robot::setConfiguration(const std::vector<double>& configuration) {
  m_configuration = std::vector<double>(configuration.size(), 0.0);
  for (auto jointCounter = 0u; jointCounter < m_constraints.size(); ++jointCounter) {
    const double clampedAngle = m_constraints[jointCounter].clampAxisValue(configuration[jointCounter]);
    m_configuration[jointCounter] = clampedAngle;
  }
}

std::vector<double> kinverse::core::Robot::getConfiguration() const {
  return getAxisValues(m_configuration);
}

unsigned int kinverse::core::Robot::getNumberOfJoints() const {
  return static_cast<unsigned int>(m_dhTable.size());
}

unsigned int kinverse::core::Robot::getNumberOfLinks() const {
  return getNumberOfJoints() + 1;
}

std::vector<Eigen::Affine3d> kinverse::core::Robot::getJointCoordinateFrames() const {
  const auto axisValues = getAxisValues(m_configuration);

  std::vector<Eigen::Affine3d> frames{};

  Eigen::Affine3d transform = m_baseTransform;
  for (auto jointCounter = 0u; jointCounter < getNumberOfJoints(); jointCounter++) {
    const auto& dhParameters = m_dhTable[jointCounter];
    const double axisValue = axisValues[jointCounter];

    Eigen::Affine3d jointTransform;
    if (dhParameters.getJointType() == JointType::Revolute)
      jointTransform = Eigen::AngleAxisd(axisValue, Eigen::Vector3d::UnitZ());
    else
      jointTransform = Eigen::Translation3d(0.0, 0.0, axisValue);

    const Eigen::Affine3d currentTransform = transform * jointTransform;
    frames.push_back(currentTransform);

    transform = transform * dhParameters.getTransform(axisValue);
  }

  return frames;
}

std::vector<Eigen::Affine3d> kinverse::core::Robot::getLinkCoordinateFrames() const {
  // Number of links is always equal to number of joints + 1
  // The first link (also known as base frame or inertial frame) is always the origin.
  // The last link is the end-effector.

  const auto axisValues = getAxisValues(m_configuration);

  std::vector<Eigen::Affine3d> frames{};

  Eigen::Affine3d transform = m_baseTransform;

  frames.push_back(transform);

  for (auto jointCounter = 0u; jointCounter < getNumberOfJoints(); jointCounter++) {
    const auto& dhParameters = m_dhTable[jointCounter];
    const double axisValue = axisValues[jointCounter];

    const Eigen::Affine3d currentTransform = dhParameters.getTransform(axisValue);
    transform = transform * currentTransform;
    frames.push_back(transform);
  }

  return frames;
}

std::vector<double> kinverse::core::Robot::getAxisValues(const std::vector<double>& axisValues) const {
  if (axisValues.empty()) {
    // if empty vector was provided then assume zero configuration
    return std::vector<double>(getNumberOfJoints(), 0.0);
  }

  // otherwise check that number of joints and values matches
  if (axisValues.size() != m_dhTable.size()) {
    std::stringstream ss;
    ss << "Robot has " << getNumberOfJoints() << " joints, but only " << axisValues.size() << " axis values were given!";
    throw std::invalid_argument(ss.str());
  }

  return axisValues;
}

void kinverse::core::Robot::setBaseTransform(const Eigen::Affine3d& transform) {
  if (!transform.matrix().allFinite())
    throw std::domain_error("Failed to set base transform! All values of the transform matrix must be finite numbers!");
  m_baseTransform = transform;
}

Eigen::Affine3d kinverse::core::Robot::getBaseTransform() const {
  return m_baseTransform;
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

Eigen::Affine3d kinverse::core::Robot::getEndEffectorTransform() const {
  return getLinkCoordinateFrames().back();
}
