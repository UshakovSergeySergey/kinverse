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
#include "../include/kinverse/core/denavit_hartenberg_parameters.h"
#include <kinverse/math/math.h>

kinverse::core::DenavitHartenbergParameters::DenavitHartenbergParameters(
    JointType jointType, double zAxisDisplacement, double zAxisRotation, double xAxisDisplacement, double xAxisRotation) {
  setJointType(jointType);
  setZAxisDisplacement(zAxisDisplacement);
  setZAxisRotation(zAxisRotation);
  setXAxisDisplacement(xAxisDisplacement);
  setXAxisRotation(xAxisRotation);
}

void kinverse::core::DenavitHartenbergParameters::setJointType(JointType jointType) {
  m_jointType = jointType;
}

kinverse::core::JointType kinverse::core::DenavitHartenbergParameters::getJointType() const {
  return m_jointType;
}

void kinverse::core::DenavitHartenbergParameters::setZAxisDisplacement(double displacement) {
  if (std::isfinite(displacement) == false)
    throw std::domain_error("Failed to set z axis displacement! Value must be finite number but provided " + std::to_string(displacement) + "!");
  m_zAxisDisplacement = displacement;
}

double kinverse::core::DenavitHartenbergParameters::getZAxisDisplacement() const {
  return m_zAxisDisplacement;
}

void kinverse::core::DenavitHartenbergParameters::setZAxisRotation(double angle) {
  if (std::isfinite(angle) == false)
    throw std::domain_error("Failed to set z axis rotation! Value must be finite number but provided " + std::to_string(angle) + "!");
  m_zAxisRotation = angle;
}

double kinverse::core::DenavitHartenbergParameters::getZAxisRotation() const {
  return m_zAxisRotation;
}

void kinverse::core::DenavitHartenbergParameters::setXAxisDisplacement(double displacement) {
  if (std::isfinite(displacement) == false)
    throw std::domain_error("Failed to set x axis displacement! Value must be finite number but provided " + std::to_string(displacement) + "!");
  m_xAxisDisplacement = displacement;
}

double kinverse::core::DenavitHartenbergParameters::getXAxisDisplacement() const {
  return m_xAxisDisplacement;
}

void kinverse::core::DenavitHartenbergParameters::setXAxisRotation(double angle) {
  if (std::isfinite(angle) == false)
    throw std::domain_error("Failed to set x axis rotation! Value must be finite number but provided " + std::to_string(angle) + "!");
  m_xAxisRotation = angle;
}

double kinverse::core::DenavitHartenbergParameters::getXAxisRotation() const {
  return m_xAxisRotation;
}

Eigen::Affine3d kinverse::core::DenavitHartenbergParameters::getTransform(double value) const {
  return getTransformZ(value) * getTransformX();
}

Eigen::Affine3d kinverse::core::DenavitHartenbergParameters::getTransformZ(double value) const {
  if (std::isfinite(value) == false)
    throw std::domain_error("Failed to compute z transform! Value must be finite number but provided " + std::to_string(value) + "!");

  double angle = 0.0;
  double displacement = 0.0;

  if (m_jointType == JointType::Revolute)
    angle = value;

  if (m_jointType == JointType::Prismatic)
    displacement = value;

  const Eigen::Affine3d transform =
      Eigen::Translation3d(0.0, 0.0, m_zAxisDisplacement + displacement) * Eigen::AngleAxisd(m_zAxisRotation + angle, Eigen::Vector3d::UnitZ());
  return transform;
}

Eigen::Affine3d kinverse::core::DenavitHartenbergParameters::getTransformX() const {
  const Eigen::Affine3d transform = Eigen::Translation3d(m_xAxisDisplacement, 0.0, 0.0) * Eigen::AngleAxisd(m_xAxisRotation, Eigen::Vector3d::UnitX());
  return transform;
}

bool kinverse::core::operator==(const DenavitHartenbergParameters& lhs, const DenavitHartenbergParameters& rhs) {
  if (lhs.getJointType() != rhs.getJointType())
    return false;
  if (!math::doublesAreEqual(lhs.getZAxisDisplacement(), rhs.getZAxisDisplacement()))
    return false;
  if (!math::doublesAreEqual(lhs.getZAxisRotation(), rhs.getZAxisRotation()))
    return false;
  if (!math::doublesAreEqual(lhs.getXAxisDisplacement(), rhs.getXAxisDisplacement()))
    return false;
  if (!math::doublesAreEqual(lhs.getXAxisRotation(), rhs.getXAxisRotation()))
    return false;

  return true;
}

bool kinverse::core::operator!=(const DenavitHartenbergParameters& lhs, const DenavitHartenbergParameters& rhs) {
  return !(lhs == rhs);
}
