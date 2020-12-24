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
#include "../include/kinverse/core/joint_constraints.h"
#include <kinverse/math/math.h>

kinverse::core::JointConstraints::JointConstraints(double maximumSpeed, double minimumAxisValue, double maximumAxisValue) {
  setMaximumSpeed(maximumSpeed);
  setMinimumAxisValue(minimumAxisValue);
  setMaximumAxisValue(maximumAxisValue);
}

void kinverse::core::JointConstraints::setMaximumSpeed(double maximumSpeed) {
  if (std::isfinite(maximumSpeed) == false || maximumSpeed < 0.0)
    throw std::domain_error("Failed to set maximum speed! Speed must be finite positive number but provided " + std::to_string(maximumSpeed) + "!");
  m_maximumSpeed = maximumSpeed;
}

double kinverse::core::JointConstraints::getMaximumSpeed() const {
  return m_maximumSpeed;
}

void kinverse::core::JointConstraints::setMinimumAxisValue(double minimumAxisValue) {
  if (std::isfinite(minimumAxisValue) == false)
    throw std::domain_error("Failed to set minimum axis value! Value must be finite number but provided " + std::to_string(minimumAxisValue) + "!");
  m_minimumAxisValue = minimumAxisValue;
}

double kinverse::core::JointConstraints::getMinimumAxisValue() const {
  return m_minimumAxisValue;
}

void kinverse::core::JointConstraints::setMaximumAxisValue(double maximumAxisValue) {
  if (std::isfinite(maximumAxisValue) == false)
    throw std::domain_error("Failed to set maximum axis value! Value must be finite number but provided " + std::to_string(maximumAxisValue) + "!");
  m_maximumAxisValue = maximumAxisValue;
}

double kinverse::core::JointConstraints::getMaximumAxisValue() const {
  return m_maximumAxisValue;
}

bool kinverse::core::JointConstraints::violatesSpeedConstraint(double speed) const {
  if (std::isfinite(speed) == false || speed < 0.0)
    throw std::domain_error("Failed to check whether speed violates speed constraints! Speed must be finite positive number but provided " +
                            std::to_string(speed) + "!");
  const bool violates = m_maximumSpeed < speed;
  return violates;
}

bool kinverse::core::JointConstraints::violatesRangeConstraint(double axisValue) const {
  if (std::isfinite(axisValue) == false)
    throw std::domain_error("Failed to check whether axis value violates range constraints! Value must be finite number but provided " +
                            std::to_string(axisValue) + "!");
  const bool violates = (axisValue < m_minimumAxisValue) || (axisValue > m_maximumAxisValue);
  return violates;
}

double kinverse::core::JointConstraints::clampAxisValue(double axisValue) const {
  if (std::isfinite(axisValue) == false)
    throw std::domain_error("Failed to clamp axis value to fit constraints! Value must be finite number but provided " + std::to_string(axisValue) + "!");

  if (m_minimumAxisValue > m_maximumAxisValue)
    throw std::domain_error("Failed to clamp axis value to fit constraints! Upper bound must be greater or equal to lower bound!");

  return std::clamp(axisValue, m_minimumAxisValue, m_maximumAxisValue);
}

bool kinverse::core::operator==(const JointConstraints& lhs, const JointConstraints& rhs) {
  if (!math::doublesAreEqual(lhs.getMaximumSpeed(), rhs.getMaximumSpeed()))
    return false;
  if (!math::doublesAreEqual(lhs.getMinimumAxisValue(), rhs.getMinimumAxisValue()))
    return false;
  if (!math::doublesAreEqual(lhs.getMaximumAxisValue(), rhs.getMaximumAxisValue()))
    return false;

  return true;
}

bool kinverse::core::operator!=(const JointConstraints& lhs, const JointConstraints& rhs) {
  return !(lhs == rhs);
}
