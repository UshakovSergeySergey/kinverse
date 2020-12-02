#include "stdafx.h"
#include "joint_constraints.h"
#include <math/math.h>

kinverse::core::JointConstraints::JointConstraints(double maximumSpeed, double minimumAxisValue, double maximumAxisValue) :
    m_maximumSpeed{ maximumSpeed }, m_minimumAxisValue{ minimumAxisValue }, m_maximumAxisValue{ maximumAxisValue } {
}

void kinverse::core::JointConstraints::setMaximumSpeed(double maximumSpeed) {
  m_maximumSpeed = maximumSpeed;
}

double kinverse::core::JointConstraints::getMaximumSpeed() const {
  return m_maximumSpeed;
}

void kinverse::core::JointConstraints::setMinimumAxisValue(double minimumAxisValue) {
  m_minimumAxisValue = minimumAxisValue;
}

double kinverse::core::JointConstraints::getMinimumAxisValue() const {
  return m_minimumAxisValue;
}

void kinverse::core::JointConstraints::setMaximumAxisValue(double maximumAxisValue) {
  m_maximumAxisValue = maximumAxisValue;
}

double kinverse::core::JointConstraints::getMaximumAxisValue() const {
  return m_maximumAxisValue;
}

bool kinverse::core::JointConstraints::violatesRangeConstraint(double axisValue) const {
  const bool violates = (axisValue < m_minimumAxisValue) || (axisValue > m_maximumAxisValue);
  return violates;
}

double kinverse::core::JointConstraints::clampAxisValue(double axisValue) const {
  return std::clamp(axisValue, m_minimumAxisValue, m_maximumAxisValue);
}
