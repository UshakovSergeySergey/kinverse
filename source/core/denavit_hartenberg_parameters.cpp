#include "stdafx.h"
#include "denavit_hartenberg_parameters.h"

kinverse::core::DenavitHartenbergParameters::DenavitHartenbergParameters(JointType jointType, double d, double theta, double r, double alpha) :
    m_jointType{ jointType }, m_alpha{ alpha }, m_d{ d }, m_r{ r }, m_theta{ theta } {
}

kinverse::core::DenavitHartenbergParameters::DenavitHartenbergParameters(JointType jointType,
                                                                         const Eigen::Vector3d& position,
                                                                         const Eigen::Vector3d& direction) :
    m_jointType{ jointType } {
  throw std::exception("Not implemented yet!");
}

void kinverse::core::DenavitHartenbergParameters::setJointType(JointType jointType) {
  m_jointType = jointType;
}

kinverse::core::JointType kinverse::core::DenavitHartenbergParameters::getJointType() const {
  return m_jointType;
}

Eigen::Affine3d kinverse::core::DenavitHartenbergParameters::getTransform(double value) const {
  double angle = 0.0;
  double displacement = 0.0;

  if (m_jointType == JointType::Revolute)
    angle = value;

  if (m_jointType == JointType::Prismatic)
    displacement = value;

  const Eigen::Affine3d transform = Eigen::Translation3d(0.0, 0.0, m_d + displacement) * Eigen::AngleAxisd(m_theta + angle, Eigen::Vector3d::UnitZ()) *
                                    Eigen::Translation3d(m_r, 0.0, 0.0) * Eigen::AngleAxisd(m_alpha, Eigen::Vector3d::UnitX());
  return transform;
}
