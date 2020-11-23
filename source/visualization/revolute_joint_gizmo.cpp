#include "stdafx.h"
#include "revolute_joint_gizmo.h"

kinverse::visualization::RevoluteJointGizmo::RevoluteJointGizmo(const IGizmo* parentGizmo, const Eigen::Affine3d& transform, unsigned int jointIndex) :
    IGizmo{ parentGizmo } {
  m_coordinateFrame = std::make_shared<CoordinateFrameGizmo>(this);
  m_coordinateFrame->setScale(0.25);
  m_cylinder = std::make_shared<CylinderGizmo>(this);

  setTransform(transform);
  setJointIndex(jointIndex);
}

void kinverse::visualization::RevoluteJointGizmo::draw(void* renderer) {
  IGizmo::draw(renderer);
  m_coordinateFrame->draw(renderer);
  m_cylinder->draw(renderer);
}

void kinverse::visualization::RevoluteJointGizmo::setTransform(const Eigen::Affine3d& transform) {
  m_transform = transform;
  m_coordinateFrame->setTransform(transform);
  m_cylinder->setTransform(transform);
}

Eigen::Affine3d kinverse::visualization::RevoluteJointGizmo::getTransform() const {
  return m_transform;
}

void kinverse::visualization::RevoluteJointGizmo::setJointIndex(unsigned int jointIndex) {
  m_jointIndex = jointIndex;

  const auto index = std::to_string(m_jointIndex);

  m_coordinateFrame->setCaption("frame" + index);

  std::array<std::string, 3> axisLabels{ "X" + index, "Y" + index, "Z" + index };
  m_coordinateFrame->setAxesLabels(axisLabels);
}

unsigned int kinverse::visualization::RevoluteJointGizmo::getJointIndex() const {
  return m_jointIndex;
}
