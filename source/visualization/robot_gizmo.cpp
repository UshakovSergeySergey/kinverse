#include "stdafx.h"
#include "robot_gizmo.h"
#include "revolute_joint_gizmo.h"
#include "i_gizmo_impl.h"

kinverse::visualization::RobotGizmo::RobotGizmo(const IGizmo* parentGizmo, core::Robot::ConstPtr robot) : IGizmo{ parentGizmo } {
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
  update();
}

std::vector<double> kinverse::visualization::RobotGizmo::getConfiguration() const {
  return m_axisValues;
}

void kinverse::visualization::RobotGizmo::robotStructureChanged() {
  // remove old robot gizmos
  {
    for (auto gizmo : m_jointGizmos) {
      gizmo->hide(m_pImpl->m_renderer);
    }
    for (auto gizmo : m_linkGizmos) {
      gizmo->hide(m_pImpl->m_renderer);
    }
    m_jointGizmos.clear();
    m_linkGizmos.clear();
  }

  // add new robot gizmos
  {
    const auto joints = m_robot->getJointCoordinateFrames();
    const auto numberOfJoints = joints.size();
    for (auto jointCounter = 0u; jointCounter < numberOfJoints; ++jointCounter) {
      auto jointGizmo = std::make_shared<RevoluteJointGizmo>(this, joints[jointCounter], jointCounter);
      m_jointGizmos.push_back(jointGizmo);
    }

    // const auto links = m_robot->getLinkCoordinateFrames();
    // const auto numberOfLinks = links.size();

    // for (auto linkCounter = 0u; linkCounter < numberOfLinks; ++linkCounter) {
    //  auto linkGizmo = std::make_shared<CoordinateFrameGizmo>(links[linkCounter], "link " + std::to_string(linkCounter));
    //  linkGizmo->setScale(0.1);
    //  m_linkGizmos.push_back(linkGizmo);
    //}
  }

  // draw if renderer is not nullptr
  {
    if (m_pImpl->m_renderer) {
      for (auto gizmo : m_jointGizmos) {
        gizmo->draw(m_pImpl->m_renderer);
      }
      for (auto gizmo : m_linkGizmos) {
        gizmo->draw(m_pImpl->m_renderer);
      }
    }
  }
}

void kinverse::visualization::RobotGizmo::robotConfigurationChanged() {
  {
    const auto joints = m_robot->getJointCoordinateFrames(m_axisValues);
    const auto numberOfJoints = joints.size();
    for (auto jointCounter = 0u; jointCounter < numberOfJoints; ++jointCounter) {
      m_jointGizmos[jointCounter]->setTransform(joints[jointCounter]);
    }

    // const auto links = m_robot->getLinkCoordinateFrames();
    // const auto numberOfLinks = links.size();

    // for (auto linkCounter = 0u; linkCounter < numberOfLinks; ++linkCounter) {
    //  auto linkGizmo = std::make_shared<CoordinateFrameGizmo>(links[linkCounter], "link " + std::to_string(linkCounter));
    //  linkGizmo->setScale(0.1);
    //  m_linkGizmos.push_back(linkGizmo);
    //}
  }
}

void kinverse::visualization::RobotGizmo::draw(void* renderer) {
  IGizmo::draw(renderer);
  for (auto gizmo : m_jointGizmos) {
    gizmo->draw(renderer);
  }
  for (auto gizmo : m_linkGizmos) {
    gizmo->draw(renderer);
  }
}
