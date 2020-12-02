#include "stdafx.h"
#include "robot_gizmo.h"
#include "revolute_joint_gizmo.h"
#include "i_gizmo_impl.h"

kinverse::visualization::RobotGizmo::RobotGizmo(const IGizmo* parentGizmo, core::Robot::ConstPtr robot) : IGizmo{ parentGizmo } {
  m_endEffectorGizmo = std::make_shared<CoordinateFrameGizmo>(this);
  //  m_endEffectorGizmo->setCaption("end effector");
  m_endEffectorGizmo->setScale(0.1);

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
    for (auto gizmo : m_jointMeshGizmos) {
      gizmo->hide(m_pImpl->m_renderer);
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
    if (m_pImpl->m_renderer) {
      for (auto gizmo : m_jointGizmos) {
        gizmo->show(m_pImpl->m_renderer);
      }
      for (auto gizmo : m_linkGizmos) {
        gizmo->show(m_pImpl->m_renderer);
      }
      m_endEffectorGizmo->show(m_pImpl->m_renderer);
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
    gizmo->show(renderer);
  }
  for (auto gizmo : m_linkGizmos) {
    gizmo->show(renderer);
  }
  for (auto gizmo : m_jointMeshGizmos) {
    gizmo->show(renderer);
  }
  m_endEffectorGizmo->show(renderer);
}

void kinverse::visualization::RobotGizmo::setMeshes(const std::vector<core::Mesh::ConstPtr>& jointMeshes) {
  m_meshes = jointMeshes;
  robotConfigurationChanged();
}

std::vector<kinverse::core::Mesh::ConstPtr> kinverse::visualization::RobotGizmo::getMeshes() const {
  return m_meshes;
}
