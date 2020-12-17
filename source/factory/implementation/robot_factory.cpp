#include "stdafx.h"
#include "../include/kinverse/factory/robot_factory.h"
#include <kinverse/math/math.h>
#include <kinverse/io/mesh_reader.h>

kinverse::core::Robot::Ptr kinverse::factory::RobotFactory::create(core::RobotType robotType) {
  core::Robot::Ptr robot{ nullptr };

  switch (robotType) {
    case core::RobotType::KukaKR5Arc:
      robot = createKukaKR5ArcRobot();
      break;
    default:
      throw std::invalid_argument("Failed to create robot! Unknown RobotType received!");
  }

  return robot;
}

kinverse::core::Robot::Ptr kinverse::factory::RobotFactory::createKukaKR5ArcRobot() {
  auto robot = std::make_shared<core::Robot>();

  const std::vector<core::DenavitHartenbergParameters> dhTable{
    { core::JointType::Revolute, -400.0, 0.0, 180.0, M_PI_2 },   //
    { core::JointType::Revolute, 0.0, 0.0, 600.0, 0.0 },         //
    { core::JointType::Revolute, 0.0, -M_PI_2, 120.0, M_PI_2 },  //
    { core::JointType::Revolute, -620.0, 0.0, 0.0, -M_PI_2 },    //
    { core::JointType::Revolute, 0.0, 0.0, 0.0, M_PI_2 },        //
    { core::JointType::Revolute, -115.0, M_PI, 0.0, M_PI }       //
  };
  robot->setDHTable(dhTable);

  Eigen::Affine3d baseTransform;
  baseTransform = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  robot->setBaseTransform(baseTransform);

  const std::vector<core::JointConstraints> jointConstraints{
    { math::degreesToRadians(154.0), math::degreesToRadians(-155.0), math::degreesToRadians(155.0) },  //
    { math::degreesToRadians(154.0), math::degreesToRadians(-180.0), math::degreesToRadians(65.0) },   //
    { math::degreesToRadians(228.0), math::degreesToRadians(-15.0), math::degreesToRadians(158.0) },   //
    { math::degreesToRadians(343.0), math::degreesToRadians(-350.0), math::degreesToRadians(350.0) },  //
    { math::degreesToRadians(384.0), math::degreesToRadians(-130.0), math::degreesToRadians(130.0) },  //
    { math::degreesToRadians(721.0), math::degreesToRadians(-350.0), math::degreesToRadians(350.0) }   //
  };
  robot->setJointConstraints(jointConstraints);

  std::vector<core::Mesh::ConstPtr> meshes{};
  const std::vector<std::string> axisMeshFilenames{
    "D:/Git/kinverse/meshes/kuka-kr5-arc/base-link.ply",  //
    "D:/Git/kinverse/meshes/kuka-kr5-arc/link-1.ply",     //
    "D:/Git/kinverse/meshes/kuka-kr5-arc/link-2.ply",     //
    "D:/Git/kinverse/meshes/kuka-kr5-arc/link-3.ply",     //
    "D:/Git/kinverse/meshes/kuka-kr5-arc/link-4.ply",     //
    "D:/Git/kinverse/meshes/kuka-kr5-arc/link-5.ply",     //
    "D:/Git/kinverse/meshes/kuka-kr5-arc/link-6.ply"      //
  };
  for (auto filename : axisMeshFilenames) {
    meshes.push_back(io::MeshReader().read(filename));
  }
  robot->setMeshes(meshes);

  return robot;
}
