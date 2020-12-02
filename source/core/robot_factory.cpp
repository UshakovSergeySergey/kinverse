#include "stdafx.h"
#include "robot_factory.h"
#include <math/math.h>

kinverse::core::Robot::Ptr kinverse::core::RobotFactory::create(RobotType robotType) {
  Robot::Ptr robot{ nullptr };

  switch (robotType) {
    case RobotType::KukaKR5Arc:
      robot = createKukaKR5ArcRobot();
      break;
    default:
      throw std::invalid_argument("Failed to create robot! Unknown RobotType received!");
  }

  return robot;
}

kinverse::core::Robot::Ptr kinverse::core::RobotFactory::createKukaKR5ArcRobot() {
  auto robot = std::make_shared<Robot>();

  const std::vector<DenavitHartenbergParameters> dhTable{
    { JointType::Revolute, -400.0, 0.0, 180.0, M_PI_2 },   //
    { JointType::Revolute, 0.0, 0.0, 600.0, 0.0 },         //
    { JointType::Revolute, 0.0, -M_PI_2, 120.0, M_PI_2 },  //
    { JointType::Revolute, -620.0, 0.0, 0.0, -M_PI_2 },    //
    { JointType::Revolute, 0.0, 0.0, 0.0, M_PI_2 },        //
    { JointType::Revolute, -115.0, M_PI, 0.0, M_PI }       //
  };
  robot->setDHTable(dhTable);

  Eigen::Affine3d baseTransform;
  baseTransform = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  robot->setBaseTransform(baseTransform);

  const std::vector<JointConstraints> jointConstraints{
    { math::degreesToRadians(154.0), math::degreesToRadians(-155.0), math::degreesToRadians(155.0) },  //
    { math::degreesToRadians(154.0), math::degreesToRadians(-180.0), math::degreesToRadians(65.0) },   //
    { math::degreesToRadians(228.0), math::degreesToRadians(-15.0), math::degreesToRadians(158.0) },   //
    { math::degreesToRadians(343.0), math::degreesToRadians(-350.0), math::degreesToRadians(350.0) },  //
    { math::degreesToRadians(384.0), math::degreesToRadians(-130.0), math::degreesToRadians(130.0) },  //
    { math::degreesToRadians(721.0), math::degreesToRadians(-350.0), math::degreesToRadians(350.0) }   //
  };
  robot->setJointConstraints(jointConstraints);

  return robot;
}
