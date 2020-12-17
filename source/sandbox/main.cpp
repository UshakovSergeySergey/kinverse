#include "stdafx.h"

#include <kinverse/visualization/kinverse_visualizer.h>
#include <kinverse/visualization/coordinate_frame_gizmo.h>
#include <kinverse/visualization/joint_gizmo.h>
#include <kinverse/visualization/mesh_gizmo.h>
#include <kinverse/visualization/robot_gizmo.h>
#include <kinverse/core/robot.h>
#include <kinverse/factory/robot_factory.h>
#include <kinverse/core/analytical_solver.h>
#include <kinverse/math/math.h>
#include <kinverse/io/mesh_reader.h>

#include <iostream>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkSTLReader.h>
#include <unsupported/Eigen/EulerAngles>

void printConfiguration(const std::vector<double>& configuration) {
  std::cout << std::fixed << std::showpoint << std::setprecision(5);
  std::cout << "(A1, A2, A3, A4, A5, A6) = ("                              //
            << kinverse::math::radiansToDegrees(configuration[0]) << ", "  //
            << kinverse::math::radiansToDegrees(configuration[1]) << ", "  //
            << kinverse::math::radiansToDegrees(configuration[2]) << ", "  //
            << kinverse::math::radiansToDegrees(configuration[3]) << ", "  //
            << kinverse::math::radiansToDegrees(configuration[4]) << ", "  //
            << kinverse::math::radiansToDegrees(configuration[5]) << ")" << std::endl;
}

void printEndEffector(const Eigen::Vector3d& xyz, const Eigen::Vector3d& abc) {
  std::cout << std::fixed << std::showpoint << std::setprecision(5);
  std::cout << "(X, Y, Z, A, B, C) = ("                           //
            << xyz.x() << ", "                                    //
            << xyz.y() << ", "                                    //
            << xyz.z() << ", "                                    //
            << kinverse::math::radiansToDegrees(abc.x()) << ", "  //
            << kinverse::math::radiansToDegrees(abc.y()) << ", "  //
            << kinverse::math::radiansToDegrees(abc.z()) << ")" << std::endl;
}

int debugVisualizer();

int main(int argc, char** argv) {
  return debugVisualizer();
  // бага, если поставить робота в конфигурацию (0, 0, 0, 0, 0, 0) и взять координаты endeffector как таргет для ik, решение будет странным, почему он
  // складывается в гармошку?

  // create robot
  auto robot = kinverse::factory::RobotFactory::create(kinverse::core::RobotType::KukaKR5Arc);

  // set robot initial configuration
  const std::vector<double> robotConfiguration{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  //{ kinverse::math::degreesToRadians(22.854), kinverse::math::degreesToRadians(-80.0),
  //                                              kinverse::math::degreesToRadians(80.0),   kinverse::math::degreesToRadians(0.073),
  //                                              kinverse::math::degreesToRadians(22.879), kinverse::math::degreesToRadians(119.070) };
  robot->setConfiguration(robotConfiguration);

  // set target for IK
  const Eigen::Affine3d targetTransform = robot->getLinkCoordinateFrames().back();
  Eigen::Vector3d targetXYZ;
  Eigen::Vector3d targetABC;
  kinverse::math::toXYZABC(targetTransform, targetXYZ, targetABC);

  // const Eigen::Vector3d targetXYZ{ 400.0, -300.0, 300.0 };
  // const Eigen::Vector3d targetXYZ{ 400.0, 300.0, 300.0 };
  // const Eigen::Vector3d targetXYZ{ 600.0, -300.0, 300.0 };
  // const Eigen::Vector3d targetXYZ{ 600.0, 300.0, 300.0 };
  // const Eigen::Vector3d targetABC{ 0.0, 0.0, M_PI };
  // const Eigen::Affine3d targetTransform = kinverse::math::fromXYZABC(targetXYZ, targetABC);

  // create visualizer
  auto kinverseVisualizer = std::make_shared<kinverse::visualization::KinverseVisualizer>();

  auto robotGizmo = std::make_shared<kinverse::visualization::RobotGizmo>(nullptr, robot);
  kinverseVisualizer->addGizmo(robotGizmo);

  auto worldGizmo = std::make_shared<kinverse::visualization::CoordinateFrameGizmo>(nullptr);
  worldGizmo->setCaption("world");
  kinverseVisualizer->addGizmo(worldGizmo);

  auto targetGizmo = std::make_shared<kinverse::visualization::CoordinateFrameGizmo>(nullptr, targetTransform);
  targetGizmo->setCaption("target");
  kinverseVisualizer->addGizmo(targetGizmo);

  // create solver
  auto ikSolver = std::make_shared<kinverse::core::AnalyticalSolver>(robot);

  // solve IK
  const auto configuration = ikSolver->solveUnrefactored(targetTransform);

  // set robot configuration
  robot->setConfiguration(configuration);
  robotGizmo->updateRobotConfiguration();

  // print info
  const Eigen::Affine3d solvedTransform = robot->getLinkCoordinateFrames().back();
  Eigen::Vector3d solvedXYZ;
  Eigen::Vector3d solvedABC;
  kinverse::math::toXYZABC(solvedTransform, solvedXYZ, solvedABC);

  // std::cout << std::endl;
  // std::cout << "Initial configuration:" << std::endl;
  // printConfiguration(robotConfiguration);
  // printEndEffector(targetXYZ, targetABC);
  // std::cout << std::endl;

  // std::cout << "Solved configuration:" << std::endl;
  // printConfiguration(configuration);
  // printEndEffector(solvedXYZ, solvedABC);

  // bool flag = false;
  // while (std::cin.get()) {
  //  flag = !flag;
  //  std::cout << "flag = " << flag << std::endl;

  //  if (flag) {
  //    robot->setConfiguration(configuration);
  //    robotGizmo->setConfiguration(configuration);
  //  } else {
  //    robot->setConfiguration(robotConfiguration);
  //    robotGizmo->setConfiguration(robotConfiguration);
  //  }
  //}

  return 0;
}

/*
1)  class CoordinateFrameGizmo {};
2)  class CylinderGizmo {};
3)  class CubeGizmo {};
4)  class Text3DGizmo {};
5)  class RevoluteJointGizmo {};

6)  class CircularArrowGizmo {};

7)  class LineGizmo {};
8)  class ArrowGizmo {};
9)  class AngleGizmo {};
10)  class DistanceGizmo {};
11) class FrameConnectionGizmo {};
12) class PrismaticJointGizmo {};
13) class EndEffectorGizmo {};
*/

/*
const Eigen::Affine3d targetTransform = convertWorldToLocal(endEffectorTransform);
  const Eigen::Vector3d wristPosition = computeWristPosition(targetTransform);
  const Eigen::Vector3d firstJointZAxis = getFirstJointZAxis();
  if (math::pointLiesOnLine(Eigen::Vector3d::Zero(), firstJointZAxis, wristPosition)) {
    // if wristPosition lies on z0 axis then we have a singularity, in this case theta1 has infinite number of solutions
    throw std::domain_error("We have a singularity! Wrist position lies on z0 axis! theta1 has infinite number of solutions!");
  }

  const auto forwardFacingSolutions = solveForForwardFacing(theta1);
  const auto backwardFacingSolutions = solveForBackwardFacing(theta1 + M_PI);

  // NOTE 2: here we do not consider another possible solution [atan2(y, x) + M_PI]
  const double theta1 = std::atan2(wristPosition.y(), wristPosition.x());

  // now when we know theta1 we can compute position of the second joint
  const Eigen::Vector3d secondJointPosition = m_robot->getDHTable()[0].getTransform(theta1).translation();

  // project wristPosition and secondJointPosition on Oxy plane
  const Eigen::Vector2d wristPositionProjection{ wristPosition.x(), wristPosition.y() };
  const Eigen::Vector2d secondJointPositionProjection{ secondJointPosition.x(), secondJointPosition.y() };

  const double a = (wristPosition - secondJointPosition).norm();
  const double b = std::abs(wristPosition.z() - secondJointPosition.z());
  const double c = (wristPositionProjection - secondJointPositionProjection).norm();

  const double distanceFromSecondToThirdJoint = m_robot->getDHTable()[1].getTransform().translation().norm();
  const double distanceFromThirdJointToWristPosition = (m_robot->getDHTable()[2].getTransform() * m_robot->getDHTable()[3].getTransform()).translation().norm();

  // using Pythagorean theorem we get first equation
  // a * a = b * b + c * c

  // using Law of cosines we get another equation
  // a * a = distanceFromSecondToThirdJoint * distanceFromSecondToThirdJoint + distanceFromThirdJointToWristPosition * distanceFromThirdJointToWristPosition
  // - 2.0 * distanceFromSecondToThirdJoint * distanceFromThirdJointToWristPosition * cosKsi

  // cosKsi = cos(M_PI - gamma - theta3)
  // gamma is the angle between z4 axis and direction from third joint to wrist position
  const double gamma = computeGamma();

  // a * a = distanceFromSecondToThirdJoint * distanceFromSecondToThirdJoint + distanceFromThirdJointToWristPosition * distanceFromThirdJointToWristPosition
  // + 2.0 * distanceFromSecondToThirdJoint * distanceFromThirdJointToWristPosition * cos(gamma + theta3)

  // using Pythagorean trigonometric identity
  // sin * sin + cos * cos = 1

  // cosine = cos(gamma + theta3)
  // sine = sin(gamma + theta3)

  const double cosine = (b * b + c * c - distanceFromSecondToThirdJoint * distanceFromSecondToThirdJoint -
                         distanceFromThirdJointToWristPosition * distanceFromThirdJointToWristPosition) /
                        (2.0 * distanceFromSecondToThirdJoint * distanceFromThirdJointToWristPosition);
  const double sine = std::sqrt(1.0 - cosine * cosine);

  // NOTE 3: here we do not consider another possible solution [atan2(-sine, cosine) + gamma]
  const double theta3 = atan2(sine, cosine) + gamma;

  // const double tanAlpha = b / c;
  // const double tanBetta =
  //    (distanceFromThirdJointToWristPosition * sin(theta3)) / (distanceFromSecondToThirdJoint + distanceFromThirdJointToWristPosition * cos(theta3));

  // const double theta2 = -atan2(b, c) - atan2(distanceFromThirdJointToWristPosition * sin(theta3),
  //                                           distanceFromSecondToThirdJoint + distanceFromThirdJointToWristPosition * cos(theta3));

  const double cosBetta = (distanceFromSecondToThirdJoint * distanceFromSecondToThirdJoint + a * a -
                           distanceFromThirdJointToWristPosition * distanceFromThirdJointToWristPosition) /
                          (2.0 * distanceFromSecondToThirdJoint * a);
  const double theta2 = -atan2(b, c) - acos(cosBetta);

  const Eigen::Matrix3d endEffectorOrientation = targetTransform.rotation();
  const Eigen::Matrix3d wristOrientation = computeWristOrientation(theta1, theta2, theta3);  // it is not wrist, it is the fourth joint
  const Eigen::Matrix3d fromWristToEndEffectorRotation = wristOrientation.transpose() * endEffectorOrientation;

  const double r33 = fromWristToEndEffectorRotation(2, 2);
  const double r23 = fromWristToEndEffectorRotation(1, 2);
  const double r13 = fromWristToEndEffectorRotation(0, 2);
  const double r32 = fromWristToEndEffectorRotation(2, 1);
  const double r31 = fromWristToEndEffectorRotation(2, 0);

  const double theta5 = M_PI + atan2(-std::sqrt(1.0 - r33 * r33), r33);
  const double theta4 = atan2(-r23, -r13);
  const double theta6 = -atan2(-r32, +r31);

  const std::vector<double> configuration{ theta1, theta2, theta3, theta4, theta5, theta6 };
  return configuration;
*/

#include <thread>
int debugVisualizer() {
  auto visualizer = std::make_shared<kinverse::visualization::KinverseVisualizer>();

  auto world = std::make_shared<kinverse::visualization::CoordinateFrameGizmo>();
  world->setCaption("world");
  visualizer->addGizmo(world);

  auto gizmo = std::make_shared<kinverse::visualization::CoordinateFrameGizmo>();
  {
    Eigen::Affine3d transform;
    transform = Eigen::Translation3d(2000.0, 0.0, 0.0);
    gizmo->setTransform(transform);
  }
  visualizer->addGizmo(gizmo);

  const auto getCurrentTime = []() -> long long {
    return static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
  };
  auto updateGizmo = [&]() {
    const auto startTime = getCurrentTime();
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      const auto currentTime = getCurrentTime();
      const auto secondsPassed = (currentTime - startTime) / 1000000.0;

      const double angle = secondsPassed * (5.0 / 180.0) * M_PI;
      const Eigen::Affine3d transform = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(2000.0, 0.0, 0.0);
      std::cout << "before update" << std::endl;
      // gizmo->setTransform(transform);
      std::cout << "after update" << std::endl;
      std::cout << std::endl << std::endl << std::endl << std::endl;

      // gizmo->setAxesLength(100.0);
      // gizmo->setAxesLabels({ "x", "y", "z" });
      gizmo->setCaption("fuck");
      // gizmo->setTransform(Eigen::Affine3d::Identity());
    }
  };
  auto updateThread = std::thread(updateGizmo);

  if (updateThread.joinable())
    updateThread.join();

  return 0;
}
