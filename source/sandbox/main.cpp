#include "stdafx.h"
#include <visualization/kinverse_visualizer.h>
#include <visualization/coordinate_frame_gizmo.h>
#include <visualization/revolute_joint_gizmo.h>
#include <core/robot.h>

#include <iostream>

int main(int argc, char** argv) {
  // create empty robot
  auto robot = std::make_shared<kinverse::core::Robot>();
  // Robot
  // RobotConfiguration
  // RobotGizmo

  // configure robot joints
  std::vector<kinverse::core::DenavitHartenbergParameters> dhTable{
    { kinverse::core::JointType::Revolute, 0.0, 0.0, 0.0, M_PI },          //
    { kinverse::core::JointType::Revolute, -400.0, 0.0, 180.0, M_PI_2 },   //
    { kinverse::core::JointType::Revolute, 0.0, 0.0, 600.0, 0.0 },         //
    { kinverse::core::JointType::Revolute, 0.0, -M_PI_2, 120.0, M_PI_2 },  //
    { kinverse::core::JointType::Revolute, -620.0, 0.0, 0.0, -M_PI_2 },    //
    { kinverse::core::JointType::Revolute, 0.0, 0.0, 0.0, M_PI_2 }         //
  };
  robot->setDHTable(dhTable);

  // create visualizer with world frame
  const auto worldFrameGizmo = std::make_shared<kinverse::visualization::CoordinateFrameGizmo>(nullptr, Eigen::Affine3d::Identity(), "world");
  worldFrameGizmo->setScale(0.25);
  auto visualizer = std::make_shared<kinverse::visualization::KinverseVisualizer>();
  visualizer->addGizmo(worldFrameGizmo);

  // draw robot joints
  const auto joints = robot->getJointCoordinateFrames();
  const auto numberOfJoints = joints.size();
  for (auto jointCounter = 0u; jointCounter < numberOfJoints; ++jointCounter) {
    auto revoluteJointGizmo = std::make_shared<kinverse::visualization::RevoluteJointGizmo>(nullptr, joints[jointCounter], jointCounter);
    visualizer->addGizmo(revoluteJointGizmo);
  }

  int ggg;
  std::cin >> ggg;

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

int main(int argc, char** argv) {
        //create empty robot
        auto robot = std::make_shared<kinverse::Robot>();

        //lets configure it
        robot->addJoint(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d{ 0.0, 0.0, 0.0 }, kinverse::JointType::Revolute, "A1");
        robot->addJoint(Eigen::Vector3d::UnitY(), Eigen::Vector3d{ 180.0, 0.0, 400.0 }, kinverse::JointType::Revolute, "A2");
        robot->addJoint(Eigen::Vector3d::UnitY(), Eigen::Vector3d{ 780.0, 0.0, 400.0 }, kinverse::JointType::Revolute, "A3");
        robot->addJoint(-Eigen::Vector3d::UnitX(), Eigen::Vector3d{ 780.0, 0.0, 520.0 }, kinverse::JointType::Revolute, "A4");
        robot->addJoint(Eigen::Vector3d::UnitY(), Eigen::Vector3d{ 1400.0, 0.0, 520.0 }, kinverse::JointType::Revolute, "A5");
        robot->addJoint(-Eigen::Vector3d::UnitX(), Eigen::Vector3d{ 1400.0, 0.0, 520.0 }, kinverse::JointType::Revolute, "A6");

        //robot->getDenaviteHartenbergParameters();
        //	robot->setEndEffectorDisplacement();
        //	robot->addJoint(Eigen::Vector3d::Zero(), Eigen::Vector3d{ 1515.0, 0.0, 520.0 }, kinverse::JointType::EndEffector);
        //	robot->addJoint(kinverse::JointType::Revolute);
        return 0;
}
*/
