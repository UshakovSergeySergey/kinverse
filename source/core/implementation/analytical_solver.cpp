﻿#include "stdafx.h"
#include "../include/kinverse/core/analytical_solver.h"
#include <kinverse/math/math.h>
#include "analytical_solver_implementation.h"

// SphericalWristRobotArm
// ArticulatedArm6DofRobot
// Robot6Dof
//
// enum class SolutionType {
//  Correct,
//  PointIsUnreachable,
//  WristSingularity,
//  ShoulderSingularity,
//  ElbowSingularity
//};
//
// class Solution {
//  std::vector<double> m_configuration{};
//  SolutionType m_type{};
//};

kinverse::core::AnalyticalSolver::AnalyticalSolver(Robot::ConstPtr robot) : m_robot{ robot } {
  m_pImpl = std::make_shared<AnalyticalSolverImplementation>(robot);
}

std::vector<std::vector<double>> kinverse::core::AnalyticalSolver::solve(const Eigen::Affine3d& tcpTransform) {
  if (tcpTransform.matrix().allFinite() == false)
    throw std::domain_error("Failed to solve IK problem! TCP transform must have only finite values!");

  m_flangeTransform = m_pImpl->convertWorldToA1Local(tcpTransform);
  m_flangeTransform = m_pImpl->convertTCPToFlange(m_flangeTransform);

  m_wristPosition = computeWristPosition(m_flangeTransform);

  std::vector<std::vector<double>> positionalSolutions = solveForPosition();

  std::vector<std::vector<double>> solutions{};
  for (const auto& positionalSolution : positionalSolutions) {
    auto orientedSolutions = solveForOrientation(positionalSolution);
    solutions.insert(solutions.end(), orientedSolutions.begin(), orientedSolutions.end());
  }

  return solutions;
}

Eigen::Vector3d kinverse::core::AnalyticalSolver::computeWristPosition(const Eigen::Affine3d& targetTransform) const {
  const Eigen::Vector3d endEffectorPosition = targetTransform.translation();
  const Eigen::Vector3d endEffectorZAxis = targetTransform.rotation().col(2);
  const double wristToEndEffectorDisplacement = std::abs(m_robot->getDHTable().back().getZAxisDisplacement());
  const Eigen::Vector3d wristPosition = endEffectorPosition - endEffectorZAxis * wristToEndEffectorDisplacement;
  return wristPosition;
}

std::vector<std::vector<double>> kinverse::core::AnalyticalSolver::solveForPosition() const {
  const Eigen::Vector3d firstJointZAxis = m_pImpl->getA1ZAxis();
  if (math::pointLiesOnLine(Eigen::Vector3d::Zero(), firstJointZAxis, m_wristPosition)) {
    // shoulder singularity (KUKA calls it overhead singularity)
    throw std::domain_error("We have a singularity! Wrist position lies on z0 axis! theta1 has infinite number of solutions!");
  }

  const double theta1 = std::atan2(m_wristPosition.y(), m_wristPosition.x());

  const auto forwardSolutions = solvePosition(theta1, true);
  const auto backwardSolutions = solvePosition(theta1 + M_PI, false);

  std::vector<std::vector<double>> positionalSolutions{};
  positionalSolutions.insert(positionalSolutions.end(), forwardSolutions.begin(), forwardSolutions.end());
  positionalSolutions.insert(positionalSolutions.end(), backwardSolutions.begin(), backwardSolutions.end());
  return positionalSolutions;
}

std::vector<std::vector<double>> kinverse::core::AnalyticalSolver::solvePosition(double theta1, bool facingForward) const {
  double a, b, c;
  getTriangle(theta1, a, b, c);

  const double cosine = (b * b + c * c - m_pImpl->getDistanceFromA3ToA2() * m_pImpl->getDistanceFromA3ToA2() -
                         m_pImpl->getDistanceFromA3ToWrist() * m_pImpl->getDistanceFromA3ToWrist()) /
                        (2.0 * m_pImpl->getDistanceFromA3ToA2() * m_pImpl->getDistanceFromA3ToWrist());
  const double sine = std::sqrt(1.0 - cosine * cosine);
  const double alpha = atan2(b, c);

  const auto solveForFacingForward = [&](double sine) -> std::vector<double> {
    const double theta3 = atan2(sine, cosine) + m_pImpl->getGamma();
    const double beta = atan2(m_pImpl->getDistanceFromA3ToWrist() * sine, m_pImpl->getDistanceFromA3ToA2() + m_pImpl->getDistanceFromA3ToWrist() * cosine);
    const double theta2 = -(alpha + beta);
    return { theta1, theta2, theta3, 0.0, 0.0, 0.0 };
  };

  const auto solveForFacingBackward = [&](double sine) -> std::vector<double> {
    const double theta3 = -atan2(sine, cosine) + m_pImpl->getGamma();
    const double beta = atan2(m_pImpl->getDistanceFromA3ToWrist() * sine, m_pImpl->getDistanceFromA3ToA2() + m_pImpl->getDistanceFromA3ToWrist() * cosine);
    const double theta2 = (alpha + beta) - M_PI;
    return { theta1, theta2, theta3, 0.0, 0.0, 0.0 };
  };

  if (facingForward)
    return { solveForFacingForward(sine), solveForFacingForward(-sine) };
  else
    return { solveForFacingBackward(sine), solveForFacingBackward(-sine) };
}

void kinverse::core::AnalyticalSolver::getTriangle(double theta1, double& a, double& b, double& c) const {
  const Eigen::Vector3d secondJointPosition = computeSecondJointPosition(theta1);
  const Eigen::Vector2d wristPositionProjection{ m_wristPosition.x(), m_wristPosition.y() };
  const Eigen::Vector2d secondJointPositionProjection{ secondJointPosition.x(), secondJointPosition.y() };

  a = (m_wristPosition - secondJointPosition).norm();
  b = secondJointPosition.z() - m_wristPosition.z();
  c = (wristPositionProjection - secondJointPositionProjection).norm();
}

Eigen::Vector3d kinverse::core::AnalyticalSolver::computeSecondJointPosition(double theta1) const {
  return m_robot->getDHTable().front().getTransform(theta1).translation();
}

std::vector<std::vector<double>> kinverse::core::AnalyticalSolver::solveForOrientation(const std::vector<double>& configuration) const {
  double theta1 = configuration[0];
  double theta2 = configuration[1];
  double theta3 = configuration[2];

  const Eigen::Matrix3d endEffectorOrientation = m_flangeTransform.rotation();
  const Eigen::Matrix3d wristOrientation = computeWristOrientation(theta1, theta2, theta3);  // it is not wrist, it is the fourth joint
  const Eigen::Matrix3d fromWristToEndEffectorRotation = wristOrientation.transpose() * endEffectorOrientation;

  const double r33 = fromWristToEndEffectorRotation(2, 2);
  const double r23 = fromWristToEndEffectorRotation(1, 2);
  const double r13 = fromWristToEndEffectorRotation(0, 2);
  const double r32 = fromWristToEndEffectorRotation(2, 1);
  const double r31 = fromWristToEndEffectorRotation(2, 0);

  std::vector<double> firstSolution{};
  {
    const double theta5 = M_PI + atan2(-std::sqrt(1.0 - r33 * r33), r33);
    const double theta4 = atan2(-r23, -r13);
    const double theta6 = -atan2(-r32, +r31);
    firstSolution = { theta1, theta2, theta3, theta4, theta5, theta6 };
  }

  std::vector<double> secondSolution{};
  {
    const double theta5 = M_PI + atan2(+std::sqrt(1.0 - r33 * r33), r33);
    const double theta4 = atan2(+r23, +r13);
    const double theta6 = -atan2(+r32, -r31);
    secondSolution = { theta1, theta2, theta3, theta4, theta5, theta6 };
  }

  return { firstSolution, secondSolution };
}

Eigen::Matrix3d kinverse::core::AnalyticalSolver::computeWristOrientation(double theta1, double theta2, double theta3) const {
  const auto dhTable = m_robot->getDHTable();
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();

  transform = transform * dhTable[0].getTransform(theta1);
  transform = transform * dhTable[1].getTransform(theta2);
  transform = transform * dhTable[2].getTransform(theta3);

  return transform.rotation();
}
