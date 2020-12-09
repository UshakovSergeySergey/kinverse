#include "stdafx.h"
#include "../include/kinverse/core/analytical_solver.h"
#include <kinverse/math/math.h>

kinverse::core::AnalyticalSolver::AnalyticalSolver(Robot::ConstPtr robot) : m_robot{ robot } {
}

std::vector<double> kinverse::core::AnalyticalSolver::solveUnrefactored(const Eigen::Affine3d& endEffectorTransform) const {
  // we have 4 possible solutions for position
  //(facing forward; elbow up)
  //(facing forward; elbow down)
  //(facing backward; elbow up)
  //(facing bacward; elbow down)
  using RobotConfiguration = std::vector<double>;
  std::vector<RobotConfiguration> solutions(4);

  const Eigen::Affine3d targetTransform = convertWorldToLocal(endEffectorTransform);
  const Eigen::Vector3d wristPosition = computeWristPosition(targetTransform);
  const Eigen::Vector3d firstJointZAxis = getFirstJointZAxis();
  // if (math::pointLiesOnLine(Eigen::Vector3d::Zero(), firstJointZAxis, wristPosition)) {
  //  throw std::domain_error("We have a singularity! Wrist position lies on z0 axis! theta1 has infinite number of solutions!");
  //}

  // NOTE 1: here we do not consider another possible solution [atan2(y, x) + M_PI]
  const double theta1 = std::atan2(wristPosition.y(), wristPosition.x());

  // now when we know theta1 we can compute position of the second joint
  const Eigen::Vector3d secondJointPosition = m_robot->getDHTable().front().getTransform(theta1).translation();

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
}

std::vector<std::vector<double>> kinverse::core::AnalyticalSolver::solve(const Eigen::Affine3d& endEffectorTransform) const {
  const Eigen::Affine3d targetTransform = convertWorldToLocal(endEffectorTransform);
  const Eigen::Vector3d wristPosition = computeWristPosition(targetTransform);
  const Eigen::Vector3d firstJointZAxis = getFirstJointZAxis();
  if (math::pointLiesOnLine(Eigen::Vector3d::Zero(), firstJointZAxis, wristPosition)) {
    throw std::domain_error("We have a singularity! Wrist position lies on z0 axis! theta1 has infinite number of solutions!");
  }

  const double theta1 = std::atan2(wristPosition.y(), wristPosition.x());

  // const auto forwardFacingSolutions = solveForForwardFacing(theta1);
  // const auto backwardFacingSolutions = solveForBackwardFacing(theta1 + M_PI);

  // std::vector<std::vector<double>> solutions{};
  // solutions.insert(solutions.end(), forwardFacingSolutions.begin(), forwardFacingSolutions.end());
  // solutions.insert(solutions.end(), backwardFacingSolutions.begin(), backwardFacingSolutions.end());

  // return solutions;
  return {};
}

Eigen::Affine3d kinverse::core::AnalyticalSolver::convertWorldToLocal(const Eigen::Affine3d& transform) const {
  return m_robot->getBaseTransform().inverse() * transform;
}

Eigen::Vector3d kinverse::core::AnalyticalSolver::computeWristPosition(const Eigen::Affine3d& targetTransform) const {
  const Eigen::Vector3d endEffectorPosition = targetTransform.translation();
  const Eigen::Vector3d endEffectorZAxis = targetTransform.rotation().col(2);
  const double wristToEndEffectorDisplacement = m_robot->getDHTable().back().getTransform().translation().norm();
  const Eigen::Vector3d wristPosition = endEffectorPosition - endEffectorZAxis * wristToEndEffectorDisplacement;
  return wristPosition;
}

Eigen::Vector3d kinverse::core::AnalyticalSolver::getFirstJointZAxis() const {
  return m_robot->getDHTable().front().getTransform().rotation().col(2);
}

double kinverse::core::AnalyticalSolver::computeGamma() const {
  const Eigen::Vector3d fourthJointPosition = m_robot->getDHTable()[2].getTransform().translation();
  const Eigen::Vector3d wristPosition = (m_robot->getDHTable()[2].getTransform() * m_robot->getDHTable()[3].getTransform()).translation();

  const double a = (wristPosition - fourthJointPosition).norm();
  const double c = wristPosition.norm();

  const double cosine = a / c;
  double angle = std::abs(acos(cosine));
  if (angle > M_PI_2)
    angle = M_PI - angle;

  return angle;
};

Eigen::Matrix3d kinverse::core::AnalyticalSolver::computeWristOrientation(double theta1, double theta2, double theta3) const {
  const auto dhTable = m_robot->getDHTable();
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();

  transform = transform * dhTable[0].getTransform(theta1);
  transform = transform * dhTable[1].getTransform(theta2);
  transform = transform * dhTable[2].getTransform(theta3);

  return transform.rotation();
};
