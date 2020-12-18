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
  //(facing backward; elbow down)
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
  std::vector<std::vector<double>> positionalSolutions = solveForPosition(endEffectorTransform);
  return positionalSolutions;
}

std::vector<std::vector<double>> kinverse::core::AnalyticalSolver::solveForPosition(const Eigen::Affine3d& endEffectorTransform) const {
  auto facingForwardElbowUp = getFacingForwardElbowUpSolution(endEffectorTransform);
  auto facingForwardElbowDown = getFacingForwardElbowDownSolution(endEffectorTransform);
  auto facingBackwardElbowUp = getFacingBackwardElbowUpSolution(endEffectorTransform);
  auto facingBackwardElbowDown = getFacingBackwardElbowDownSolution(endEffectorTransform);

  return {
    facingForwardElbowUp,    //
    facingForwardElbowDown,  //
    facingBackwardElbowUp,   //
    facingBackwardElbowDown  //
  };
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

std::vector<double> kinverse::core::AnalyticalSolver::getFacingForwardElbowUpSolution(const Eigen::Affine3d& endEffectorTransform) const {
  const Eigen::Affine3d targetTransform = convertWorldToLocal(endEffectorTransform);
  const Eigen::Vector3d wristPosition = computeWristPosition(targetTransform);
  const Eigen::Vector3d firstJointZAxis = getFirstJointZAxis();

  //@todo uncomment this
  // if (math::pointLiesOnLine(Eigen::Vector3d::Zero(), firstJointZAxis, wristPosition)) {
  //  // shoulder singularity (KUKA calls it overhead singularity)
  //  throw std::domain_error("We have a singularity! Wrist position lies on z0 axis! theta1 has infinite number of solutions!");
  //}

  const double theta1 = std::atan2(wristPosition.y(), wristPosition.x());

  // now when we know theta1 we can compute position of the second joint
  const Eigen::Vector3d secondJointPosition = computeSecondJointPosition(theta1);

  // project wristPosition and secondJointPosition on Oxy plane
  const Eigen::Vector2d wristPositionProjection{ wristPosition.x(), wristPosition.y() };
  const Eigen::Vector2d secondJointPositionProjection{ secondJointPosition.x(), secondJointPosition.y() };

  // using Pythagorean theorem we get first equation
  // a * a = b * b + c * c
  const double a = (wristPosition - secondJointPosition).norm();
  const double b = secondJointPosition.z() - wristPosition.z();
  const double c = (wristPositionProjection - secondJointPositionProjection).norm();

  // using Law of cosines we get another equation
  // a * a = d * d + e * e - 2.0 * d * e * cosKsi
  const double d = getDistanceBetweenSecondAndThirdJoints();
  const double e = getDistanceBetweenThirdJointAndWristPosition();

  // remember that theta3 is positive because we consider elbow up case =>
  // cosKsi = cos(M_PI + gamma - theta3)
  // where gamma is the angle between z4 axis and direction from third joint to wrist position
  const double gamma = computeGamma();

  // using previous considerations we get final equation
  // a * a = d * d + e * e + 2.0 * d * e * cos(gamma + theta3)

  // we will also need Pythagorean trigonometric identity
  // sine * sine + cosine * cosine = 1
  //
  // where sine and cosine are as follows
  // cosine = cos(gamma - theta3)
  // sine = sin(gamma - theta3)

  const double cosine = (b * b + c * c - d * d - e * e) / (2.0 * d * e);
  const double sine = std::sqrt(1.0 - cosine * cosine);

  const double theta3 = atan2(sine, cosine) + gamma;

  // remember that theta2 is negative because we consider elbow up case
  // theta2 = -(alpha + beta)
  //
  // here we assume that alpha and beta are positive angles of triangles ABC and ADE
  //
  // tan(alpha) = b / c     => alpha = atan2(b, c)
  // tan(beta) = e * sine / (d + e * cosine)
  //
  // where sine and cosine are as follows
  // cosine = cos(gamma - theta3)
  // sine = sin(gamma - theta3)
  //
  //
  //
  // e * e = d * d + a * a + 2.0 * d * a * cos(beta)
  // cos(beta) = (e * e - d * d - a * a) / (2.0 * d * a)
  const double alpha = atan2(b, c);
  const double beta = atan2(e * sine, d + e * cosine);
  const double theta2 = -(alpha + beta);

  return { theta1, theta2, theta3, 0.0, 0.0, 0.0 };
}

std::vector<double> kinverse::core::AnalyticalSolver::getFacingForwardElbowDownSolution(const Eigen::Affine3d& endEffectorTransform) const {
  const Eigen::Affine3d targetTransform = convertWorldToLocal(endEffectorTransform);
  const Eigen::Vector3d wristPosition = computeWristPosition(targetTransform);
  const Eigen::Vector3d firstJointZAxis = getFirstJointZAxis();

  //@todo uncomment this
  // if (math::pointLiesOnLine(Eigen::Vector3d::Zero(), firstJointZAxis, wristPosition)) {
  //  // shoulder singularity (KUKA calls it overhead singularity)
  //  throw std::domain_error("We have a singularity! Wrist position lies on z0 axis! theta1 has infinite number of solutions!");
  //}

  const double theta1 = std::atan2(wristPosition.y(), wristPosition.x());

  // now when we know theta1 we can compute position of the second joint
  const Eigen::Vector3d secondJointPosition = computeSecondJointPosition(theta1);

  // project wristPosition and secondJointPosition on Oxy plane
  const Eigen::Vector2d wristPositionProjection{ wristPosition.x(), wristPosition.y() };
  const Eigen::Vector2d secondJointPositionProjection{ secondJointPosition.x(), secondJointPosition.y() };

  // using Pythagorean theorem we get first equation
  // a * a = b * b + c * c
  const double a = (wristPosition - secondJointPosition).norm();
  const double b = secondJointPosition.z() - wristPosition.z();
  const double c = (wristPositionProjection - secondJointPositionProjection).norm();

  // using Law of cosines we get another equation
  // a * a = d * d + e * e - 2.0 * d * e * cosKsi
  const double d = getDistanceBetweenSecondAndThirdJoints();
  const double e = getDistanceBetweenThirdJointAndWristPosition();

  // remember that theta3 is positive because we consider elbow down case =>
  // cosKsi = cos(M_PI + gamma - theta3)
  // where gamma is the angle between z4 axis and direction from third joint to wrist position
  const double gamma = computeGamma();

  // using previous considerations we get final equation
  // a * a = d * d + e * e + 2.0 * d * e * cos(gamma + theta3)

  // we will also need Pythagorean trigonometric identity
  // sine * sine + cosine * cosine = 1
  //
  // where sine and cosine are as follows
  // cosine = cos(gamma - theta3)
  // sine = sin(gamma - theta3)

  const double cosine = (b * b + c * c - d * d - e * e) / (2.0 * d * e);
  const double sine = -std::sqrt(1.0 - cosine * cosine);

  const double theta3 = atan2(sine, cosine) + gamma;

  // remember that theta2 is negative because we consider elbow down case
  // theta2 = -(alpha + beta)
  //
  // here we assume that alpha and beta are positive angles of triangles ABC and ADE
  //
  // tan(alpha) = b / c     => alpha = atan2(b, c)
  // tan(beta) = e * sine / (d + e * cosine)
  //
  // where sine and cosine are as follows
  // cosine = cos(gamma - theta3)
  // sine = sin(gamma - theta3)
  //
  //
  //
  // e * e = d * d + a * a + 2.0 * d * a * cos(beta)
  // cos(beta) = (e * e - d * d - a * a) / (2.0 * d * a)
  const double alpha = atan2(b, c);
  const double beta = atan2(e * sine, d + e * cosine);
  const double theta2 = -(alpha + beta);

  return { theta1, theta2, theta3, 0.0, 0.0, 0.0 };
}

std::vector<double> kinverse::core::AnalyticalSolver::getFacingBackwardElbowUpSolution(const Eigen::Affine3d& endEffectorTransform) const {
  const Eigen::Affine3d targetTransform = convertWorldToLocal(endEffectorTransform);
  const Eigen::Vector3d wristPosition = computeWristPosition(targetTransform);
  const Eigen::Vector3d firstJointZAxis = getFirstJointZAxis();

  //@todo uncomment this
  // if (math::pointLiesOnLine(Eigen::Vector3d::Zero(), firstJointZAxis, wristPosition)) {
  //  // shoulder singularity (KUKA calls it overhead singularity)
  //  throw std::domain_error("We have a singularity! Wrist position lies on z0 axis! theta1 has infinite number of solutions!");
  //}

  const double theta1 = std::atan2(wristPosition.y(), wristPosition.x()) + M_PI;

  // now when we know theta1 we can compute position of the second joint
  const Eigen::Vector3d secondJointPosition = computeSecondJointPosition(theta1);

  // project wristPosition and secondJointPosition on Oxy plane
  const Eigen::Vector2d wristPositionProjection{ wristPosition.x(), wristPosition.y() };
  const Eigen::Vector2d secondJointPositionProjection{ secondJointPosition.x(), secondJointPosition.y() };

  // using Pythagorean theorem we get first equation
  // a * a = b * b + c * c
  const double a = (wristPosition - secondJointPosition).norm();
  const double b = secondJointPosition.z() - wristPosition.z();
  const double c = (wristPositionProjection - secondJointPositionProjection).norm();

  // using Law of cosines we get another equation
  // a * a = d * d + e * e - 2.0 * d * e * cosKsi
  const double d = getDistanceBetweenSecondAndThirdJoints();
  const double e = getDistanceBetweenThirdJointAndWristPosition();

  // remember that theta3 is negative because we consider elbow up case =>
  // cosKsi = cos(M_PI + gamma + theta3)
  // where gamma is the angle between z4 axis and direction from third joint to wrist position
  const double gamma = computeGamma();

  // using previous considerations we get final equation
  // a * a = d * d + e * e + 2.0 * d * e * cos(gamma + theta3)

  // we will also need Pythagorean trigonometric identity
  // sine * sine + cosine * cosine = 1
  //
  // where sine and cosine are as follows
  // cosine = cos(gamma - theta3)
  // sine = sin(gamma - theta3)

  const double cosine = (b * b + c * c - d * d - e * e) / (2.0 * d * e);
  const double sine = std::sqrt(1.0 - cosine * cosine);

  const double theta3 = -atan2(sine, cosine) + gamma;

  // remember that theta2 is negative because we consider elbow up case
  // theta2 = -(alpha + beta)
  //
  // here we assume that alpha and beta are positive angles of triangles ABC and ADE
  //
  // tan(alpha) = b / c     => alpha = atan2(b, c)
  // tan(beta) = e * sine / (d + e * cosine)
  //
  // where sine and cosine are as follows
  // cosine = cos(gamma - theta3)
  // sine = sin(gamma - theta3)
  //
  //
  //
  // e * e = d * d + a * a + 2.0 * d * a * cos(beta)
  // cos(beta) = (e * e - d * d - a * a) / (2.0 * d * a)
  const double alpha = atan2(b, c);
  const double beta = atan2(e * sine, d + e * cosine);
  const double theta2 = (alpha + beta) - M_PI;

  return { theta1, theta2, theta3, 0.0, 0.0, 0.0 };
}

std::vector<double> kinverse::core::AnalyticalSolver::getFacingBackwardElbowDownSolution(const Eigen::Affine3d& endEffectorTransform) const {
  const Eigen::Affine3d targetTransform = convertWorldToLocal(endEffectorTransform);
  const Eigen::Vector3d wristPosition = computeWristPosition(targetTransform);
  const Eigen::Vector3d firstJointZAxis = getFirstJointZAxis();

  //@todo uncomment this
  // if (math::pointLiesOnLine(Eigen::Vector3d::Zero(), firstJointZAxis, wristPosition)) {
  //  // shoulder singularity (KUKA calls it overhead singularity)
  //  throw std::domain_error("We have a singularity! Wrist position lies on z0 axis! theta1 has infinite number of solutions!");
  //}

  const double theta1 = std::atan2(wristPosition.y(), wristPosition.x()) + M_PI;

  // now when we know theta1 we can compute position of the second joint
  const Eigen::Vector3d secondJointPosition = computeSecondJointPosition(theta1);

  // project wristPosition and secondJointPosition on Oxy plane
  const Eigen::Vector2d wristPositionProjection{ wristPosition.x(), wristPosition.y() };
  const Eigen::Vector2d secondJointPositionProjection{ secondJointPosition.x(), secondJointPosition.y() };

  // using Pythagorean theorem we get first equation
  // a * a = b * b + c * c
  const double a = (wristPosition - secondJointPosition).norm();
  const double b = secondJointPosition.z() - wristPosition.z();
  const double c = (wristPositionProjection - secondJointPositionProjection).norm();

  // using Law of cosines we get another equation
  // a * a = d * d + e * e - 2.0 * d * e * cosKsi
  const double d = getDistanceBetweenSecondAndThirdJoints();
  const double e = getDistanceBetweenThirdJointAndWristPosition();

  // remember that theta3 is negative because we consider elbow up case =>
  // cosKsi = cos(M_PI + gamma + theta3)
  // where gamma is the angle between z4 axis and direction from third joint to wrist position
  const double gamma = computeGamma();

  // using previous considerations we get final equation
  // a * a = d * d + e * e + 2.0 * d * e * cos(gamma + theta3)

  // we will also need Pythagorean trigonometric identity
  // sine * sine + cosine * cosine = 1
  //
  // where sine and cosine are as follows
  // cosine = cos(gamma - theta3)
  // sine = sin(gamma - theta3)

  const double cosine = (b * b + c * c - d * d - e * e) / (2.0 * d * e);
  const double sine = -std::sqrt(1.0 - cosine * cosine);

  const double theta3 = -atan2(sine, cosine) + gamma;

  // remember that theta2 is negative because we consider elbow up case
  // theta2 = -(alpha + beta)
  //
  // here we assume that alpha and beta are positive angles of triangles ABC and ADE
  //
  // tan(alpha) = b / c     => alpha = atan2(b, c)
  // tan(beta) = e * sine / (d + e * cosine)
  //
  // where sine and cosine are as follows
  // cosine = cos(gamma - theta3)
  // sine = sin(gamma - theta3)
  //
  //
  //
  // e * e = d * d + a * a + 2.0 * d * a * cos(beta)
  // cos(beta) = (e * e - d * d - a * a) / (2.0 * d * a)
  const double alpha = atan2(b, c);
  const double beta = atan2(e * sine, d + e * cosine);
  const double theta2 = (alpha + beta) - M_PI;

  return { theta1, theta2, theta3, 0.0, 0.0, 0.0 };
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

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

Eigen::Vector3d kinverse::core::AnalyticalSolver::computeSecondJointPosition(double theta1) const {
  return m_robot->getDHTable().front().getTransform(theta1).translation();
}

double kinverse::core::AnalyticalSolver::getDistanceBetweenSecondAndThirdJoints() const {
  return m_robot->getDHTable()[1].getTransform().translation().norm();
}

double kinverse::core::AnalyticalSolver::getDistanceBetweenThirdJointAndWristPosition() const {
  return (m_robot->getDHTable()[2].getTransform() * m_robot->getDHTable()[3].getTransform()).translation().norm();
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
