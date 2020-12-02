#include "stdafx.h"
#include "math.h"

double kinverse::math::degreesToRadians(double angleInDegrees) {
  constexpr double toRadians = M_PI / 180.0;
  return angleInDegrees * toRadians;
}

double kinverse::math::radiansToDegrees(double angleInRadians) {
  constexpr double toDegrees = 180.0 * M_1_PI;
  return angleInRadians * toDegrees;
}

void kinverse::math::toXYZABC(const Eigen::Affine3d& transform, Eigen::Vector3d& xyz, Eigen::Vector3d& abc) {
  xyz = transform.translation();
  abc = Eigen::EulerAnglesZYXd(transform.rotation()).angles();
}

Eigen::Affine3d kinverse::math::fromXYZABC(const Eigen::Vector3d& xyz, const Eigen::Vector3d& abc) {
  return Eigen::Translation3d(xyz) * Eigen::EulerAnglesZYXd(abc.x(), abc.y(), abc.z());
}

bool kinverse::math::pointLiesOnLine(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const Eigen::Vector3d& point) {
  throw std::exception("Not implemented yet!");
}
