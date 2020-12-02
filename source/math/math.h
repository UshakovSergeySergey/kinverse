#pragma once

#include "exports.h"

namespace kinverse {
  namespace math {

    double KINVERSE_MATH_API degreesToRadians(double angleInDegrees);
    double KINVERSE_MATH_API radiansToDegrees(double angleInRadians);
    void KINVERSE_MATH_API toXYZABC(const Eigen::Affine3d& transform, Eigen::Vector3d& xyz, Eigen::Vector3d& abc);
    Eigen::Affine3d KINVERSE_MATH_API fromXYZABC(const Eigen::Vector3d& xyz, const Eigen::Vector3d& abc);
    bool KINVERSE_MATH_API pointLiesOnLine(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const Eigen::Vector3d& point);

  }  // namespace math
}  // namespace kinverse
