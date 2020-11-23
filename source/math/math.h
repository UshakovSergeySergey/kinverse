#pragma once

#include "exports.h"

namespace kinverse {
  namespace math {

    double KINVERSE_MATH_API degreesToRadians(double angleInDegrees);
    double KINVERSE_MATH_API radiansToDegrees(double angleInRadians);
    double KINVERSE_MATH_API clamp(double value, double min, double max);

  }  // namespace math
}  // namespace kinverse
