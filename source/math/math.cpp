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

double kinverse::math::clamp(double value, double min, double max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}
