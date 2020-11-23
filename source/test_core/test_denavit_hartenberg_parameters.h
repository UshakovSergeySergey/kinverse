#pragma once

namespace kinverse {
  namespace core {

    class TestDenavitHartenbergParameters : public testing::Test {
     protected:
      Eigen::Matrix4d getDHMatrix(double d, double theta, double r, double alpha, double angle, double distance) {
        const double cosTheta = cos(theta + angle);
        const double sinTheta = sin(theta + angle);
        const double cosAlpha = cos(alpha);
        const double sinAlpha = sin(alpha);

        Eigen::Matrix4d matrix;
        matrix(0, 0) = cosTheta;
        matrix(0, 1) = -sinTheta * cosAlpha;
        matrix(0, 2) = sinTheta * sinAlpha;
        matrix(0, 3) = r * cosTheta;

        matrix(1, 0) = sinTheta;
        matrix(1, 1) = cosTheta * cosAlpha;
        matrix(1, 2) = -cosTheta * sinAlpha;
        matrix(1, 3) = r * sinTheta;

        matrix(2, 0) = 0.0;
        matrix(2, 1) = sinAlpha;
        matrix(2, 2) = cosAlpha;
        matrix(2, 3) = d + distance;

        matrix(3, 0) = 0.0;
        matrix(3, 1) = 0.0;
        matrix(3, 2) = 0.0;
        matrix(3, 3) = 1.0;

        return matrix;
      }
    };

  }  // namespace core
}  // namespace kinverse
