/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Sergey Ushakov
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

namespace kinverse {
  namespace core {

    class TestDenavitHartenbergParameters : public testing::Test {
     protected:
      Eigen::Matrix4d getDHMatrix(double d, double theta, double r, double alpha, double angle, double distance) const {
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
      Eigen::Matrix4d getZMatrix(double d, double theta, double angle, double distance) const {
        const double cosTheta = cos(theta + angle);
        const double sinTheta = sin(theta + angle);

        Eigen::Matrix4d matrix;
        matrix(0, 0) = cosTheta;
        matrix(0, 1) = -sinTheta;
        matrix(0, 2) = 0.0;
        matrix(0, 3) = 0.0;

        matrix(1, 0) = sinTheta;
        matrix(1, 1) = cosTheta;
        matrix(1, 2) = 0.0;
        matrix(1, 3) = 0.0;

        matrix(2, 0) = 0.0;
        matrix(2, 1) = 0.0;
        matrix(2, 2) = 1.0;
        matrix(2, 3) = d + distance;

        matrix(3, 0) = 0.0;
        matrix(3, 1) = 0.0;
        matrix(3, 2) = 0.0;
        matrix(3, 3) = 1.0;

        return matrix;
      }
      Eigen::Matrix4d getXMatrix(double r, double alpha) const {
        const double cosAlpha = cos(alpha);
        const double sinAlpha = sin(alpha);

        Eigen::Matrix4d matrix;
        matrix(0, 0) = 1.0;
        matrix(0, 1) = 0.0;
        matrix(0, 2) = 0.0;
        matrix(0, 3) = r;

        matrix(1, 0) = 0.0;
        matrix(1, 1) = cosAlpha;
        matrix(1, 2) = -sinAlpha;
        matrix(1, 3) = 0.0;

        matrix(2, 0) = 0.0;
        matrix(2, 1) = sinAlpha;
        matrix(2, 2) = cosAlpha;
        matrix(2, 3) = 0.0;

        matrix(3, 0) = 0.0;
        matrix(3, 1) = 0.0;
        matrix(3, 2) = 0.0;
        matrix(3, 3) = 1.0;

        return matrix;
      }
    };

  }  // namespace core
}  // namespace kinverse
