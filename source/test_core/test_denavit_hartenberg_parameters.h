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

#include <kinverse/core/denavit_hartenberg_parameters.h>

namespace kinverse {
  namespace core {

    using DHParamsEquality = std::tuple<DenavitHartenbergParameters, DenavitHartenbergParameters, bool>;

    class TestDenavitHartenbergParameters : public testing::Test, public testing::WithParamInterface<DHParamsEquality> {
     protected:
      Eigen::Matrix4d getDHMatrix(double d, double theta, double r, double alpha, double angle, double distance) const {
        return getZMatrix(d, theta, angle, distance) * getXMatrix(r, alpha);
      }
      Eigen::Matrix4d getZMatrix(double d, double theta, double angle, double distance) const {
        return getRotationMatrixAboutZAxis(theta + angle) * getZTranslationMatrix(d + distance);
      }
      Eigen::Matrix4d getXMatrix(double r, double alpha) const {
        return getRotationMatrixAboutXAxis(alpha) * getXTranslationMatrix(r);
      }
      Eigen::Matrix4d getRotationMatrixAboutZAxis(double angle) const {
        const double cosine = cos(angle);
        const double sine = sin(angle);

        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        matrix(0, 0) = cosine;
        matrix(0, 1) = -sine;
        matrix(1, 0) = sine;
        matrix(1, 1) = cosine;
        return matrix;
      }
      Eigen::Matrix4d getRotationMatrixAboutXAxis(double angle) const {
        const double cosine = cos(angle);
        const double sine = sin(angle);

        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        matrix(1, 1) = cosine;
        matrix(1, 2) = -sine;
        matrix(2, 1) = sine;
        matrix(2, 2) = cosine;
        return matrix;
      }
      Eigen::Matrix4d getZTranslationMatrix(double displacement) const {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        matrix(2, 3) = displacement;
        return matrix;
      }
      Eigen::Matrix4d getXTranslationMatrix(double displacement) const {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        matrix(0, 3) = displacement;
        return matrix;
      }
    };

    INSTANTIATE_TEST_SUITE_P(EqualityOperator,
                             TestDenavitHartenbergParameters,
                             testing::Values(                                                                                                                //
                                 DHParamsEquality{ { JointType::Prismatic, 1.0, 2.0, 3.0, 4.0 }, { JointType::Prismatic, 1.0, 2.0, 3.0, 4.0 }, true },       //
                                 DHParamsEquality{ { JointType::Prismatic, 1.0, 2.0, 3.0, 4.0 }, { JointType::Revolute, 1.0, 2.0, 3.0, 4.0 }, false },       //
                                 DHParamsEquality{ { JointType::Prismatic, 459.0, 2.0, 3.0, 4.0 }, { JointType::Prismatic, 1.0, 2.0, 3.0, 4.0 }, false },    //
                                 DHParamsEquality{ { JointType::Prismatic, 1.0, 2.0, 3.0, 4.0 }, { JointType::Prismatic, 1.0, 6598.0, 3.0, 4.0 }, false },   //
                                 DHParamsEquality{ { JointType::Prismatic, 1.0, 2.0, -34.082, 4.0 }, { JointType::Prismatic, 1.0, 2.0, 3.0, 4.0 }, false },  //
                                 DHParamsEquality{ { JointType::Prismatic, 1.0, 2.0, 3.0, 4.0 }, { JointType::Prismatic, 1.0, 2.0, 3.0, 4330.304 }, false }  //
                                 ));

  }  // namespace core
}  // namespace kinverse
