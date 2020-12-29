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

#include "stdafx.h"
#include "test_denavit_hartenberg_parameters.h"

namespace kinverse {
  namespace core {

    TEST_F(TestDenavitHartenbergParameters, GetJointType_WhenDefaultConstructed_ReturnsRevolute) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act
      const auto jointType = dhParameters.getJointType();

      // assert
      EXPECT_EQ(jointType, JointType::Revolute);
    }

    TEST_F(TestDenavitHartenbergParameters, GetJointType_WhenJointTypeIsSet_ReturnsProvidedJointType) {
      // arrange
      const auto expectedJointType = JointType::Prismatic;
      DenavitHartenbergParameters dhParameters;
      dhParameters.setJointType(expectedJointType);

      // act
      const auto jointType = dhParameters.getJointType();

      // assert
      EXPECT_EQ(jointType, expectedJointType);
    }

    TEST_F(TestDenavitHartenbergParameters, GetZAxisDisplacement_WhenDefaultConstructed_ReturnsZero) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act
      const auto zAxisDisplacement = dhParameters.getZAxisDisplacement();

      // assert
      EXPECT_DOUBLE_EQ(zAxisDisplacement, 0.0);
    }

    TEST_F(TestDenavitHartenbergParameters, GetZAxisDisplacement_WhenZAxisDisplacementIsSet_ReturnsProvidedDisplacement) {
      // arrange
      const double expectedZAxisDisplacement = 1.23456789;
      DenavitHartenbergParameters dhParameters;
      dhParameters.setZAxisDisplacement(expectedZAxisDisplacement);

      // act
      const auto zAxisDisplacement = dhParameters.getZAxisDisplacement();

      // assert
      EXPECT_DOUBLE_EQ(zAxisDisplacement, expectedZAxisDisplacement);
    }

    TEST_F(TestDenavitHartenbergParameters, SetZAxisDisplacement_WhenDisplacementIsNotFinite_ThrowsException) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act assert
      EXPECT_THROW(dhParameters.setZAxisDisplacement(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(dhParameters.setZAxisDisplacement(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestDenavitHartenbergParameters, GetZAxisRotation_WhenDefaultConstructed_ReturnsZero) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act
      const auto zAxisRotation = dhParameters.getZAxisRotation();

      // assert
      EXPECT_DOUBLE_EQ(zAxisRotation, 0.0);
    }

    TEST_F(TestDenavitHartenbergParameters, GetZAxisRotation_WhenZAxisRotationIsSet_ReturnsProvidedRotation) {
      // arrange
      const double expectedZAxisRotation = 3.231251;
      DenavitHartenbergParameters dhParameters;
      dhParameters.setZAxisRotation(expectedZAxisRotation);

      // act
      const auto zAxisRotation = dhParameters.getZAxisRotation();

      // assert
      EXPECT_DOUBLE_EQ(zAxisRotation, expectedZAxisRotation);
    }

    TEST_F(TestDenavitHartenbergParameters, SetZAxisRotation_WhenAngleIsNotFinite_ThrowsException) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act assert
      EXPECT_THROW(dhParameters.setZAxisRotation(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(dhParameters.setZAxisRotation(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestDenavitHartenbergParameters, GetXAxisDisplacement_WhenDefaultConstructed_ReturnsZero) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act
      const auto xAxisDisplacement = dhParameters.getXAxisDisplacement();

      // assert
      EXPECT_DOUBLE_EQ(xAxisDisplacement, 0.0);
    }

    TEST_F(TestDenavitHartenbergParameters, GetXAxisDisplacement_WhenXAxisDisplacementIsSet_ReturnsProvidedDisplacement) {
      // arrange
      const double expectedXAxisDisplacement = 4.234509;
      DenavitHartenbergParameters dhParameters;
      dhParameters.setXAxisDisplacement(expectedXAxisDisplacement);

      // act
      const auto xAxisDisplacement = dhParameters.getXAxisDisplacement();

      // assert
      EXPECT_DOUBLE_EQ(xAxisDisplacement, expectedXAxisDisplacement);
    }

    TEST_F(TestDenavitHartenbergParameters, SetXAxisDisplacement_WhenDisplacementIsNotFinite_ThrowsException) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act assert
      EXPECT_THROW(dhParameters.setXAxisDisplacement(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(dhParameters.setXAxisDisplacement(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestDenavitHartenbergParameters, GetXAxisRotation_WhenDefaultConstructed_ReturnsZero) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act
      const auto xAxisRotation = dhParameters.getXAxisRotation();

      // assert
      EXPECT_DOUBLE_EQ(xAxisRotation, 0.0);
    }

    TEST_F(TestDenavitHartenbergParameters, GetXAxisRotation_WhenXAxisRotationIsSet_ReturnsProvidedRotation) {
      // arrange
      const double expectedXAxisRotation = -12.3445652;
      DenavitHartenbergParameters dhParameters;
      dhParameters.setXAxisRotation(expectedXAxisRotation);

      // act
      const auto xAxisRotation = dhParameters.getXAxisRotation();

      // assert
      EXPECT_DOUBLE_EQ(xAxisRotation, expectedXAxisRotation);
    }

    TEST_F(TestDenavitHartenbergParameters, SetXAxisRotation_WhenAngleIsNotFinite_ThrowsException) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act assert
      EXPECT_THROW(dhParameters.setXAxisRotation(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(dhParameters.setXAxisRotation(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestDenavitHartenbergParameters, GetTransform_IfJointTypeIsRevolute_ReturnsValidTransform) {
      // arrange
      const double zAxisDisplacement = 100.0;
      const double zAxisRotation = 0.123 * M_PI;
      const double xAxisDisplacement = 200.0;
      const double xAxisRotation = 0.456 * M_PI;
      const double angle = 0.789 * M_PI;
      const Eigen::Matrix4d expectedMatrix = getDHMatrix(zAxisDisplacement, zAxisRotation, xAxisDisplacement, xAxisRotation, angle, 0.0);

      DenavitHartenbergParameters dhParameters(JointType::Revolute, zAxisDisplacement, zAxisRotation, xAxisDisplacement, xAxisRotation);

      // act
      const Eigen::Affine3d transform = dhParameters.getTransform(angle);

      // assert
      EXPECT_TRUE(transform.matrix().isApprox(expectedMatrix));
    };

    TEST_F(TestDenavitHartenbergParameters, GetTransform_IfJointTypeIsPrismatic_ReturnsValidTransform) {
      // arrange
      const double zAxisDisplacement = 100.0;
      const double zAxisRotation = 0.123 * M_PI;
      const double xAxisDisplacement = 200.0;
      const double xAxisRotation = 0.456 * M_PI;
      const double distance = 123.0;
      const Eigen::Matrix4d expectedMatrix = getDHMatrix(zAxisDisplacement, zAxisRotation, xAxisDisplacement, xAxisRotation, 0.0, distance);

      DenavitHartenbergParameters dhParameters(JointType::Prismatic, zAxisDisplacement, zAxisRotation, xAxisDisplacement, xAxisRotation);

      // act
      const Eigen::Affine3d transform = dhParameters.getTransform(distance);

      // assert
      EXPECT_TRUE(transform.matrix().isApprox(expectedMatrix));
    };

    TEST_F(TestDenavitHartenbergParameters, GetTransform_WhenValueIsNotFinite_ThrowsException) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act assert
      EXPECT_THROW(dhParameters.getTransform(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(dhParameters.getTransform(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestDenavitHartenbergParameters, GetTransformZ_IfJointTypeIsRevolute_ReturnsValidZPartOfTransform) {
      // arrange
      const double zAxisDisplacement = 100.0;
      const double zAxisRotation = 0.123 * M_PI;
      const double xAxisDisplacement = 200.0;
      const double xAxisRotation = 0.456 * M_PI;
      const double angle = 0.789 * M_PI;
      const Eigen::Matrix4d expectedMatrix = getZMatrix(zAxisDisplacement, zAxisRotation, angle, 0.0);

      DenavitHartenbergParameters dhParameters(JointType::Revolute, zAxisDisplacement, zAxisRotation, xAxisDisplacement, xAxisRotation);

      // act
      const Eigen::Affine3d transform = dhParameters.getTransformZ(angle);

      // assert
      EXPECT_TRUE(transform.matrix().isApprox(expectedMatrix));
    }

    TEST_F(TestDenavitHartenbergParameters, GetTransformZ_IfJointTypeIsPrismatic_ReturnsValidZPartOfTransform) {
      // arrange
      const double zAxisDisplacement = 100.0;
      const double zAxisRotation = 0.123 * M_PI;
      const double xAxisDisplacement = 200.0;
      const double xAxisRotation = 0.456 * M_PI;
      const double distance = 123.0;
      const Eigen::Matrix4d expectedMatrix = getZMatrix(zAxisDisplacement, zAxisRotation, 0.0, distance);

      DenavitHartenbergParameters dhParameters(JointType::Prismatic, zAxisDisplacement, zAxisRotation, xAxisDisplacement, xAxisRotation);

      // act
      const Eigen::Affine3d transform = dhParameters.getTransformZ(distance);

      // assert
      EXPECT_TRUE(transform.matrix().isApprox(expectedMatrix));
    }

    TEST_F(TestDenavitHartenbergParameters, GetTransformZ_WhenValueIsNotFinite_ThrowsException) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act assert
      EXPECT_THROW(dhParameters.getTransformZ(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(dhParameters.getTransformZ(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestDenavitHartenbergParameters, GetTransformX_ReturnsValidXPartOfTransform) {
      // arrange
      const double zAxisDisplacement = 100.0;
      const double zAxisRotation = 0.123 * M_PI;
      const double xAxisDisplacement = 200.0;
      const double xAxisRotation = 0.456 * M_PI;
      const Eigen::Matrix4d expectedMatrix = getXMatrix(xAxisDisplacement, xAxisRotation);

      DenavitHartenbergParameters dhParameters(JointType::Revolute, zAxisDisplacement, zAxisRotation, xAxisDisplacement, xAxisRotation);

      // act
      const Eigen::Affine3d transform = dhParameters.getTransformX();

      // assert
      EXPECT_TRUE(transform.matrix().isApprox(expectedMatrix));
    }

    TEST_F(TestDenavitHartenbergParameters, GetTransform_EqualsSumOfZAndXPartOfTransforms) {
      // arrange
      const double zAxisDisplacement = 100.0;
      const double zAxisRotation = 0.123 * M_PI;
      const double xAxisDisplacement = 200.0;
      const double xAxisRotation = 0.456 * M_PI;
      const double angle = 0.789 * M_PI;
      DenavitHartenbergParameters dhParameters(JointType::Revolute, zAxisDisplacement, zAxisRotation, xAxisDisplacement, xAxisRotation);

      // act
      const Eigen::Affine3d transform = dhParameters.getTransform(angle);
      const Eigen::Affine3d zTransform = dhParameters.getTransformZ(angle);
      const Eigen::Affine3d xTransform = dhParameters.getTransformX();

      // assert
      EXPECT_TRUE(transform.isApprox(zTransform * xTransform));
    }

    TEST_F(TestDenavitHartenbergParameters, GetJointTransform_WhenAxisValueIsNotFinite_ThrowsException) {
      // arrange
      DenavitHartenbergParameters dhParameters;

      // act assert
      EXPECT_THROW(dhParameters.getJointTransform(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(dhParameters.getJointTransform(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestDenavitHartenbergParameters, GetJointTransforms_WhenJointIsRevoluteAndAxisValueIsZero_ReturnsIdentityTransform) {
      // arrange
      DenavitHartenbergParameters dhParameters(JointType::Revolute);

      // act
      const auto transform = dhParameters.getJointTransform(0.0);

      // assert
      EXPECT_TRUE(transform.isApprox(Eigen::Affine3d::Identity()));
    }

    TEST_F(TestDenavitHartenbergParameters, GetJointTransforms_WhenJointIsPrismaticAndAxisValueIsZero_ReturnsIdentityTransform) {
      // arrange
      DenavitHartenbergParameters dhParameters(JointType::Prismatic);

      // act
      const auto transform = dhParameters.getJointTransform(0.0);

      // assert
      EXPECT_TRUE(transform.isApprox(Eigen::Affine3d::Identity()));
    }

    TEST_F(TestDenavitHartenbergParameters, GetJointTransforms_WhenJointIsRevolute_ReturnsValidTransform) {
      // arrange
      const auto angle = M_PI_2 / 3.0;
      const Eigen::Matrix4d expectedMatrix = getRotationMatrixAboutZAxis(angle);

      DenavitHartenbergParameters dhParameters(JointType::Revolute);

      // act
      const auto transform = dhParameters.getJointTransform(angle);

      // assert
      EXPECT_TRUE(transform.matrix().isApprox(expectedMatrix));
    }

    TEST_F(TestDenavitHartenbergParameters, GetJointTransforms_WhenJointIsPrismatic_ReturnsValidTransform) {
      // arrange
      const auto displacement = 123.0456789;
      const Eigen::Matrix4d expectedMatrix = getZTranslationMatrix(displacement);

      DenavitHartenbergParameters dhParameters(JointType::Prismatic);

      // act
      const auto transform = dhParameters.getJointTransform(displacement);

      // assert
      EXPECT_TRUE(transform.matrix().isApprox(expectedMatrix));
    }

    TEST_P(TestDenavitHartenbergParameters, EqualityOperator_WorksCorrectly) {
      // arrange
      const auto lhs = std::get<0>(GetParam());
      const auto rhs = std::get<1>(GetParam());
      const bool expectedResult = std::get<2>(GetParam());

      // act
      const auto result = lhs == rhs;

      // assert
      EXPECT_EQ(result, expectedResult);
    }

    TEST_P(TestDenavitHartenbergParameters, InequalityOperator_WorksCorrectly) {
      // arrange
      const auto lhs = std::get<0>(GetParam());
      const auto rhs = std::get<1>(GetParam());
      const bool expectedResult = !std::get<2>(GetParam());

      // act
      const auto result = lhs != rhs;

      // assert
      EXPECT_EQ(result, expectedResult);
    }

  }  // namespace core
}  // namespace kinverse
