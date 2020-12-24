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
#include "test_joint_constraints.h"

namespace kinverse {
  namespace core {

    TEST_F(TestJointConstraints, GetMaximumSpeed_WhenDefaultConstructed_HasNoSpeedConstraint) {
      // arrange
      JointConstraints constraints;

      // act
      const auto maximumSpeed = constraints.getMaximumSpeed();

      // assert
      EXPECT_DOUBLE_EQ(maximumSpeed, std::numeric_limits<double>::max());
    }

    TEST_F(TestJointConstraints, GetMaximumSpeed_WhenMaximumSpeedIsSet_ReturnsProvidedSpeed) {
      // arrange
      const double expectedSpeed = 0.123456789;
      JointConstraints constraints;
      constraints.setMaximumSpeed(expectedSpeed);

      // act
      const auto maximumSpeed = constraints.getMaximumSpeed();

      // assert
      EXPECT_DOUBLE_EQ(maximumSpeed, expectedSpeed);
    }

    TEST_F(TestJointConstraints, SetMaximumSpeed_WhenSpeedIsNonFinite_ThrowsException) {
      // arrange
      JointConstraints constraints;

      // act assert
      EXPECT_THROW(constraints.setMaximumSpeed(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(constraints.setMaximumSpeed(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestJointConstraints, SetMaximumSpeed_WhenSpeedIsNegative_ThrowsException) {
      // arrange
      JointConstraints constraints;

      // act assert
      EXPECT_THROW(constraints.setMaximumSpeed(-1.0), std::domain_error);
    }

    TEST_F(TestJointConstraints, GetMinimumAxisValue_WhenDefaultConstructed_HasNoLowerBoundConstraint) {
      // arrange
      JointConstraints constraints;

      // act
      const auto minimumAxisValue = constraints.getMinimumAxisValue();

      // assert
      EXPECT_DOUBLE_EQ(minimumAxisValue, -std::numeric_limits<double>::max());
    }

    TEST_F(TestJointConstraints, GetMinimumAxisValue_WhenMinimumAxisValueIsSet_ReturnsProvidedLowerBound) {
      // arrange
      const double expectedMinimumAxisValue = -1.23456789;
      JointConstraints constraints;
      constraints.setMinimumAxisValue(expectedMinimumAxisValue);

      // act
      const auto minimumAxisValue = constraints.getMinimumAxisValue();

      // assert
      ASSERT_DOUBLE_EQ(minimumAxisValue, expectedMinimumAxisValue);
    }

    TEST_F(TestJointConstraints, SetMinimumAxisValue_WhenValueIsNotFinite_ThrowsException) {
      // arrange
      JointConstraints constraints;

      // act assert
      EXPECT_THROW(constraints.setMinimumAxisValue(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(constraints.setMinimumAxisValue(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestJointConstraints, GetMaximumAxisValue_WhenDefaultConstructed_HasNoUpperBoundConstraint) {
      // arrange
      JointConstraints constraints;

      // act
      const auto maximumAxisValue = constraints.getMaximumAxisValue();

      // assert
      EXPECT_DOUBLE_EQ(maximumAxisValue, std::numeric_limits<double>::max());
    }

    TEST_F(TestJointConstraints, GetMaximumAxisValue_WhenMaximumAxisValueIsSet_ReturnsProvidedUpperBound) {
      // arrange
      const double expectedMaximumAxisValue = 10.40932498;
      JointConstraints constraints;
      constraints.setMaximumAxisValue(expectedMaximumAxisValue);

      // act
      const auto maximumAxisValue = constraints.getMaximumAxisValue();

      // assert
      EXPECT_DOUBLE_EQ(maximumAxisValue, expectedMaximumAxisValue);
    }

    TEST_F(TestJointConstraints, SetMaximumAxisValue_WhenValueIsNotFinite_ThrowsException) {
      // arrange
      JointConstraints constraints;

      // act assert
      EXPECT_THROW(constraints.setMaximumAxisValue(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(constraints.setMaximumAxisValue(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestJointConstraints, ViolatesSpeedConstraint_WhenSpeedIsLowerOrEqualThanMaximumSpeed_ReturnsFalse) {
      // arrange
      JointConstraints constraints;
      constraints.setMaximumSpeed(0.123456789);

      for (const auto speed : { 0.02, 0.123456789 }) {
        // act
        const bool violatesConstraints = constraints.violatesSpeedConstraint(speed);

        // assert
        EXPECT_FALSE(violatesConstraints);
      }
    }

    TEST_F(TestJointConstraints, ViolatesSpeedConstraint_WhenSpeedIsGreaterThanMaximumSpeed_ReturnsTrue) {
      // arrange
      JointConstraints constraints;
      constraints.setMaximumSpeed(0.123456789);

      for (const auto speed : { 1.02, 0.123456790 }) {
        // act
        const bool violatesConstraints = constraints.violatesSpeedConstraint(speed);

        // assert
        EXPECT_TRUE(violatesConstraints);
      }
    }

    TEST_F(TestJointConstraints, ViolatesSpeedConstraint_WhenSpeedIsNonFinite_ThrowsException) {
      // arrange
      JointConstraints constraints;
      constraints.setMaximumSpeed(10.0);

      // act assert
      EXPECT_THROW(constraints.violatesSpeedConstraint(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(constraints.violatesSpeedConstraint(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestJointConstraints, ViolatesSpeedConstraint_WhenSpeedIsNegative_ThrowsException) {
      // arrange
      JointConstraints constraints;
      constraints.setMaximumSpeed(10.0);

      // act assert
      EXPECT_THROW(constraints.violatesSpeedConstraint(-1.0), std::domain_error);
    }

    TEST_F(TestJointConstraints, ViolatesRangeConstraint_WhenValueIsLowerThanLowerBound_ReturnsTrue) {
      // arrange
      JointConstraints constraints;
      constraints.setMinimumAxisValue(25.0);
      constraints.setMaximumAxisValue(30.0);

      // act
      const auto violatesConstraints = constraints.violatesRangeConstraint(24.999);

      // assert
      EXPECT_TRUE(violatesConstraints);
    }

    TEST_F(TestJointConstraints, ViolatesRangeConstraint_WhenValueIsGreaterThanUpperBound_ReturnsTrue) {
      // arrange
      JointConstraints constraints;
      constraints.setMinimumAxisValue(50.0);
      constraints.setMaximumAxisValue(75.0);

      // act
      const auto violatesConstraints = constraints.violatesRangeConstraint(100.0);

      // assert
      EXPECT_TRUE(violatesConstraints);
    }

    TEST_F(TestJointConstraints, ViolatesRangeConstraint_WhenValueIsInRange_ReturnsFalse) {
      // arrange
      JointConstraints constraints;
      constraints.setMinimumAxisValue(-100.0);
      constraints.setMaximumAxisValue(100.0);

      // act
      const bool violatesConstraints = constraints.violatesRangeConstraint(0.0);

      // assert
      EXPECT_FALSE(violatesConstraints);
    }

    TEST_F(TestJointConstraints, ViolatesRangeConstraint_WhenValueIsNotFinite_ThrowsException) {
      // arrange
      JointConstraints constraints;
      constraints.setMinimumAxisValue(-100.0);
      constraints.setMaximumAxisValue(100.0);

      // act assert
      EXPECT_THROW(constraints.violatesRangeConstraint(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(constraints.violatesRangeConstraint(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestJointConstraints, ViolatesRangeConstraint_WhenLowerBoundIsGreaterThanUpperBound_ReturnsTrue) {
      // arrange
      JointConstraints constraints;
      constraints.setMinimumAxisValue(100.0);
      constraints.setMaximumAxisValue(0.0);

      // act
      const bool violatesConstraints = constraints.violatesRangeConstraint(50.0);

      // assert
      EXPECT_TRUE(violatesConstraints);
    }

    TEST_F(TestJointConstraints, ClampAxisValue_WhenValueIsLowerThanLowerBound_ReturnsLowerBound) {
      // arrange
      const double lowerBound = 20.0;
      const double upperBound = 40.0;
      JointConstraints constraints;
      constraints.setMinimumAxisValue(lowerBound);
      constraints.setMaximumAxisValue(upperBound);

      // act
      const auto clampedValue = constraints.clampAxisValue(10.0);

      // assert
      EXPECT_DOUBLE_EQ(clampedValue, lowerBound);
    }

    TEST_F(TestJointConstraints, ClampAxisValue_WhenValueIsGreaterThanUpperBound_ReturnsUpperBound) {
      // arrange
      const double lowerBound = 20.0;
      const double upperBound = 40.0;
      JointConstraints constraints;
      constraints.setMinimumAxisValue(lowerBound);
      constraints.setMaximumAxisValue(upperBound);

      // act
      const auto clampedValue = constraints.clampAxisValue(50.0);

      // assert
      EXPECT_DOUBLE_EQ(clampedValue, upperBound);
    }

    TEST_F(TestJointConstraints, ClampAxisValue_WhenValueIsInRange_ReturnsUnchangedValue) {
      // arrange
      const double lowerBound = 20.0;
      const double upperBound = 40.0;
      const double expectedValue = 23.12243;
      JointConstraints constraints;
      constraints.setMinimumAxisValue(lowerBound);
      constraints.setMaximumAxisValue(upperBound);

      // act
      const auto clampedValue = constraints.clampAxisValue(expectedValue);

      // assert
      EXPECT_DOUBLE_EQ(clampedValue, expectedValue);
    }

    TEST_F(TestJointConstraints, ClampAxisValue_WhenValueIsNotFinite_ThrowsException) {
      // arrange
      const double lowerBound = 20.0;
      const double upperBound = 40.0;
      JointConstraints constraints;
      constraints.setMinimumAxisValue(lowerBound);
      constraints.setMaximumAxisValue(upperBound);

      // act assert
      EXPECT_THROW(constraints.clampAxisValue(std::numeric_limits<double>::quiet_NaN()), std::domain_error);
      EXPECT_THROW(constraints.clampAxisValue(std::numeric_limits<double>::infinity()), std::domain_error);
    }

    TEST_F(TestJointConstraints, ClampAxisValue_WhenLowerBoundIsGreaterThanUpperBound_ThrowsException) {
      // arrange
      JointConstraints constraints;
      constraints.setMinimumAxisValue(100.0);
      constraints.setMaximumAxisValue(0.0);

      // act assert
      EXPECT_THROW(constraints.clampAxisValue(50.0), std::domain_error);
    }

    TEST_P(TestJointConstraints, EqualityOperator_WorksCorrectly) {
      // arrange
      const auto lhs = std::get<0>(GetParam());
      const auto rhs = std::get<1>(GetParam());
      const bool expectedResult = std::get<2>(GetParam());

      // act
      const auto result = lhs == rhs;

      // assert
      EXPECT_EQ(result, expectedResult);
    }

    TEST_P(TestJointConstraints, InequalityOperator_WorksCorrectly) {
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
