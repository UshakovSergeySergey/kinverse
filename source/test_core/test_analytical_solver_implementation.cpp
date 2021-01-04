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
#include "test_analytical_solver_implementation.h"
#include <implementation/analytical_solver_implementation.h>
#include <kinverse/math/math.h>

namespace kinverse {
  namespace core {

    TEST_F(TestAnalyticalSolverImplementation, Constructor_WhenRobotIsNullptr_ThrowsException) {
      // arrange act assert
      EXPECT_THROW(AnalyticalSolverImplementation(nullptr), std::domain_error);
    }

    TEST_F(TestAnalyticalSolverImplementation, Constructor_WhenNumberOfJointsNotEqualSix_ThrowsException) {
      // arrange
      const std::vector<DenavitHartenbergParameters> dhTable{ DenavitHartenbergParameters{}, DenavitHartenbergParameters{} };
      const auto robot = std::make_shared<Robot>();
      robot->setDHTable(dhTable);

      // act assert
      EXPECT_THROW(std::make_shared<AnalyticalSolverImplementation>(robot), std::domain_error);
    }

    TEST_P(TestAnalyticalSolverImplementationGetGamma, GetGamma_WhenCalled_ReturnsCorrectDistance) {
      // arrange
      const DenavitHartenbergParameters dhParamsA3 = std::get<0>(GetParam());
      const DenavitHartenbergParameters dhParamsA4 = std::get<1>(GetParam());
      const double expectedGamma = std::get<2>(GetParam());

      std::vector<DenavitHartenbergParameters> dhTable(6, DenavitHartenbergParameters{});
      dhTable[2] = dhParamsA3;
      dhTable[3] = dhParamsA4;

      const auto robot = std::make_shared<Robot>();
      robot->setDHTable(dhTable);

      AnalyticalSolverImplementation solver(robot);

      // act
      const auto gamma = solver.getGamma();

      // assert
      EXPECT_DOUBLE_EQ(gamma, expectedGamma);
    }

    TEST_P(TestAnalyticalSolverImplementationGetDistanceFromA3ToA2, GetDistanceFromA3ToA2_WhenCalled_ReturnsCorrectDistance) {
      // arrange
      const DenavitHartenbergParameters dhParamsA2 = std::get<0>(GetParam());
      const double expectedDistance = std::get<1>(GetParam());

      std::vector<DenavitHartenbergParameters> dhTable(6, DenavitHartenbergParameters{});
      dhTable[1] = dhParamsA2;

      const auto robot = std::make_shared<Robot>();
      robot->setDHTable(dhTable);

      AnalyticalSolverImplementation solver(robot);

      // act
      const auto distance = solver.getDistanceFromA3ToA2();

      // assert
      EXPECT_DOUBLE_EQ(distance, expectedDistance);
    }

    TEST_P(TestAnalyticalSolverImplementationGetDistanceFromA3ToWrist, GetDistanceFromA3ToWrist_WhenCalled_ReturnsCorrectDistance) {
      // arrange
      const DenavitHartenbergParameters dhParamsA3 = std::get<0>(GetParam());
      const DenavitHartenbergParameters dhParamsA4 = std::get<1>(GetParam());
      const double expectedDistance = std::get<2>(GetParam());

      std::vector<DenavitHartenbergParameters> dhTable(6, DenavitHartenbergParameters{});
      dhTable[2] = dhParamsA3;
      dhTable[3] = dhParamsA4;

      const auto robot = std::make_shared<Robot>();
      robot->setDHTable(dhTable);

      AnalyticalSolverImplementation solver(robot);

      // act
      const auto distance = solver.getDistanceFromA3ToWrist();

      // assert
      EXPECT_DOUBLE_EQ(distance, expectedDistance);
    }

    TEST_F(TestAnalyticalSolverImplementation, GetA1ZAxis_WhenCalled_ReturnsUnitZ) {
      // arrange
      const std::vector<DenavitHartenbergParameters> dhTable(6, DenavitHartenbergParameters{});
      const auto robot = std::make_shared<Robot>();
      robot->setDHTable(dhTable);

      AnalyticalSolverImplementation solver(robot);

      // act
      const Eigen::Vector3d axis = solver.getA1ZAxis();

      // assert
      EXPECT_TRUE(axis.isApprox(Eigen::Vector3d::UnitZ()));
    }

    TEST_F(TestAnalyticalSolverImplementation, ConvertWorldToA1Local_WhenCalled_ReturnsCorrectResult) {
      // arrange
      const Eigen::Affine3d robotTransform = math::fromXYZABC(Eigen::Vector3d{ 100.0, 200.0, 300.0 }, Eigen::Vector3d{ M_PI_2, -M_PI_2, M_PI_2 });
      const Eigen::Affine3d robotBaseTransform = math::fromXYZABC(Eigen::Vector3d::Zero(), Eigen::Vector3d{ 0.0, 0.0, M_PI });
      const Eigen::Affine3d worldTransform = math::fromXYZABC(Eigen::Vector3d{ -300.0, -200.0, -100.0 }, Eigen::Vector3d{ M_PI, -M_PI_2, 0.0 });
      const Eigen::Affine3d expectedLocalTransform = math::fromXYZABC(Eigen::Vector3d{ -400.0, -400.0, 400.0 }, Eigen::Vector3d{ 0.0, 0.0, M_PI });

      const std::vector<DenavitHartenbergParameters> dhTable(6, DenavitHartenbergParameters{});

      const auto robot = std::make_shared<Robot>();
      robot->setDHTable(dhTable);
      robot->setTransform(robotTransform);
      robot->setBaseTransform(robotBaseTransform);

      AnalyticalSolverImplementation solver(robot);

      // act
      const Eigen::Affine3d localTransform = solver.convertWorldToA1Local(worldTransform);

      // assert
      EXPECT_TRUE(localTransform.isApprox(expectedLocalTransform));
    }

    TEST_F(TestAnalyticalSolverImplementation, ConvertTCPToFlange_WhenCalled_ReturnsCorrectResult) {
      // arrange
      const Eigen::Affine3d robotTCPTransform = math::fromXYZABC(Eigen::Vector3d{ 100.0, 200.0, 300.0 }, Eigen::Vector3d{ M_PI_2, -M_PI_2, 0.0 });
      const Eigen::Affine3d tcpTransform = math::fromXYZABC(Eigen::Vector3d{ 100.0, -100.0, 100.0 }, Eigen::Vector3d{ -M_PI_2, 0.0, M_PI_2 });
      const Eigen::Affine3d expectedFlangeTransform = math::fromXYZABC(Eigen::Vector3d{ -100.0, 200.0, 200.0 }, Eigen::Vector3d{ -M_PI_2, M_PI_2, 0.0 });

      const std::vector<DenavitHartenbergParameters> dhTable(6, DenavitHartenbergParameters{});

      const auto robot = std::make_shared<Robot>();
      robot->setDHTable(dhTable);
      robot->setTCPTransform(robotTCPTransform);

      AnalyticalSolverImplementation solver(robot);

      // act
      const Eigen::Affine3d flangeTransform = solver.convertTCPToFlange(tcpTransform);

      // assert
      EXPECT_TRUE(flangeTransform.isApprox(expectedFlangeTransform));
    }

  }  // namespace core
}  // namespace kinverse
