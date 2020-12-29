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
#include "test_forward_kinematics_solver.h"
#include <kinverse/core/forward_kinematics_solver.h>
#include <kinverse/core/robot.h>

namespace kinverse {
  namespace core {

    TEST_F(TestForwardKinematicsSolver, GetRobot_WhenDefaultConstructed_ReturnsNullptr) {
      // arrange
      ForwardKinematicsSolver fkSolver;

      // act
      const auto robot = fkSolver.getRobot();

      // assert
      EXPECT_EQ(robot, nullptr);
    }

    TEST_F(TestForwardKinematicsSolver, GetRobot_WhenRobotIsSet_ReturnsProvidedRobot) {
      // arrange
      const auto expectedRobot = createSimpleRobotWithTwoJoints();

      ForwardKinematicsSolver fkSolver;
      fkSolver.setRobot(expectedRobot);

      // act
      const auto robot = fkSolver.getRobot();

      // assert
      EXPECT_EQ(robot, expectedRobot);
    }

    TEST_F(TestForwardKinematicsSolver, Solve_WhenConfigurationNotFinite_ThrowsException) {
      // arrange
      Eigen::VectorXd configuration(3);
      configuration << 0.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity();

      ForwardKinematicsSolver fkSolver;

      // act assert
      EXPECT_THROW(fkSolver.solve(configuration), std::domain_error);
    }

    TEST_F(TestForwardKinematicsSolver, Solve_WhenConfigurationDoesntCorrespondToNumberOfJoints_ThrowsException) {
      // arrange
      const Eigen::VectorXd configuration(3);
      const std::vector<DenavitHartenbergParameters> dhTable{ DenavitHartenbergParameters{}, DenavitHartenbergParameters{} };
      const auto robot = std::make_shared<Robot>();
      robot->setDHTable(dhTable);

      ForwardKinematicsSolver fkSolver(robot);

      // act assert
      EXPECT_THROW(fkSolver.solve(configuration), std::domain_error);
    }

    TEST_F(TestForwardKinematicsSolver, Solve_WhenRobotIsNullptr_ThrowsException) {
      // arrange
      Eigen::VectorXd configuration(3);
      configuration << 1.0, 2.0, 3.0;
      ForwardKinematicsSolver fkSolver;

      // act assert
      EXPECT_THROW(fkSolver.solve(configuration), std::domain_error);
    }

    TEST_P(TestForwardKinematicsSolver, Solve_WhenConfigurationIsValid_ReturnsCorrectEndEffectorTransform) {
      // arrange
      Eigen::VectorXd configuration;
      Eigen::Affine3d solution;
      getConfigurationAndSolution(configuration, solution);

      const auto robot = createSimpleRobotWithTwoJoints();
      ForwardKinematicsSolver fkSolver(robot);

      // act
      const Eigen::Affine3d endEffectorTransform = fkSolver.solve(configuration);

      // assert
      EXPECT_TRUE(endEffectorTransform.isApprox(solution));
    }

    TEST_P(TestForwardKinematicsSolver, Solve_WhenRobotBaseTransformIsSet_ItIsAppliedToSolution) {
      // arrange
      const Eigen::Affine3d robotBaseTransform = Eigen::Translation3d(743.32984, 3473.2398, -92.0238) * Eigen::AngleAxisd(23.2345, Eigen::Vector3d::UnitZ());

      Eigen::VectorXd configuration;
      Eigen::Affine3d solution;
      getConfigurationAndSolution(configuration, solution);

      const auto robot = createSimpleRobotWithTwoJoints();
      robot->setBaseTransform(robotBaseTransform);
      ForwardKinematicsSolver fkSolver(robot);

      // act
      const Eigen::Affine3d endEffectorTransform = fkSolver.solve(configuration);

      // assert
      EXPECT_TRUE(endEffectorTransform.isApprox(robotBaseTransform * solution));
    }

    TEST_P(TestForwardKinematicsSolver, Solve_WhenRobotTransformIsSet_ItIsAppliedToSolution) {
      // arrange
      const Eigen::Affine3d robotTransform = Eigen::Translation3d(123.456789, -456.7890123, 98.7654321) * Eigen::AngleAxisd(1.2345, Eigen::Vector3d::UnitY());

      Eigen::VectorXd configuration;
      Eigen::Affine3d solution;
      getConfigurationAndSolution(configuration, solution);

      const auto robot = createSimpleRobotWithTwoJoints();
      robot->setTransform(robotTransform);
      ForwardKinematicsSolver fkSolver(robot);

      // act
      const Eigen::Affine3d endEffectorTransform = fkSolver.solve(configuration);

      // assert
      EXPECT_TRUE(endEffectorTransform.isApprox(robotTransform * solution));
    }

    TEST_P(TestForwardKinematicsSolver, Solve_WhenTCPTransformIsSet_ItIsAppliedToSolution) {
      // arrange
      const Eigen::Affine3d tcpTransform = Eigen::Translation3d(Eigen::Vector3d::Random()) * Eigen::Quaterniond::UnitRandom();

      Eigen::VectorXd configuration;
      Eigen::Affine3d solution;
      getConfigurationAndSolution(configuration, solution);

      const auto robot = createSimpleRobotWithTwoJoints();
      robot->setTCPTransform(tcpTransform);
      ForwardKinematicsSolver fkSolver(robot);

      // act
      const Eigen::Affine3d endEffectorTransform = fkSolver.solve(configuration);

      // assert
      EXPECT_TRUE(endEffectorTransform.isApprox(solution * tcpTransform));
    }

    TEST_P(TestForwardKinematicsSolver, Solve_RobotBaseTransform_IsAppliedToSolutionBeforeRobotTransform) {
      // arrange
      const Eigen::Affine3d robotTransform = Eigen::Translation3d(123.456789, -456.7890123, 98.7654321) * Eigen::AngleAxisd(1.2345, Eigen::Vector3d::UnitY());
      const Eigen::Affine3d robotBaseTransform = Eigen::Translation3d(743.32984, 3473.2398, -92.0238) * Eigen::AngleAxisd(23.2345, Eigen::Vector3d::UnitZ());

      Eigen::VectorXd configuration;
      Eigen::Affine3d solution;
      getConfigurationAndSolution(configuration, solution);

      const auto robot = createSimpleRobotWithTwoJoints();
      robot->setTransform(robotTransform);
      robot->setBaseTransform(robotBaseTransform);
      ForwardKinematicsSolver fkSolver(robot);

      // act
      const Eigen::Affine3d endEffectorTransform = fkSolver.solve(configuration);

      // assert
      EXPECT_TRUE(endEffectorTransform.isApprox(robotTransform * robotBaseTransform * solution));
    }

    TEST_F(TestForwardKinematicsSolver, GetLinkCoordinateFrames_ByDefault_IsEmpty) {
      // arrange
      const auto robot = createSimpleRobotWithTwoJoints();
      ForwardKinematicsSolver fkSolver(robot);

      // act
      const auto linkCoordinateFrames = fkSolver.getLinkCoordinateFrames();

      // assert
      EXPECT_TRUE(linkCoordinateFrames.empty());
    }

    TEST_F(TestForwardKinematicsSolver, GetLinkCoordinateFrames_AfterSolveIsCalled_ReturnsCorrectNumberOfTransforms) {
      // arrange
      const auto robot = createSimpleRobotWithTwoJoints();
      ForwardKinematicsSolver fkSolver(robot);

      Eigen::VectorXd configuration(2);
      configuration << 0.0, 0.0;
      fkSolver.solve(configuration);

      // act
      const auto linkCoordinateFrames = fkSolver.getLinkCoordinateFrames();

      // assert
      EXPECT_EQ(linkCoordinateFrames.size(), robot->getNumberOfJoints() + 3);
    }

    TEST_F(TestForwardKinematicsSolver, GetLinkCoordinateFrames_AfterSolveIsCalled_ReturnsCorrectTransforms) {
      std::vector<Eigen::Affine3d> expectedLinkCoordinateFrames{};
      // fill in the expected transforms (I have calculated them manually)
      {
        const Eigen::Vector3d robotTranslation{ 1000.0, 2000.0, 3000.0 };

        // link to BASE
        const Eigen::Affine3d baseTransform = math::fromXYZABC(robotTranslation, Eigen::Vector3d::Zero());
        expectedLinkCoordinateFrames.push_back(baseTransform);

        // link to A1
        const Eigen::Affine3d a1Transform = math::fromXYZABC(robotTranslation, Eigen::Vector3d(0.0, 0.0, M_PI));
        expectedLinkCoordinateFrames.push_back(a1Transform);

        // link to A2
        const Eigen::Vector3d a2Translation{ 100.0 * cos(M_PI_2 / 3.0), -100.0 * sin(M_PI_2 / 3.0), 0.0 };
        const Eigen::Affine3d a2Transform = math::fromXYZABC(robotTranslation + a2Translation, Eigen::Vector3d(-M_PI_2 / 3.0, 0.0, M_PI_2));
        expectedLinkCoordinateFrames.push_back(a2Transform);

        // link to FLANGE
        const Eigen::Vector3d flangeTranslation{ 0.0, 0.0, -300.0 };
        const Eigen::Affine3d flangeTransform =
            math::fromXYZABC(robotTranslation + a2Translation + flangeTranslation, Eigen::Vector3d(-M_PI_2 / 3.0, M_PI_2, 0.0));
        expectedLinkCoordinateFrames.push_back(flangeTransform);

        // link to TCP
        const Eigen::Affine3d tcpTransform =
            flangeTransform * Eigen::Translation3d(1234.0348, 2348.3498, 4584.382) * Eigen::AngleAxisd(23432.0232, Eigen::Vector3d::UnitY());
        expectedLinkCoordinateFrames.push_back(tcpTransform);
      }

      // arrange
      const Eigen::Affine3d transform = Eigen::Translation3d(1000.0, 2000.0, 3000.0) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
      const Eigen::Affine3d baseTransform = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
      const Eigen::Affine3d tcpTransform = Eigen::Translation3d(1234.0348, 2348.3498, 4584.382) * Eigen::AngleAxisd(23432.0232, Eigen::Vector3d::UnitY());

      const auto robot = createSimpleRobotWithTwoJoints();
      robot->setTransform(transform);
      robot->setBaseTransform(baseTransform);
      robot->setTCPTransform(tcpTransform);

      Eigen::VectorXd configuration(2);
      configuration << M_PI_2 / 3.0, -200.0;

      ForwardKinematicsSolver fkSolver(robot);
      fkSolver.solve(configuration);

      // act
      const auto linkCoordinateFrames = fkSolver.getLinkCoordinateFrames();

      // assert
      ASSERT_EQ(linkCoordinateFrames.size(), expectedLinkCoordinateFrames.size());
      for (auto linkCounter = 0u; linkCounter < linkCoordinateFrames.size(); ++linkCounter) {
        const Eigen::Affine3d& link = linkCoordinateFrames[linkCounter];
        const Eigen::Affine3d& expectedLink = expectedLinkCoordinateFrames[linkCounter];
        EXPECT_TRUE(link.isApprox(expectedLink));
      }
    }

  }  // namespace core
}  // namespace kinverse
