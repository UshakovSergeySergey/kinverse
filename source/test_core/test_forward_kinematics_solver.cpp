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
#include <kinverse/math/math.h>

namespace kinverse {
  namespace core {

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

    TEST_P(TestForwardKinematicsSolver, Solve_WhenRobotIsKukaKR5Arc_ReturnsCorrectEndEffectorTransform) {
      // arrange
      const Eigen::VectorXd configuration = std::get<0>(GetParam());
      const Eigen::Vector3d expectedXYZ = std::get<1>(GetParam());
      const Eigen::Vector3d expectedABC = std::get<2>(GetParam());
      const Eigen::Affine3d expectedEndEffectorTransform = math::fromXYZABC(expectedXYZ, expectedABC);

      const auto robot = getKukaKR5ArcRobot();
      ForwardKinematicsSolver fkSolver(robot);

      // act
      const Eigen::Affine3d endEffectorTransform = fkSolver.solve(configuration);

      // assert
      EXPECT_TRUE(endEffectorTransform.isApprox(expectedEndEffectorTransform));
    }

  }  // namespace core
}  // namespace kinverse
