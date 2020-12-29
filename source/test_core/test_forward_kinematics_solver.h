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

#include <kinverse/core/robot.h>
#include <kinverse/math/math.h>

namespace kinverse {
  namespace core {

    using RobotConfiguration = std::vector<double>;
    using EndEffectorPosition = Eigen::Vector3d;
    using EndEffectorOrientation = Eigen::Vector3d;
    using FKTask = std::tuple<RobotConfiguration, EndEffectorPosition, EndEffectorOrientation>;

    class TestForwardKinematicsSolver : public testing::Test, public testing::WithParamInterface<FKTask> {
     protected:
      void getConfigurationAndSolution(Eigen::VectorXd& configuration, Eigen::Affine3d& solution) const {
        const Eigen::Vector3d expectedXYZ = std::get<1>(GetParam());
        const Eigen::Vector3d expectedABC = std::get<2>(GetParam());

        configuration = convertRobotConfiguration(std::get<0>(GetParam()));
        solution = math::fromXYZABC(expectedXYZ, expectedABC);
      }
      Robot::Ptr createSimpleRobotWithTwoJoints() const {
        const std::vector<DenavitHartenbergParameters> dhTable{
          DenavitHartenbergParameters{ JointType::Revolute, 0.0, 0.0, 100.0, -M_PI_2 },        //
          DenavitHartenbergParameters{ JointType::Prismatic, 200.0, -M_PI_2, 300.0, -M_PI_2 }  //
        };

        const auto robot = std::make_shared<Robot>();
        robot->setDHTable(dhTable);
        return robot;
      }
      Eigen::VectorXd convertRobotConfiguration(const std::vector<double>& axisValues) const {
        Eigen::VectorXd configuration(axisValues.size());
        for (auto jointCounter = 0u; jointCounter < axisValues.size(); ++jointCounter) {
          configuration(jointCounter) = axisValues[jointCounter];
        }
        return configuration;
      }
    };

    INSTANTIATE_TEST_SUITE_P(Solve_SolvesForwardKinematicsCorrectly,
                             TestForwardKinematicsSolver,
                             testing::Values(                                                                                                  //
                                 FKTask{ { 0.0, 0.0 }, { 100.0, 200.0, 300.0 }, { M_PI, -M_PI_2, 0.0 } },                                      //
                                 FKTask{ { 0.0, 500.0 }, { 100.0, 700.0, 300.0 }, { M_PI, -M_PI_2, 0.0 } },                                    //
                                 FKTask{ { 0.0, -500.0 }, { 100.0, -300.0, 300.0 }, { M_PI, -M_PI_2, 0.0 } },                                  //
                                 FKTask{ { M_PI_2, 0.0 }, { -200.0, 100.0, 300.0 }, { M_PI_2, -M_PI_2, M_PI } },                               //
                                 FKTask{ { -M_PI_2, 0.0 }, { 200.0, -100.0, 300.0 }, { M_PI_2, -M_PI_2, 0.0 } },                               //
                                 FKTask{ { M_PI_2 + M_PI_4, -300.0 }, { 0.0, 100.0 * std::sqrt(2.0), 300.0 }, { M_PI_4, -M_PI_2, -M_PI_2 } },  //
                                 FKTask{ { -M_PI / 3.0, 100.0 },
                                         { 50.0 + 300.0 * cos(M_PI_2 / 3.0), 150.0 - 100.0 * cos(M_PI_2 / 3.0), 300.0 },
                                         { M_PI_2 / 3.0, -M_PI_2, M_PI_2 } }  //
                                 ));

  }  // namespace core
}  // namespace kinverse
