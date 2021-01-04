﻿/**
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
#include "test_analytical_solver.h"
#include <kinverse/core/analytical_solver.h>
#include <kinverse/factory/robot_factory.h>
#include <kinverse/math/math.h>

namespace kinverse {
  namespace core {

    TEST_F(TestAnalyticalSolver, Solve_WhenEndEffectorTransformIsNotFinite_ThrowsException) {
      // arrange
      auto robot = factory::RobotFactory::create(RobotType::KukaKR5Arc);
      auto ikSolver = std::make_shared<AnalyticalSolver>(robot);

      const Eigen::Vector3d xyz{ std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0 };
      const Eigen::Vector3d abc{ std::numeric_limits<double>::infinity(), 0.0, 0.0 };
      const Eigen::Affine3d endEffectorTransform = math::fromXYZABC(xyz, abc);

      // act assert
      EXPECT_THROW(ikSolver->solve(endEffectorTransform), std::domain_error);
    }

    TEST_F(TestAnalyticalSolver, Solve_WhenShoulderSingularityDetected_ThrowsException) {
      // arrange
      auto robot = factory::RobotFactory::create(RobotType::KukaKR5Arc);
      auto ikSolver = std::make_shared<AnalyticalSolver>(robot);

      const Eigen::Vector3d xyz{ 0.0, 0.0, 1000.0 };
      const Eigen::Vector3d abc{ 0.0, 0.0, 0.0 };
      const Eigen::Affine3d endEffectorTransform = math::fromXYZABC(xyz, abc);

      // act assert
      EXPECT_THROW(ikSolver->solve(endEffectorTransform), std::exception);
    }

    // ShoulderSingularity
    // ElbowSingularity
    // WristSingularity
    // PointIsUnreachable
  }  // namespace core
}  // namespace kinverse
