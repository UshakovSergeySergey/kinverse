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

    using DHP = DenavitHartenbergParameters;

    class TestAnalyticalSolverImplementation : public testing::Test {};

    using DataForGetGamma = std::tuple<DHP, DHP, double>;
    class TestAnalyticalSolverImplementationGetGamma : public testing::Test, public testing::WithParamInterface<DataForGetGamma> {};
    INSTANTIATE_TEST_SUITE_P(Gamma_ComputedCorrectly,
                             TestAnalyticalSolverImplementationGetGamma,
                             testing::Values(DataForGetGamma{ DHP{ core::JointType::Revolute, 0.0, -M_PI_2, 100.0, M_PI_2 },
                                                              DHP{ core::JointType::Revolute, 100.0 * std::sqrt(3.0), 0.0, 0.0, -M_PI_2 },
                                                              M_PI_2 / 3.0 },
                                             DataForGetGamma{ DHP{ core::JointType::Revolute, 0.0, -M_PI_2, 100.0, M_PI_2 },
                                                              DHP{ core::JointType::Revolute, -100.0 * std::sqrt(3.0), 0.0, 0.0, -M_PI_2 },
                                                              M_PI_2 / 3.0 },
                                             DataForGetGamma{ DHP{ core::JointType::Revolute, 0.0, -M_PI_2, -100.0, M_PI_2 },
                                                              DHP{ core::JointType::Revolute, 100.0 * std::sqrt(3.0), 0.0, 0.0, -M_PI_2 },
                                                              M_PI_2 / 3.0 },
                                             DataForGetGamma{ DHP{ core::JointType::Revolute, 0.0, -M_PI_2, -100.0, M_PI_2 },
                                                              DHP{ core::JointType::Revolute, -100.0 * std::sqrt(3.0), 0.0, 0.0, -M_PI_2 },
                                                              M_PI_2 / 3.0 },
                                             DataForGetGamma{ DHP{ core::JointType::Revolute, 0.0, -M_PI_2, 0.0, M_PI_2 },
                                                              DHP{ core::JointType::Revolute, 400.0, 0.0, 0.0, -M_PI_2 },
                                                              0.0 },
                                             DataForGetGamma{ DHP{ core::JointType::Revolute, 0.0, -M_PI_2, 300.0, M_PI_2 },
                                                              DHP{ core::JointType::Revolute, 0.0, 0.0, 0.0, -M_PI_2 },
                                                              M_PI_2 }));

    using DataForGetDistanceFromA3ToA2 = std::tuple<DHP, double>;
    class TestAnalyticalSolverImplementationGetDistanceFromA3ToA2 : public testing::Test, public testing::WithParamInterface<DataForGetDistanceFromA3ToA2> {};
    INSTANTIATE_TEST_SUITE_P(DistanceFromA3ToA2_ComputedCorrectly,
                             TestAnalyticalSolverImplementationGetDistanceFromA3ToA2,
                             testing::Values(                                                                                //
                                 DataForGetDistanceFromA3ToA2{ DHP{ JointType::Revolute, 0.0, 0.0, 0.0, 0.0 }, 0.0 },        //
                                 DataForGetDistanceFromA3ToA2{ DHP{ JointType::Revolute, 0.0, 0.0, 200.0, 0.0 }, 200.0 },    //
                                 DataForGetDistanceFromA3ToA2{ DHP{ JointType::Revolute, 0.0, 0.0, 350.0, 340.0 }, 350.0 },  //
                                 DataForGetDistanceFromA3ToA2{ DHP{ JointType::Revolute, 0.0, 430.0, 4.0, 340.0 }, 4.0 }     //
                                 ));

    using DataForGetDistanceFromA3ToWrist = std::tuple<DHP, DHP, double>;
    class TestAnalyticalSolverImplementationGetDistanceFromA3ToWrist :
        public testing::Test,
        public testing::WithParamInterface<DataForGetDistanceFromA3ToWrist> {};
    INSTANTIATE_TEST_SUITE_P(
        DistanceFromA3ToWrist_ComputedCorrectly,
        TestAnalyticalSolverImplementationGetDistanceFromA3ToWrist,
        testing::Values(  //
            DataForGetDistanceFromA3ToWrist{
                DHP{ core::JointType::Revolute, 0.0, -M_PI_2, 300.0, M_PI_2 }, DHP{ core::JointType::Revolute, 400.0, 0.0, 0.0, -M_PI_2 }, 500.0 },  //
            DataForGetDistanceFromA3ToWrist{
                DHP{ core::JointType::Revolute, 0.0, -M_PI_2, 300.0, M_PI_2 }, DHP{ core::JointType::Revolute, -400.0, 0.0, 0.0, -M_PI_2 }, 500.0 },  //
            DataForGetDistanceFromA3ToWrist{
                DHP{ core::JointType::Revolute, 0.0, -M_PI_2, -300.0, M_PI_2 }, DHP{ core::JointType::Revolute, 400.0, 0.0, 0.0, -M_PI_2 }, 500.0 },  //
            DataForGetDistanceFromA3ToWrist{
                DHP{ core::JointType::Revolute, 0.0, -M_PI_2, -300.0, M_PI_2 }, DHP{ core::JointType::Revolute, -400.0, 0.0, 0.0, -M_PI_2 }, 500.0 },  //
            DataForGetDistanceFromA3ToWrist{
                DHP{ core::JointType::Revolute, 0.0, -M_PI_2, 0.0, M_PI_2 }, DHP{ core::JointType::Revolute, 400.0, 0.0, 0.0, -M_PI_2 }, 400.0 },  //
            DataForGetDistanceFromA3ToWrist{
                DHP{ core::JointType::Revolute, 0.0, -M_PI_2, 300.0, M_PI_2 }, DHP{ core::JointType::Revolute, 0.0, 0.0, 0.0, -M_PI_2 }, 300.0 },  //
            DataForGetDistanceFromA3ToWrist{
                DHP{ core::JointType::Revolute, 0.0, -M_PI_2, 0.0, M_PI_2 }, DHP{ core::JointType::Revolute, 0.0, 0.0, 0.0, -M_PI_2 }, 0.0 },  //
            DataForGetDistanceFromA3ToWrist{
                DHP{ core::JointType::Revolute, 300.0, -M_PI_2, 1200.0, M_PI_2 }, DHP{ core::JointType::Revolute, 400.0, 0.0, 0.0, -M_PI_2 }, 1300.0 },  //
            DataForGetDistanceFromA3ToWrist{
                DHP{ core::JointType::Revolute, -300.0, -M_PI_2, -1200.0, M_PI_2 }, DHP{ core::JointType::Revolute, -400.0, 0.0, 0.0, -M_PI_2 }, 1300.0 },  //
            DataForGetDistanceFromA3ToWrist{ DHP{ core::JointType::Revolute, 0.0, -M_PI_2 * 1.23452, 300.0, M_PI_2 * 54.23423 },
                                             DHP{ core::JointType::Revolute, 400.0, 0.0, 0.0, -M_PI_2 * 342.0239 },
                                             500.0 }  //
            ));

  }  // namespace core
}  // namespace kinverse
