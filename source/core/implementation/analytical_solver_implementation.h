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

#include "../include/kinverse/core/exports.h"
#include "../include/kinverse/core/robot.h"

namespace kinverse {
  namespace core {

    /**
     * Note that this solver works if robot has the following kinematic diagram:
     *
     *                     A4          A5 and A6
     *                     *---------------*---------> FLANGE
     *   A2                |
     *    *----------------* A3
     *    |
     *    |
     *    |
     *    * A1
     *
     * Following kinematic diagrams not tested and not guaranteed to work:
     *
     *   A2            A3 and A4       A5 and A6
     *    *----------------*---------------*---------> FLANGE
     *    |
     *    |
     *    |
     *    * A1
     *
     *   A2               A3
     *    *----------------*
     *    |                |           A5 and A6
     *    |             A4 *---------------*---------> FLANGE
     *    |
     *    * A1
     *
     * We also do not consider exotic cases when A5 is placed behind A4.
     */
    class KINVERSE_CORE_API AnalyticalSolverImplementation {
     public:
      using Ptr = std::shared_ptr<AnalyticalSolverImplementation>;
      using ConstPtr = std::shared_ptr<const AnalyticalSolverImplementation>;

      explicit AnalyticalSolverImplementation(Robot::ConstPtr robot);

      double getGamma() const;
      double getDistanceFromA3ToA2() const;
      double getDistanceFromA3ToWrist() const;
      Eigen::Vector3d getA1ZAxis() const;

      //@todo make @p convertWorldToA1Local method part of Robot class
      Eigen::Affine3d convertWorldToA1Local(const Eigen::Affine3d& transform) const;
      //@todo make @p convertTCPToFlange method part of Robot class
      Eigen::Affine3d convertTCPToFlange(const Eigen::Affine3d& tcpTransform) const;

      Eigen::Vector3d computeWristPosition(const Eigen::Affine3d& flangeTransform) const;

     private:
      double computeGamma(Robot::ConstPtr robot) const;

      Robot::ConstPtr m_robot{ nullptr };
      /**
       * @brief Stores angle between robot's Z4 axis and direction from third joint to wrist position.
       */
      double m_gamma{ 0.0 };
      double m_distanceFromA3ToA2{ 0.0 };
      double m_distanceFromA3ToWrist{ 0.0 };
      Eigen::Affine3d m_worldToA1LocalTransform{ Eigen::Affine3d::Identity() };
    };

  }  // namespace core
}  // namespace kinverse
