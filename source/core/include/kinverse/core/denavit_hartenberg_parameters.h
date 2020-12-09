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

#include "exports.h"
#include "joint_type.h"

namespace kinverse {
  namespace core {

    /**
     * @class DenavitHartenbergParameters
     * @brief This class represents structure that stores Denavit-Hartenberg parameters.
     * It allows to easily transform DH parameters to Eigen::Affine transform.
     * Transform is parameterized with the axis value (angle or distance depending on the joint type).
     */
    class KINVERSE_CORE_API DenavitHartenbergParameters {
     public:
      /**
       * @brief Simple constructor.
       * @param[in] jointType - type of joint
       * @param[in] d - displacement along the previous joint's Z axis
       * @param[in] theta - angle in radians about the previous joint's Z axis
       * @param[in] r - displacement along the rotated X axis
       * @param[in] alpha - angle in radians about the rotated X axis
       */
      DenavitHartenbergParameters(JointType jointType, double d, double theta, double r, double alpha);

      /**
       * @brief This method sets the joint type described with these DH parameters
       * @param[in] jointType - joint type (revolute or prismatic)
       */
      void setJointType(JointType jointType);

      /**
       * @brief Returns type of joint described with these DH parameters.
       */
      JointType getJointType() const;

      /**
       * @brief Returns Denavit-Hartenberg transform from the previous joint's coordinate frame to the current joint coordinate frame.
       * Transform is parameterized with angle/distance depending on the joint type.
       * For example, if you want to get transform of the revolute joint coordinate frame when axis angle equals Pi, just pass this angle as parameter.
       * @param[in] value - joint axis value. If joint type is Prismatic, then it represents displacement along Z axis. If joint type is Revolute, then it
       * represents angle in radians about Z axis.
       * @return Returns transform from the previous joint's coordinate frame to the current joint coordinate frame
       */
      Eigen::Affine3d getTransform(double value = 0.0) const;

     private:
      /**
       * @brief Stores type of joint described by Denavit-Hartenberg parameters.
       */
      JointType m_jointType{ JointType::Revolute };

      /**
       * @brief Stores displacement along the previous joint's Z axis
       */
      double m_d{ 0.0 };

      /**
       * @brief Stores angle in radians about the previous joint's Z axis
       */
      double m_theta{ 0.0 };

      /**
       * @brief Stores displacement along the rotated X axis
       */
      double m_r{ 0.0 };

      /**
       * @brief Stores angle in radians about the rotated X axis
       */
      double m_alpha{ 0.0 };
    };

  }  // namespace core
}  // namespace kinverse
