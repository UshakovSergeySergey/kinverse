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
       * @param[in] zAxisDisplacement - displacement along the previous joint's Z axis (also known as D parameter)
       * @param[in] zAxisRotation - angle in radians about the previous joint's Z axis (also known as Theta parameter)
       * @param[in] xAxisDisplacement - displacement along the rotated X axis (also known as R parameter)
       * @param[in] xAxisRotation - angle in radians about the rotated X axis (also known as Alpha parameter)
       */
      DenavitHartenbergParameters(JointType jointType = JointType::Revolute,
                                  double zAxisDisplacement = 0.0,
                                  double zAxisRotation = 0.0,
                                  double xAxisDisplacement = 0.0,
                                  double xAxisRotation = 0.0);

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
       * @brief Sets displacement along the previous joint's Z axis (also known as D parameter)
       * @param[in] displacement - displacement
       */
      void setZAxisDisplacement(double displacement);

      /**
       * @brief Returns displacement along the previous joint's Z axis (also known as D parameter)
       */
      double getZAxisDisplacement() const;

      /**
       * @brief Sets angle in radians about the previous joint's Z axis (also known as Theta parameter)
       * @param[in] angle - angle
       */
      void setZAxisRotation(double angle);

      /**
       * @brief Returns angle in radians about the previous joint's Z axis (also known as Theta parameter)
       */
      double getZAxisRotation() const;

      /**
       * @brief Sets displacement along the rotated X axis (also known as R parameter)
       * @param[in] displacement - displacement
       */
      void setXAxisDisplacement(double displacement);

      /**
       * @brief Returns displacement along the rotated X axis (also known as R parameter)
       */
      double getXAxisDisplacement() const;

      /**
       * @brief Sets angle in radians about the rotated X axis (also known as Alpha parameter)
       * @param[in] angle - angle
       */
      void setXAxisRotation(double angle);

      /**
       * @brief Returns angle in radians about the rotated X axis (also known as Alpha parameter)
       */
      double getXAxisRotation() const;

      /**
       * @brief Returns Denavit-Hartenberg transform from the previous joint's coordinate frame to the current joint coordinate frame.
       * Transform is parameterized with angle/distance depending on the joint type.
       * For example, if you want to get transform of the revolute joint coordinate frame when axis angle equals Pi, just pass this angle as parameter.
       * @param[in] value - joint axis value. If joint type is Prismatic, then it represents displacement along Z axis. If joint type is Revolute, then it
       * represents angle in radians about Z axis.
       * @return Returns transform from the previous joint's coordinate frame to the current joint coordinate frame
       */
      Eigen::Affine3d getTransform(double value = 0.0) const;

      /**
       * @brief Transform from the previous joint's coordinate frame to the current joint coordinate frame consists of two parts:
       * rotation and translation along Z axis, rotation and translation along X axis.
       * This method returns Z part of transform.
       * @param[in] value - joint axis value. If joint type is Prismatic, then it represents displacement along Z axis. If joint type is Revolute, then it
       * represents angle in radians about Z axis.
       */
      Eigen::Affine3d getTransformZ(double value = 0.0) const;

      /**
       * @brief Transform from the previous joint's coordinate frame to the current joint coordinate frame consists of two parts:
       * rotation and translation along Z axis, rotation and translation along X axis.
       * This method returns X part of transform.
       */
      Eigen::Affine3d getTransformX() const;

     private:
      /**
       * @brief Stores type of joint described by Denavit-Hartenberg parameters.
       */
      JointType m_jointType{ JointType::Revolute };

      /**
       * @brief Stores displacement along the previous joint's Z axis (also known as D parameter)
       */
      double m_zAxisDisplacement{ 0.0 };

      /**
       * @brief Stores angle in radians about the previous joint's Z axis (also known as Theta parameter)
       */
      double m_zAxisRotation{ 0.0 };

      /**
       * @brief Stores displacement along the rotated X axis (also known as R parameter)
       */
      double m_xAxisDisplacement{ 0.0 };

      /**
       * @brief Stores angle in radians about the rotated X axis (also known as Alpha parameter)
       */
      double m_xAxisRotation{ 0.0 };
    };

    /**
     * @brief Equality operator for Denavit-Hartenberg parameters class.
     * @param[in] lhs - left side object
     * @param[in] rhs - right side object
     * @return Returns true if objects are equal, false otherwise.
     */
    inline bool KINVERSE_CORE_API operator==(const DenavitHartenbergParameters& lhs, const DenavitHartenbergParameters& rhs);

    /**
     * @brief Inequality operator for Denavit-Hartenberg parameters class.
     * @param[in] lhs - left side object
     * @param[in] rhs - right side object
     * @return Returns true if objects are not equal, false otherwise.
     */
    inline bool KINVERSE_CORE_API operator!=(const DenavitHartenbergParameters& lhs, const DenavitHartenbergParameters& rhs);

  }  // namespace core
}  // namespace kinverse
