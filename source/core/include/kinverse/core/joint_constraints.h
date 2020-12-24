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

namespace kinverse {
  namespace core {

    /**
     * @class JointConstraints
     * @brief This class holds joint constraints. We have two constraints:
     * axis value constraint and velocity constraint.
     * By default constraints are set to infinity, so that there is no constraints at all.
     */
    class KINVERSE_CORE_API JointConstraints {
     public:
      /**
       * @brief Default constructor. Initializes all variables to infinity,
       * so that there is no constraints at all.
       */
      JointConstraints() = default;

      /**
       * @brief Simple constructor that allows to set constraint values.
       * @param[in] maximumSpeed - maximum speed in radians (millimeters) per second in case of revolute (prismatic) joint
       * @param[in] minimumAxisValue - minimum axis value in radians (millimeters) in case of revolute (prismatic) joint
       * @param[in] maximumAxisValue - maximum axis value in radians (millimeters) in case of revolute (prismatic) joint
       */
      JointConstraints(double maximumSpeed, double minimumAxisValue, double maximumAxisValue);

      /**
       * @brief Sets maximum speed in radians (millimeters) per second in case of revolute (prismatic) joint.
       * @param[in] maximumSpeed - maximum speed
       */
      void setMaximumSpeed(double maximumSpeed);

      /**
       * @brief Returns maximum speed in radians (millimeters) per second in case of revolute (prismatic) joint.
       */
      double getMaximumSpeed() const;

      /**
       * @brief Sets minimum axis value in radians (millimeters) in case of revolute (prismatic) joint.
       * @param[in] minimumAxisValue - minimum axis value
       */
      void setMinimumAxisValue(double minimumAxisValue);

      /**
       * @brief Returns minimum axis value in radians (millimeters) in case of revolute (prismatic) joint.
       */
      double getMinimumAxisValue() const;

      /**
       * @brief Sets maximum axis value in radians (millimeters) in case of revolute (prismatic) joint.
       * @param[in] maximumAxisValue - maximum axis value
       */
      void setMaximumAxisValue(double maximumAxisValue);

      /**
       * @brief Returns maximum axis value in radians (millimeters) in case of revolute (prismatic) joint.
       */
      double getMaximumAxisValue() const;

      /**
       * @brief This method checks whether provided speed violates speed constraint.
       * @param[in] speed - axis speed
       * @return Returns true if violates speed constraint, otherwise false.
       */
      bool violatesSpeedConstraint(double speed) const;

      /**
       * @brief This method checks whether provided axis value violates range constraint.
       * @param[in] axisValue - axis value
       * @return Returns true if violates range constraint, otherwise false.
       */
      bool violatesRangeConstraint(double axisValue) const;

      /**
       * @brief This method clamps provided axis value in order to fit range constraint.
       * @param[in] axisValue - axis value to clamp
       * @return Returns clamped axis value.
       */
      double clampAxisValue(double axisValue) const;

     private:
      /**
       * @brief Stores maximum speed in radians per second in case of revolute joint,
       * and millimeters per second in case of prismatic joint.
       */
      double m_maximumSpeed{ std::numeric_limits<double>::max() };

      /**
       * @brief Stores minimal joint value (radians/millimeters in case of revolute/prismatic joint).
       */
      double m_minimumAxisValue{ -std::numeric_limits<double>::max() };

      /**
       * @brief Stores maximum joint value (radians/millimeters in case of revolute/prismatic joint).
       */
      double m_maximumAxisValue{ std::numeric_limits<double>::max() };
    };

    /**
     * @brief Equality operator for JointConstraints class.
     * @param[in] lhs - left side object
     * @param[in] rhs - right side object
     * @return Returns true if objects are equal, false otherwise.
     */
    inline bool KINVERSE_CORE_API operator==(const JointConstraints& lhs, const JointConstraints& rhs);

    /**
     * @brief Inequality operator for JointConstraints class.
     * @param[in] lhs - left side object
     * @param[in] rhs - right side object
     * @return Returns true if objects are not equal, false otherwise.
     */
    inline bool KINVERSE_CORE_API operator!=(const JointConstraints& lhs, const JointConstraints& rhs);

  }  // namespace core
}  // namespace kinverse
