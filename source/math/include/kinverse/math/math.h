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
  namespace math {

    /**
     * @brief Convert degrees to radians.
     * @param[in] angleInDegrees - angle in degrees
     * @return Returns angle in radians.
     */
    double KINVERSE_MATH_API degreesToRadians(double angleInDegrees);

    /**
     * @brief Convert radians to degrees.
     * @param[in] angleInRadians - angle in radians
     * @return Returns angle in degrees.
     */
    double KINVERSE_MATH_API radiansToDegrees(double angleInRadians);

    /**
     * @brief This method converts transform to (x, y, z) position and (a, b, c) rotation.
     * Note that this method uses ZYX euler angle notation. So a corresponds to rotation about Z axis,
     * b corresponds to rotation about Y axis and c corresponds to rotation about X axis
     * @param[in] transform - transform from which position and rotation must be extracted
     * @param[out] xyz - position
     * @param[out] abc - rotation
     */
    void KINVERSE_MATH_API toXYZABC(const Eigen::Affine3d& transform, Eigen::Vector3d& xyz, Eigen::Vector3d& abc);

    /**
     * @brief This method converts (x, y, z) position and (a, b, c) rotation to transform.
     * Note that this method uses ZYX euler angle notation. So a corresponds to rotation about Z axis,
     * b corresponds to rotation about Y axis and c corresponds to rotation about X axis
     * @param[in] xyz - position
     * @param[in] abc - rotation
     * @return Returns transform.
     */
    Eigen::Affine3d KINVERSE_MATH_API fromXYZABC(const Eigen::Vector3d& xyz, const Eigen::Vector3d& abc);

    /**
     * @brief This method checks if the point lies on the line represented with (origin, direction).
     * @param[in] origin - line origin
     * @param[in] direction - line direction
     * @param[in] point - point to check
     * @return Returns true if point lies on the given line, false otherwise.
     */
    bool KINVERSE_MATH_API pointLiesOnLine(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const Eigen::Vector3d& point);

    /**
     * @brief This method compares two double values using predefined epsilon value.
     * This epsilon value defines accuracy of double comparisons.
     * @param[in] lhs - first double value
     * @param[in] rhs - second double value
     * @return Returns true if two values are almost equal
     * (distance between them is within predefined epsilon), false otherwise.
     */
    bool KINVERSE_MATH_API doublesAreEqual(double lhs, double rhs);

    /**
     * @brief This method checks whether two points are equal. In order to do that it computes distance between
     * these points and checks if it is lower than predefined epsilon value.
     * @param[in] p1 - first point
     * @param[in] p2 - second point
     * @return Returns true if two points are almost equal
     * (distance between them is within predefined epsilon), false otherwise.
     */
    bool KINVERSE_MATH_API pointsAreEqual(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

  }  // namespace math
}  // namespace kinverse
