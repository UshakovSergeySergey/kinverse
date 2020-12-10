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
#include "../include/kinverse/math/math.h"

double kinverse::math::degreesToRadians(double angleInDegrees) {
  constexpr double toRadians = M_PI / 180.0;
  return angleInDegrees * toRadians;
}

double kinverse::math::radiansToDegrees(double angleInRadians) {
  constexpr double toDegrees = 180.0 * M_1_PI;
  return angleInRadians * toDegrees;
}

void kinverse::math::toXYZABC(const Eigen::Affine3d& transform, Eigen::Vector3d& xyz, Eigen::Vector3d& abc) {
  xyz = transform.translation();
  abc = Eigen::EulerAnglesZYXd(transform.rotation()).angles();
}

Eigen::Affine3d kinverse::math::fromXYZABC(const Eigen::Vector3d& xyz, const Eigen::Vector3d& abc) {
  return Eigen::Translation3d(xyz) * Eigen::EulerAnglesZYXd(abc.x(), abc.y(), abc.z());
}

bool kinverse::math::pointLiesOnLine(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const Eigen::Vector3d& point) {
  throw std::exception("Not implemented yet!");
}
