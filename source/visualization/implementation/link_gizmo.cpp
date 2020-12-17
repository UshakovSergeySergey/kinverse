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
#include "../include/kinverse/visualization/link_gizmo.h"

kinverse::visualization::LinkGizmo::LinkGizmo(const IGizmo* parentGizmo,
                                              const core::DenavitHartenbergParameters& dhParameters,
                                              const Eigen::Affine3d& transform) :
    IGizmo{ parentGizmo } {
  m_xAxisDisplacementGizmo = std::make_shared<CylinderGizmo>(this);
  m_xAxisDisplacementGizmo->setRadius(5.0);

  m_zAxisDisplacementGizmo = std::make_shared<CylinderGizmo>(this);
  m_zAxisDisplacementGizmo->setRadius(5.0);

  setDHParameters(dhParameters);
  setTransform(transform);
}

void kinverse::visualization::LinkGizmo::setTransform(const Eigen::Affine3d& transform) {
  m_transform = transform;
  updateTransform();
}

Eigen::Affine3d kinverse::visualization::LinkGizmo::getTransform() const {
  return m_transform;
}

void kinverse::visualization::LinkGizmo::setDHParameters(const core::DenavitHartenbergParameters& dhParameters) {
  m_dhParameters = dhParameters;

  const double zAxisDisplacement = m_dhParameters.getZAxisDisplacement();
  m_zAxisDisplacementGizmo->setHeight(std::abs(zAxisDisplacement));
  m_zInitialTransform = Eigen::Translation3d(0.0, 0.0, zAxisDisplacement * 0.5);
  if (std::abs(zAxisDisplacement) < std::numeric_limits<double>::epsilon())
    m_zAxisDisplacementGizmo->setColor(Color{ 255, 255, 255, 0 });
  else
    m_zAxisDisplacementGizmo->setColor(Color{ 255, 255, 255, 255 });

  const double xAxisDisplacement = m_dhParameters.getXAxisDisplacement();
  m_xAxisDisplacementGizmo->setHeight(std::abs(xAxisDisplacement));
  m_xInitialTransform =
      dhParameters.getTransformZ() * Eigen::Translation3d(xAxisDisplacement * 0.5, 0.0, 0.0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
  if (std::abs(xAxisDisplacement) < std::numeric_limits<double>::epsilon())
    m_xAxisDisplacementGizmo->setColor(Color{ 255, 255, 255, 0 });
  else
    m_xAxisDisplacementGizmo->setColor(Color{ 255, 255, 255, 255 });

  updateTransform();
}

kinverse::core::DenavitHartenbergParameters kinverse::visualization::LinkGizmo::getDHParameters() const {
  return m_dhParameters;
}

void kinverse::visualization::LinkGizmo::show(void* renderer) {
  IGizmo::show(m_zAxisDisplacementGizmo, renderer);
  IGizmo::show(m_xAxisDisplacementGizmo, renderer);
  IGizmo::show(renderer);
}

void kinverse::visualization::LinkGizmo::hide(void* renderer) {
  IGizmo::hide(m_zAxisDisplacementGizmo, renderer);
  IGizmo::hide(m_xAxisDisplacementGizmo, renderer);
  IGizmo::hide(renderer);
}

void kinverse::visualization::LinkGizmo::updateTransform() const {
  m_zAxisDisplacementGizmo->setTransform(m_transform * m_zInitialTransform);
  m_xAxisDisplacementGizmo->setTransform(m_transform * m_xInitialTransform);

  render();
}
