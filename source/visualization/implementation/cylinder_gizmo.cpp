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
#include "../include/kinverse/visualization/cylinder_gizmo.h"
#include "i_gizmo_impl.h"

kinverse::visualization::CylinderGizmo::CylinderGizmo(
    const IGizmo* parentGizmo, const Eigen::Affine3d& transform, double radius, double height, const Color& color) :
    IGizmo{ parentGizmo } {
  m_pImpl->setViewProp(vtkSmartPointer<vtkActor>::New());
  setTransform(transform);
  setRadius(radius);
  setHeight(height);
  setColor(color);
}

void kinverse::visualization::CylinderGizmo::setTransform(const Eigen::Affine3d& transform) {
  m_transform = transform;

  // cylinders are usually drawn along z axis on kinematic diagrams, so lets make this initial cylinder orientation
  Eigen::Affine3d cylinderInitialPosition;
  cylinderInitialPosition = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());

  // Eigen uses column major order for storing matrices, and VTK uses row major order, so we need to transpose matrix
  const Eigen::Matrix4d matrix = (m_transform * cylinderInitialPosition).matrix().transpose();

  vtkSmartPointer<vtkTransform> csTransform = vtkSmartPointer<vtkTransform>::New();
  csTransform->SetMatrix(matrix.data());

  vtkActor::SafeDownCast(m_pImpl->getViewProp())->SetUserTransform(csTransform);

  render();
}

Eigen::Affine3d kinverse::visualization::CylinderGizmo::getTransform() const {
  return m_transform;
}

void kinverse::visualization::CylinderGizmo::setRadius(double radius) {
  m_radius = radius;
  updateGeometry();
}

double kinverse::visualization::CylinderGizmo::getRadius() const {
  return m_radius;
}

void kinverse::visualization::CylinderGizmo::setHeight(double height) {
  m_height = height;
  updateGeometry();
}

double kinverse::visualization::CylinderGizmo::getHeight() const {
  return m_height;
}

void kinverse::visualization::CylinderGizmo::setColor(const Color& color) {
  m_color = color;

  auto actor = vtkActor::SafeDownCast(m_pImpl->getViewProp());

  actor->GetProperty()->SetColor(std::get<0>(m_color) / 255.0, std::get<1>(m_color) / 255.0, std::get<2>(m_color) / 255.0);
  actor->GetProperty()->SetOpacity(std::get<3>(m_color) / 255.0);

  render();
}

kinverse::visualization::Color kinverse::visualization::CylinderGizmo::getColor() const {
  return m_color;
}

void kinverse::visualization::CylinderGizmo::updateGeometry() const {
  vtkSmartPointer<vtkCylinderSource> cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
  cylinderSource->SetCenter(0.0, 0.0, 0.0);
  cylinderSource->SetRadius(m_radius);
  cylinderSource->SetHeight(m_height);
  cylinderSource->SetResolution(100);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(cylinderSource->GetOutputPort());

  vtkActor::SafeDownCast(m_pImpl->getViewProp())->SetMapper(mapper);

  render();
}
