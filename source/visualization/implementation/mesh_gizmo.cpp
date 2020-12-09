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
#include "../include/kinverse/visualization/mesh_gizmo.h"
#include "i_gizmo_impl.h"
#include "mesh_to_vtk_poly_data_converter.h"

kinverse::visualization::MeshGizmo::MeshGizmo(const IGizmo* parentGizmo, core::Mesh::ConstPtr mesh, const Eigen::Affine3d& transform, const Color& color) :
    IGizmo{ parentGizmo } {
  m_pImpl->setViewProp(vtkSmartPointer<vtkActor>::New());
  setMesh(mesh);
  setTransform(transform);
  setColor(color);
}

void kinverse::visualization::MeshGizmo::setMesh(core::Mesh::ConstPtr mesh) {
  m_mesh = mesh;

  const vtkSmartPointer<vtkPolyData> vtkMesh = MeshToVtkPolyDataConverter::convert(mesh);

  vtkSmartPointer<vtkPolyDataMapper> meshMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  meshMapper->SetInputData(vtkMesh);

  vtkActor::SafeDownCast(m_pImpl->getViewProp())->SetMapper(meshMapper);

  render();
}

kinverse::core::Mesh::ConstPtr kinverse::visualization::MeshGizmo::getMesh() const {
  return m_mesh;
}

void kinverse::visualization::MeshGizmo::setTransform(const Eigen::Affine3d& transform) {
  m_transform = transform;

  // Eigen uses column major order for storing matrices, and VTK uses row major order, so we need to transpose matrix
  const Eigen::Matrix4d matrix = m_transform.matrix().transpose();

  vtkSmartPointer<vtkTransform> meshTransform = vtkSmartPointer<vtkTransform>::New();
  meshTransform->SetMatrix(matrix.data());

  vtkActor::SafeDownCast(m_pImpl->getViewProp())->SetUserTransform(meshTransform);

  render();
}

Eigen::Affine3d kinverse::visualization::MeshGizmo::getTransform() const {
  return m_transform;
}

void kinverse::visualization::MeshGizmo::setColor(const Color& color) {
  m_color = color;

  auto actor = vtkActor::SafeDownCast(m_pImpl->getViewProp());

  actor->GetProperty()->SetColor(std::get<0>(m_color) / 255.0, std::get<1>(m_color) / 255.0, std::get<2>(m_color) / 255.0);
  actor->GetProperty()->SetOpacity(std::get<3>(m_color) / 255.0);

  render();
}

kinverse::visualization::Color kinverse::visualization::MeshGizmo::getColor() const {
  return m_color;
}
