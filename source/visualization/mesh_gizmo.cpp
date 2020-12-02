#include "stdafx.h"
#include "mesh_gizmo.h"
#include "i_gizmo_impl.h"
#include "mesh_to_vtk_poly_data_converter.h"

kinverse::visualization::MeshGizmo::MeshGizmo(const IGizmo* parentGizmo, core::Mesh::ConstPtr mesh, const Eigen::Affine3d& transform, const Color& color) :
    IGizmo{ parentGizmo } {
  m_pImpl->m_actor = vtkSmartPointer<vtkActor>::New();
  setMesh(mesh);
  setTransform(transform);
  setColor(color);
}

void kinverse::visualization::MeshGizmo::setMesh(core::Mesh::ConstPtr mesh) {
  m_mesh = mesh;

  vtkSmartPointer<vtkPolyData> vtkMesh = MeshToVtkPolyDataConverter::convert(mesh);

  vtkSmartPointer<vtkPolyDataMapper> meshMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  meshMapper->SetInputData(vtkMesh);

  vtkActor::SafeDownCast(m_pImpl->m_actor)->SetMapper(meshMapper);

  update();
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

  vtkActor::SafeDownCast(m_pImpl->m_actor)->SetUserTransform(meshTransform);

  update();
}

Eigen::Affine3d kinverse::visualization::MeshGizmo::getTransform() const {
  return m_transform;
}

void kinverse::visualization::MeshGizmo::setColor(const Color& color) {
  m_color = color;

  auto actor = vtkActor::SafeDownCast(m_pImpl->m_actor);

  actor->GetProperty()->SetColor(std::get<0>(m_color) / 255.0, std::get<1>(m_color) / 255.0, std::get<2>(m_color) / 255.0);
  actor->GetProperty()->SetOpacity(std::get<3>(m_color) / 255.0);

  update();
}

kinverse::visualization::Color kinverse::visualization::MeshGizmo::getColor() const {
  return m_color;
}
