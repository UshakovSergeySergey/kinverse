#include "stdafx.h"
#include "cube_gizmo.h"
#include "i_gizmo_impl.h"

kinverse::visualization::CubeGizmo::CubeGizmo(
    const IGizmo* parentGizmo, const Eigen::Affine3d& transform, double width, double height, double depth, const Color& color) :
    IGizmo{ parentGizmo } {
  m_pImpl->m_actor = vtkSmartPointer<vtkActor>::New();
  setTransform(transform);
  setColor(color);
  setWidth(width);
  setHeight(height);
  setDepth(depth);
}

void kinverse::visualization::CubeGizmo::setTransform(const Eigen::Affine3d& transform) {
  m_transform = transform;

  // Eigen uses column major order for storing matrices, and VTK uses row major order, so we need to transpose matrix
  const Eigen::Matrix4d matrix = m_transform.matrix().transpose();

  vtkSmartPointer<vtkTransform> cubeTransform = vtkSmartPointer<vtkTransform>::New();
  cubeTransform->SetMatrix(matrix.data());

  vtkActor::SafeDownCast(m_pImpl->m_actor)->SetUserTransform(cubeTransform);

  update();
}

Eigen::Affine3d kinverse::visualization::CubeGizmo::getTransform() const {
  return m_transform;
}

void kinverse::visualization::CubeGizmo::setColor(const Color& color) {
  m_color = color;

  auto actor = vtkActor::SafeDownCast(m_pImpl->m_actor);

  actor->GetProperty()->SetColor(std::get<0>(m_color) / 255.0, std::get<1>(m_color) / 255.0, std::get<2>(m_color) / 255.0);
  actor->GetProperty()->SetOpacity(std::get<3>(m_color) / 255.0);

  update();
}

kinverse::visualization::Color kinverse::visualization::CubeGizmo::getColor() const {
  return m_color;
}

void kinverse::visualization::CubeGizmo::setWidth(double width) {
  m_width = width;
  updateGeometry();
}

double kinverse::visualization::CubeGizmo::getWidth() const {
  return m_width;
}

void kinverse::visualization::CubeGizmo::setDepth(double depth) {
  m_depth = depth;
  updateGeometry();
}

double kinverse::visualization::CubeGizmo::getDepth() const {
  return m_depth;
}

void kinverse::visualization::CubeGizmo::setHeight(double height) {
  m_height = height;
  updateGeometry();
}

double kinverse::visualization::CubeGizmo::getHeight() const {
  return m_height;
}

void kinverse::visualization::CubeGizmo::updateGeometry() {
  vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
  cubeSource->SetXLength(m_width);
  cubeSource->SetYLength(m_depth);
  cubeSource->SetZLength(m_height);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(cubeSource->GetOutputPort());

  vtkActor::SafeDownCast(m_pImpl->m_actor)->SetMapper(mapper);

  update();
}
