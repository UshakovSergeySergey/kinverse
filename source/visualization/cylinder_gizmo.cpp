#include "stdafx.h"
#include "cylinder_gizmo.h"
#include "i_gizmo_impl.h"

kinverse::visualization::CylinderGizmo::CylinderGizmo(
    const IGizmo* parentGizmo, const Eigen::Affine3d& transform, double radius, double height, const Color& color) :
    IGizmo{ parentGizmo } {
  m_pImpl->m_actor = vtkSmartPointer<vtkActor>::New();
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

  vtkActor::SafeDownCast(m_pImpl->m_actor)->SetUserTransform(csTransform);

  update();
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

  auto actor = vtkActor::SafeDownCast(m_pImpl->m_actor);

  actor->GetProperty()->SetColor(std::get<0>(m_color) / 255.0, std::get<1>(m_color) / 255.0, std::get<2>(m_color) / 255.0);
  actor->GetProperty()->SetOpacity(std::get<3>(m_color) / 255.0);

  update();
}

kinverse::visualization::Color kinverse::visualization::CylinderGizmo::getColor() const {
  return m_color;
}

void kinverse::visualization::CylinderGizmo::updateGeometry() {
  vtkSmartPointer<vtkCylinderSource> cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
  cylinderSource->SetCenter(0.0, 0.0, 0.0);
  cylinderSource->SetRadius(m_radius);
  cylinderSource->SetHeight(m_height);
  cylinderSource->SetResolution(100);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(cylinderSource->GetOutputPort());

  vtkActor::SafeDownCast(m_pImpl->m_actor)->SetMapper(mapper);

  update();
}
