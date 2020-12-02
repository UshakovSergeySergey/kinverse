#include "stdafx.h"
#include "coordinate_frame_gizmo.h"
#include "i_gizmo_impl.h"

kinverse::visualization::CoordinateFrameGizmo::CoordinateFrameGizmo(
    const IGizmo* parentGizmo, const Eigen::Affine3d& transform, const std::string& caption, double scale, const std::array<std::string, 3>& axesLabels) :
    IGizmo{ parentGizmo } {
  m_pImpl->m_actor = vtkSmartPointer<vtkAxesActor>::New();
  m_captionGizmo = std::make_shared<Text3DGizmo>(this);
  m_captionGizmo->setFontSize(30);

  setTransform(transform);
  setCaption(caption);
  setScale(scale);
  setAxesLabels(axesLabels);
  scaleLabels(0.2);
}

void kinverse::visualization::CoordinateFrameGizmo::setCaption(const std::string& caption) {
  m_captionGizmo->setText(caption);
}

std::string kinverse::visualization::CoordinateFrameGizmo::getCaption() const {
  return m_captionGizmo->getText();
}

void kinverse::visualization::CoordinateFrameGizmo::setTransform(const Eigen::Affine3d& transform) {
  m_transform = transform;
  m_captionGizmo->setTransform(transform);
  updateTransform();
}

Eigen::Affine3d kinverse::visualization::CoordinateFrameGizmo::getTransform() const {
  return m_transform;
}

void kinverse::visualization::CoordinateFrameGizmo::setAxesLabels(const std::array<std::string, 3>& axesLabels) {
  m_axesLabels = axesLabels;
  auto actor = vtkAxesActor::SafeDownCast(m_pImpl->m_actor);
  actor->SetXAxisLabelText(m_axesLabels[0].c_str());
  actor->SetYAxisLabelText(m_axesLabels[1].c_str());
  actor->SetZAxisLabelText(m_axesLabels[2].c_str());

  update();
}

std::array<std::string, 3> kinverse::visualization::CoordinateFrameGizmo::getAxesLabels() const {
  return m_axesLabels;
}

void kinverse::visualization::CoordinateFrameGizmo::setScale(double scale) {
  m_scale = scale;
  updateTransform();
}

double kinverse::visualization::CoordinateFrameGizmo::getScale() const {
  return m_scale;
}

void kinverse::visualization::CoordinateFrameGizmo::updateTransform() {
  // Eigen uses column major order for storing matrices, and VTK uses row major order, so we need to transpose matrix
  const Eigen::Matrix4d matrix = m_transform.matrix().transpose();

  vtkSmartPointer<vtkTransform> csTransform = vtkSmartPointer<vtkTransform>::New();
  csTransform->SetMatrix(matrix.data());

  // length unit in kinverse equals 1 millimeter, here we are setting axis length to 1000 mm (1 meter)
  const double scale = 1000.0 * m_scale;
  csTransform->Scale(scale, scale, scale);

  vtkAxesActor::SafeDownCast(m_pImpl->m_actor)->SetUserTransform(csTransform);

  update();
}

void kinverse::visualization::CoordinateFrameGizmo::scaleLabels(double scale) {
  auto actor = vtkAxesActor::SafeDownCast(m_pImpl->m_actor);
  for (auto label : { actor->GetXAxisCaptionActor2D(), actor->GetYAxisCaptionActor2D(), actor->GetZAxisCaptionActor2D() }) {
    label->SetWidth(label->GetWidth() * scale);
    label->SetHeight(label->GetHeight() * scale);
  }
}

void kinverse::visualization::CoordinateFrameGizmo::show(void* renderer) {
  IGizmo::show(renderer);
  m_captionGizmo->show(renderer);
}
