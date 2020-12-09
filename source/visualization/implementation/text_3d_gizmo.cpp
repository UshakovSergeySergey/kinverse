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
#include "../include/kinverse/visualization/text_3d_gizmo.h"
#include "i_gizmo_impl.h"
#include "vtkCallbackCommandWrapper.h"

kinverse::visualization::Text3DGizmo::Text3DGizmo(const IGizmo* parentGizmo,
                                                  const Eigen::Affine3d& transform,
                                                  const std::string& text,
                                                  int fontSize,
                                                  const Color& color,
                                                  bool faceTowardsCamera,
                                                  bool viewportConstScale) :
    IGizmo{ parentGizmo } {
  m_pImpl->setViewProp(vtkSmartPointer<vtkTextActor3D>::New());
  setTransform(transform);
  setText(text);
  setFontSize(fontSize);
  setColor(color);
  setFaceTowardsCamera(faceTowardsCamera);
  setViewportConstScale(viewportConstScale);
}

void kinverse::visualization::Text3DGizmo::setTransform(const Eigen::Affine3d& transform) {
  m_transform = transform;
  updateTextOrientationAndScale(true);
}

Eigen::Affine3d kinverse::visualization::Text3DGizmo::getTransform() const {
  return m_transform;
}

void kinverse::visualization::Text3DGizmo::setText(const std::string& text) {
  m_text = text;
  auto text3d = vtkTextActor3D::SafeDownCast(m_pImpl->getViewProp());
  text3d->SetInput(m_text.c_str());

  render();
}

std::string kinverse::visualization::Text3DGizmo::getText() const {
  return m_text;
}

void kinverse::visualization::Text3DGizmo::setFontSize(int fontSize) {
  m_fontSize = fontSize;
  auto text3d = vtkTextActor3D::SafeDownCast(m_pImpl->getViewProp());
  text3d->GetTextProperty()->SetFontSize(m_fontSize);

  render();
}

int kinverse::visualization::Text3DGizmo::getFontSize() const {
  return m_fontSize;
}

void kinverse::visualization::Text3DGizmo::setColor(const Color& color) {
  m_color = color;
  auto text3d = vtkTextActor3D::SafeDownCast(m_pImpl->getViewProp());
  text3d->GetTextProperty()->SetColor(std::get<0>(m_color) / 255.0, std::get<1>(m_color) / 255.0, std::get<2>(m_color) / 255.0);
  text3d->GetTextProperty()->SetOpacity(std::get<3>(m_color) / 255.0);

  render();
}

kinverse::visualization::Color kinverse::visualization::Text3DGizmo::getColor() const {
  return m_color;
}

void kinverse::visualization::Text3DGizmo::setFaceTowardsCamera(bool faceTowardsCamera) {
  m_faceTowardsCamera = faceTowardsCamera;
  updateSubscriptionForCameraEvents();
}

bool kinverse::visualization::Text3DGizmo::getFaceTowardsCamera() const {
  return m_faceTowardsCamera;
}

void kinverse::visualization::Text3DGizmo::setViewportConstScale(bool viewportConstScale) {
  m_viewportConstScale = viewportConstScale;
  updateSubscriptionForCameraEvents();
}

bool kinverse::visualization::Text3DGizmo::getViewportConstScale() const {
  return m_viewportConstScale;
}

void kinverse::visualization::Text3DGizmo::show(void* renderer) {
  IGizmo::show(renderer);
  updateSubscriptionForCameraEvents();
}

void kinverse::visualization::Text3DGizmo::updateSubscriptionForCameraEvents() {
  if (!m_pImpl || !m_pImpl->getRenderer())
    return;

  const auto callback = [this](vtkObject* caller, unsigned long eid, void*) {
    switch (eid) {
      case vtkCommand::ResetCameraClippingRangeEvent:
      case vtkCommand::ModifiedEvent:
      case vtkCommand::ComputeVisiblePropBoundsEvent:
        break;
      default:
        return;
    }
    this->updateTextOrientationAndScale(false);
  };

  const auto eventType = vtkCommand::EventIds::AnyEvent;
  vtkSmartPointer<vtkCallbackCommandWrapper> cameraEventCallback = vtkSmartPointer<vtkCallbackCommandWrapper>::New();
  cameraEventCallback->SetCustomCallback(callback);

  const bool needToSubscribe = m_faceTowardsCamera || m_viewportConstScale;
  const bool alreadySubscribed = m_pImpl->getRenderer()->HasObserver(eventType, cameraEventCallback);

  if (!needToSubscribe) {
    if (alreadySubscribed)
      m_pImpl->getRenderer()->RemoveObserver(cameraEventCallback);
  } else {
    if (!alreadySubscribed)
      m_pImpl->getRenderer()->AddObserver(eventType, cameraEventCallback);
  }

  updateTextOrientationAndScale(true);
}

void kinverse::visualization::Text3DGizmo::updateTextOrientationAndScale(bool needsToBeRerendered) {
  Eigen::Vector3d cameraPosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d cameraUpVector = Eigen::Vector3d::Zero();

  const bool cameraIsAvailable = getCameraPositionAndOrientation(cameraPosition, cameraUpVector);

  const Eigen::Vector3d textPosition = m_transform.translation();
  Eigen::Matrix3d textRotation = m_transform.rotation();
  Eigen::Vector3d textScale = Eigen::Vector3d::Ones();

  if (m_faceTowardsCamera && cameraIsAvailable) {
    const Eigen::Vector3d zDirection = (cameraPosition - textPosition).normalized();
    const Eigen::Vector3d yDirection = cameraUpVector.normalized();
    const Eigen::Vector3d xDirection = yDirection.cross(zDirection).normalized();

    textRotation.col(0) = xDirection;
    textRotation.col(1) = yDirection;
    textRotation.col(2) = zDirection;
  }

  if (m_viewportConstScale && cameraIsAvailable) {
    textScale *= (cameraPosition - textPosition).norm() / 1419.62;
  }

  Eigen::Affine3d transform;
  transform.fromPositionOrientationScale(textPosition, textRotation, textScale);

  updateTransform(transform, needsToBeRerendered);
}

bool kinverse::visualization::Text3DGizmo::getCameraPositionAndOrientation(Eigen::Vector3d& cameraPosition, Eigen::Vector3d& cameraUpVector) const {
  if (!m_pImpl || !m_pImpl->getRenderer())
    return false;

  auto camera = m_pImpl->getRenderer()->GetActiveCamera();
  camera->GetPosition(cameraPosition.data());
  camera->GetViewUp(cameraUpVector.data());

  return true;
}

void kinverse::visualization::Text3DGizmo::updateTransform(const Eigen::Affine3d& transform, bool needsToBeRerendered) const {
  // This method must render only if @p setTransform is called. If this method is called
  // from subscription, then there is no need to rerender, because the caller will do everything.

  // Eigen uses column major order for storing matrices, and VTK uses row major order, so we need to transpose matrix
  const Eigen::Matrix4d matrix = transform.matrix().transpose();

  vtkSmartPointer<vtkTransform> textTransform = vtkSmartPointer<vtkTransform>::New();
  textTransform->SetMatrix(matrix.data());

  vtkTextActor3D::SafeDownCast(m_pImpl->getViewProp())->SetUserTransform(textTransform);

  if (needsToBeRerendered)
    render();
}
