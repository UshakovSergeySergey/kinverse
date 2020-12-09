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
#include "../include/kinverse/visualization/coordinate_frame_gizmo.h"
#include "i_gizmo_impl.h"

kinverse::visualization::CoordinateFrameGizmo::CoordinateFrameGizmo(
    const IGizmo* parentGizmo, const Eigen::Affine3d& transform, const std::string& caption, double axesLength, const std::array<std::string, 3>& axesLabels) :
    IGizmo{ parentGizmo } {
  m_pImpl->setViewProp(vtkSmartPointer<vtkAxesActor>::New());
  m_captionGizmo = std::make_shared<Text3DGizmo>(this);
  m_captionGizmo->setFontSize(30);

  setTransform(transform);
  setCaption(caption);
  setAxesLength(axesLength);
  setAxesLabels(axesLabels);
  scaleLabels(0.2);
}

void kinverse::visualization::CoordinateFrameGizmo::setCaption(const std::string& caption) const {
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
  auto actor = vtkAxesActor::SafeDownCast(m_pImpl->getViewProp());
  actor->SetXAxisLabelText(m_axesLabels[0].c_str());
  actor->SetYAxisLabelText(m_axesLabels[1].c_str());
  actor->SetZAxisLabelText(m_axesLabels[2].c_str());

  render();
}

std::array<std::string, 3> kinverse::visualization::CoordinateFrameGizmo::getAxesLabels() const {
  return m_axesLabels;
}

void kinverse::visualization::CoordinateFrameGizmo::setAxesLength(double axesLength) {
  m_axesLength = axesLength;
  updateTransform();
}

double kinverse::visualization::CoordinateFrameGizmo::getAxesLength() const {
  return m_axesLength;
}

void kinverse::visualization::CoordinateFrameGizmo::updateTransform() {
  // Eigen uses column major order for storing matrices, and VTK uses row major order, so we need to transpose matrix
  const Eigen::Matrix4d matrix = m_transform.matrix().transpose();

  vtkSmartPointer<vtkTransform> csTransform = vtkSmartPointer<vtkTransform>::New();
  csTransform->SetMatrix(matrix.data());

  // length unit in kinverse equals 1 millimeter
  const double scale = m_axesLength;
  csTransform->Scale(scale, scale, scale);

  vtkAxesActor::SafeDownCast(m_pImpl->getViewProp())->SetUserTransform(csTransform);

  render();
}

void kinverse::visualization::CoordinateFrameGizmo::scaleLabels(double scale) const {
  auto actor = vtkAxesActor::SafeDownCast(m_pImpl->getViewProp());
  for (auto label : { actor->GetXAxisCaptionActor2D(), actor->GetYAxisCaptionActor2D(), actor->GetZAxisCaptionActor2D() }) {
    label->SetWidth(label->GetWidth() * scale);
    label->SetHeight(label->GetHeight() * scale);
  }
}

void kinverse::visualization::CoordinateFrameGizmo::show(void* renderer) {
  IGizmo::show(renderer);
  IGizmo::show(m_captionGizmo, renderer);
}
