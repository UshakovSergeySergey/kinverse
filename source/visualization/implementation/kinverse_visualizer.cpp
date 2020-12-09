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
#include "../include/kinverse/visualization/kinverse_visualizer.h"
#include "kinverse_visualizer_impl.h"

kinverse::visualization::KinverseVisualizer::KinverseVisualizer() {
  m_pImpl = std::make_shared<KinverseVisualizerImpl>();

  m_pImpl->m_renderer = vtkSmartPointer<vtkRenderer>::New();
  m_pImpl->m_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  m_pImpl->m_renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

  m_pImpl->m_renderWindow->AddRenderer(m_pImpl->m_renderer);
  m_pImpl->m_renderWindowInteractor->SetRenderWindow(m_pImpl->m_renderWindow);

  m_eventProcessingThread = std::thread(&KinverseVisualizer::processEvents, this);
}

kinverse::visualization::KinverseVisualizer::KinverseVisualizer(void* renderWindow, void* renderer) {
  m_pImpl = std::make_shared<KinverseVisualizerImpl>();

  if (!renderWindow)
    throw std::invalid_argument("renderWindow is nullptr! renderWindow must be a valid object of type 'vtkSmartPointer<vtkRenderWindow>' cast to 'void*'!");

  if (!renderer)
    throw std::invalid_argument("renderer is nullptr! renderer must be a valid object of type 'vtkSmartPointer<vtkRenderer>' cast to 'void*'!");

  m_pImpl->m_renderer = *static_cast<vtkSmartPointer<vtkRenderer>*>(renderer);
  m_pImpl->m_renderWindow = *static_cast<vtkSmartPointer<vtkRenderWindow>*>(renderWindow);
}

kinverse::visualization::KinverseVisualizer::~KinverseVisualizer() {
  if (m_eventProcessingThread.joinable())
    m_eventProcessingThread.join();
}

void kinverse::visualization::KinverseVisualizer::addGizmo(IGizmo::Ptr gizmo) {
  if (gizmoIsAdded(gizmo))
    return;

  if (!m_pImpl->m_renderer)
    throw std::invalid_argument("m_pImpl->m_renderer is nullptr! It is an internal error, there is nothing you can do about it!");

  m_gizmos.push_back(gizmo);

  gizmo->show(reinterpret_cast<void*>(&m_pImpl->m_renderer));
}

void kinverse::visualization::KinverseVisualizer::removeGizmo(IGizmo::Ptr gizmo) {
  if (!gizmoIsAdded(gizmo))
    return;

  if (!m_pImpl->m_renderer)
    throw std::invalid_argument("m_pImpl->m_renderer is nullptr! It is an internal error, there is nothing you can do about it!");

  const auto iter = std::find(m_gizmos.begin(), m_gizmos.end(), gizmo);
  if (iter != m_gizmos.end())
    m_gizmos.erase(iter);

  gizmo->hide(reinterpret_cast<void*>(&m_pImpl->m_renderer));
}

void kinverse::visualization::KinverseVisualizer::processEvents() const {
  if (!m_pImpl->m_renderWindow)
    throw std::invalid_argument("m_pImpl->m_renderWindow is nullptr! It is an internal error, there is nothing you can do about it!");

  if (!m_pImpl->m_renderWindowInteractor) {
    throw std::invalid_argument("m_pImpl->m_renderWindowInteractor is nullptr! It is an internal error, there is nothing you can do about it!");
  }

  m_pImpl->m_renderWindow->Render();
  m_pImpl->m_renderWindow->SetWindowName("kinverse visualizer");
  m_pImpl->m_renderWindowInteractor->Start();
}

bool kinverse::visualization::KinverseVisualizer::gizmoIsAdded(IGizmo::Ptr gizmo) const {
  const auto iter = std::find(m_gizmos.begin(), m_gizmos.end(), gizmo);
  const bool isAdded = (iter != m_gizmos.end());
  return isAdded;
}
