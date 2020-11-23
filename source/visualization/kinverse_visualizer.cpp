#include "stdafx.h"
#include "kinverse_visualizer.h"
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

  m_pImpl->m_renderer = *reinterpret_cast<vtkSmartPointer<vtkRenderer>*>(renderer);
  m_pImpl->m_renderWindow = *reinterpret_cast<vtkSmartPointer<vtkRenderWindow>*>(renderWindow);
}

kinverse::visualization::KinverseVisualizer::~KinverseVisualizer() {
  if (m_eventProcessingThread.joinable())
    m_eventProcessingThread.join();
}

void kinverse::visualization::KinverseVisualizer::addGizmo(IGizmo::Ptr gizmo) {
  if (!m_pImpl->m_renderer)
    throw std::invalid_argument("m_pImpl->m_renderer is nullptr! It is an internal error, there is nothing you can do about it!");

  m_gizmos.push_back(gizmo);
  gizmo->draw(reinterpret_cast<void*>(&m_pImpl->m_renderer));
}

void kinverse::visualization::KinverseVisualizer::processEvents() {
  if (!m_pImpl->m_renderWindow)
    throw std::invalid_argument("m_pImpl->m_renderWindow is nullptr! It is an internal error, there is nothing you can do about it!");

  if (!m_pImpl->m_renderWindowInteractor) {
    throw std::invalid_argument("m_pImpl->m_renderWindowInteractor is nullptr! It is an internal error, there is nothing you can do about it!");
  }

  m_pImpl->m_renderWindow->Render();
  m_pImpl->m_renderWindow->SetWindowName("kinverse visualizer");
  m_pImpl->m_renderWindowInteractor->Start();
}
