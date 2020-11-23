#pragma once

namespace kinverse {
  namespace visualization {

    class KinverseVisualizerImpl {
     public:
      vtkSmartPointer<vtkRenderer> m_renderer{ nullptr };
      vtkSmartPointer<vtkRenderWindow> m_renderWindow{ nullptr };
      vtkSmartPointer<vtkRenderWindowInteractor> m_renderWindowInteractor{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
