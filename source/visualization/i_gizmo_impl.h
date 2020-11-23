#pragma once

namespace kinverse {
  namespace visualization {

    class IGizmoImpl {
     public:
      vtkSmartPointer<vtkProp> m_actor{ nullptr };
      vtkSmartPointer<vtkRenderer> m_renderer{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
