#pragma once

#include <core/mesh.h>

namespace kinverse {
  namespace visualization {

    class MeshToVtkPolyDataConverter {
     public:
      static vtkSmartPointer<vtkPolyData> convert(core::Mesh::ConstPtr mesh);

     private:
      static vtkSmartPointer<vtkPoints> convertVertexBuffer(core::Mesh::ConstPtr mesh);
      static vtkSmartPointer<vtkCellArray> convertIndexBuffer(core::Mesh::ConstPtr mesh);
    };

  }  // namespace visualization
}  // namespace kinverse
