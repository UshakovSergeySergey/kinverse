#include "stdafx.h"
#include "mesh_to_vtk_poly_data_converter.h"

vtkSmartPointer<vtkPolyData> kinverse::visualization::MeshToVtkPolyDataConverter::convert(core::Mesh::ConstPtr mesh) {
  vtkSmartPointer<vtkPoints> vertexBuffer = convertVertexBuffer(mesh);
  vtkSmartPointer<vtkCellArray> indexBuffer = convertIndexBuffer(mesh);

  vtkSmartPointer<vtkPolyData> vtkMesh = vtkSmartPointer<vtkPolyData>::New();
  vtkMesh->SetPoints(vertexBuffer);
  vtkMesh->SetPolys(indexBuffer);

  return vtkMesh;
}

vtkSmartPointer<vtkPoints> kinverse::visualization::MeshToVtkPolyDataConverter::convertVertexBuffer(core::Mesh::ConstPtr mesh) {
  vtkSmartPointer<vtkPoints> vertexBuffer = vtkSmartPointer<vtkPoints>::New();

  if (!mesh)
    return vertexBuffer;

  for (const auto& vertex : mesh->getVertices()) {
    vertexBuffer->InsertNextPoint(vertex.data());
  }

  return vertexBuffer;
}

vtkSmartPointer<vtkCellArray> kinverse::visualization::MeshToVtkPolyDataConverter::convertIndexBuffer(core::Mesh::ConstPtr mesh) {
  vtkSmartPointer<vtkCellArray> indexBuffer = vtkSmartPointer<vtkCellArray>::New();

  if (!mesh)
    return indexBuffer;

  for (const auto& face : mesh->getFaces()) {
    const auto numberOfVerticesInFace = face.size();

    vtkIdType* faceIndices = new vtkIdType[numberOfVerticesInFace];
    for (auto indexCounter = 0u; indexCounter < face.size(); ++indexCounter)
      faceIndices[indexCounter] = face[indexCounter];

    indexBuffer->InsertNextCell(numberOfVerticesInFace, faceIndices);

    delete[] faceIndices;
  }

  return indexBuffer;
}
