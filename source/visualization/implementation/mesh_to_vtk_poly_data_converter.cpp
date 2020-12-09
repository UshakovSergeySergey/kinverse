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
#include "mesh_to_vtk_poly_data_converter.h"

vtkSmartPointer<vtkPolyData> kinverse::visualization::MeshToVtkPolyDataConverter::convert(core::Mesh::ConstPtr mesh) {
  const vtkSmartPointer<vtkPoints> vertexBuffer = convertVertexBuffer(mesh);
  const vtkSmartPointer<vtkCellArray> indexBuffer = convertIndexBuffer(mesh);

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
