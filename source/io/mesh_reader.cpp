#include "stdafx.h"
#include "mesh_reader.h"
#include "io_utilities.h"

kinverse::core::Mesh::Ptr convertMesh(vtkSmartPointer<vtkPolyData> vtkMesh) {
  const auto points = vtkMesh->GetPoints();
  auto polygons = vtkMesh->GetPolys();

  std::vector<Eigen::Vector3d> vertices{};
  for (auto pointCounter = 0u; pointCounter < points->GetNumberOfPoints(); ++pointCounter) {
    Eigen::Vector3d point;
    points->GetPoint(pointCounter, point.data());
    vertices.push_back(point);
  }

  std::vector<std::vector<unsigned int>> faces{};
  polygons->InitTraversal();
  for (auto faceCounter = 0u; faceCounter < polygons->GetNumberOfCells(); ++faceCounter) {
    vtkIdType numberOfVerticesInFace = 0;
    vtkIdType* faceIndices = nullptr;
    polygons->GetNextCell(numberOfVerticesInFace, faceIndices);

    std::vector<unsigned int> face{};
    for (auto vertexCounter = 0u; vertexCounter < numberOfVerticesInFace; ++vertexCounter) {
      face.push_back(faceIndices[vertexCounter]);
    }
    faces.push_back(face);
  }

  auto mesh = std::make_shared<kinverse::core::Mesh>();
  mesh->setVertices(vertices);
  mesh->setFaces(faces);

  return mesh;
}

kinverse::core::Mesh::Ptr kinverse::io::MeshReader::read(const std::string& filename) const {
  const auto lowerCaseFilename = IOUtilities::toLowerCase(filename);
  if (!hasSupportedExtension(lowerCaseFilename))
    throw std::invalid_argument("Failed to read mesh '" + filename + "'! Extension is not supported!");

  if (!IOUtilities::fileExists(lowerCaseFilename))
    throw std::invalid_argument("Failed to read mesh '" + filename + "'! File doesn't exist!");

  return readFile(lowerCaseFilename);
}

bool kinverse::io::MeshReader::hasSupportedExtension(const std::string& filename) const {
  const auto supportedExtensions = getListOfSupportedExtensions();

  const auto fileExtension = IOUtilities::getExtension(filename);

  for (const auto& supportedExtension : supportedExtensions) {
    if (fileExtension == supportedExtension)
      return true;
  }

  return false;
}

std::vector<std::string> kinverse::io::MeshReader::getListOfSupportedExtensions() const {
  return { ".ply", ".obj", ".stl" };
}

kinverse::core::Mesh::Ptr kinverse::io::MeshReader::readFile(const std::string& filename) const {
  const auto fileExtension = IOUtilities::getExtension(filename);

  vtkSmartPointer<vtkAbstractPolyDataReader> reader = nullptr;
  if (fileExtension == ".ply")
    reader = vtkSmartPointer<vtkPLYReader>::New();
  else if (fileExtension == ".obj")
    reader = vtkSmartPointer<vtkOBJReader>::New();
  else if (fileExtension == ".stl")
    reader = vtkSmartPointer<vtkSTLReader>::New();

  if (!reader)
    throw std::invalid_argument("Failed to read mesh '" + filename + "'! Extension is not supported!");

  reader->SetFileName(filename.c_str());
  reader->Update();

  vtkSmartPointer<vtkPolyData> vtkMesh = reader->GetOutput();

  core::Mesh::Ptr mesh = convertMesh(vtkMesh);

  return mesh;
}
