#include "stdafx.h"
#include "../include/kinverse/core/mesh.h"

unsigned int kinverse::core::Mesh::getNumberOfVertices() const {
  return m_vertices.size();
}

unsigned int kinverse::core::Mesh::getNumberOfFaces() const {
  return m_faces.size();
}

void kinverse::core::Mesh::setVertices(const std::vector<Eigen::Vector3d>& vertices) {
  m_vertices = vertices;
}

std::vector<Eigen::Vector3d> kinverse::core::Mesh::getVertices() const {
  return m_vertices;
}

void kinverse::core::Mesh::setFaces(const std::vector<std::vector<unsigned int>>& faces) {
  m_faces = faces;
}

std::vector<std::vector<unsigned int>> kinverse::core::Mesh::getFaces() const {
  return m_faces;
}
