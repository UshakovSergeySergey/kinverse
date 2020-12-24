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
#include "../include/kinverse/core/mesh.h"
#include <kinverse/math/math.h>

kinverse::core::Mesh::Mesh(const Vertices& vertices, const Faces& faces) {
  setVertices(vertices);
  setFaces(faces);
}

unsigned long long kinverse::core::Mesh::getNumberOfVertices() const {
  return m_vertices.size();
}

unsigned long long kinverse::core::Mesh::getNumberOfFaces() const {
  return m_faces.size();
}

void kinverse::core::Mesh::setVertices(const Vertices& vertices) {
  m_vertices = vertices;
}

kinverse::core::Mesh::Vertices kinverse::core::Mesh::getVertices() const {
  return m_vertices;
}

void kinverse::core::Mesh::setFaces(const Faces& faces) {
  m_faces = faces;
}

kinverse::core::Mesh::Faces kinverse::core::Mesh::getFaces() const {
  return m_faces;
}

bool kinverse::core::Mesh::operator==(const Mesh& rhs) const {
  if (this->m_faces != rhs.m_faces)
    return false;

  if (this->m_vertices.size() != rhs.m_vertices.size())
    return false;

  const auto numberOfVertices = this->m_vertices.size();
  for (auto vertexCounter = 0u; vertexCounter < numberOfVertices; ++vertexCounter) {
    const Eigen::Vector3d& p1 = this->m_vertices[vertexCounter];
    const Eigen::Vector3d& p2 = rhs.m_vertices[vertexCounter];
    if (!math::pointsAreEqual(p1, p2))
      return false;
  }

  return true;
}

bool kinverse::core::Mesh::operator!=(const Mesh& rhs) const {
  return !(*this == rhs);
}
