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

#pragma once

#include "exports.h"

namespace kinverse {
  namespace core {

    /**
     * @class Mesh
     * @brief This class represents polygon mesh.
     */
    class KINVERSE_CORE_API Mesh {
     public:
      /**
       * @brief Short alias for vertex buffer array
       */
      using Vertices = std::vector<Eigen::Vector3d>;

      /**
       * @brief Short alias for index buffer array
       */
      using Faces = std::vector<std::vector<unsigned long long>>;

      /**
       * @brief Smart pointer to @p Mesh
       */
      using Ptr = std::shared_ptr<Mesh>;

      /**
       * @brief Smart pointer to const @p Mesh
       */
      using ConstPtr = std::shared_ptr<const Mesh>;

      /**
       * @brief Simple constructor for mesh creation.
       * @param[in] vertices - vertex buffer
       * @param[in] faces - index buffer
       */
      explicit Mesh(const Vertices& vertices = {}, const Faces& faces = {});

      /**
       * @brief Returns number of vertices.
       */
      unsigned long long getNumberOfVertices() const;

      /**
       * @brief Returns number of faces.
       */
      unsigned long long getNumberOfFaces() const;

      /**
       * @brief Sets vertices.
       * @param[in] vertices - mesh vertices
       */
      void setVertices(const Vertices& vertices);

      /**
       * @brief Returns mesh vertices.
       */
      Vertices getVertices() const;

      /**
       * @brief Sets mesh faces. Note that face is not necessarily a triangle. It can have as many vertices as you want.
       * @param[in] faces - mesh faces
       */
      void setFaces(const Faces& faces);

      /**
       * @brief Returns mesh faces.
       */
      Faces getFaces() const;

      /**
       * @brief Equality operator for Mesh objects.
       * @param[in] rhs - right side object
       * @return Returns true if objects are equal, false otherwise.
       */
      bool operator==(const Mesh& rhs) const;

      /**
       * @brief Inequality operator for Mesh objects.
       * @param[in] rhs - right side object
       * @return Returns true if objects are not equal, false otherwise.
       */
      bool operator!=(const Mesh& rhs) const;

     private:
      /**
       * @brief Stores mesh vertices.
       */
      Vertices m_vertices{};

      /**
       * @brief Stores mesh faces. Note that face is not necessarily a triangle. It can have as many vertices as you want.
       */
      Faces m_faces{};
    };

  }  // namespace core
}  // namespace kinverse
