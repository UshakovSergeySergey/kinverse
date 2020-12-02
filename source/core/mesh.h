#pragma once

#include "exports.h"

namespace kinverse {
  namespace core {

    class KINVERSE_CORE_API Mesh {
     public:
      using Ptr = std::shared_ptr<Mesh>;
      using ConstPtr = std::shared_ptr<const Mesh>;

      unsigned int getNumberOfVertices() const;
      unsigned int getNumberOfFaces() const;

      void setVertices(const std::vector<Eigen::Vector3d>& vertices);
      std::vector<Eigen::Vector3d> getVertices() const;

      void setFaces(const std::vector<std::vector<unsigned int>>& faces);
      std::vector<std::vector<unsigned int>> getFaces() const;

     private:
      std::vector<Eigen::Vector3d> m_vertices{};
      std::vector<std::vector<unsigned int>> m_faces{};
    };

  }  // namespace core
}  // namespace kinverse
