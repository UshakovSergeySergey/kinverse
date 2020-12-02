#pragma once

#include "exports.h"
#include <core/mesh.h>

namespace kinverse {
  namespace io {

    class KINVERSE_IO_API MeshReader {
     public:
      using Ptr = std::shared_ptr<MeshReader>;
      using ConstPtr = std::shared_ptr<const MeshReader>;

      core::Mesh::Ptr read(const std::string& filename) const;

      bool hasSupportedExtension(const std::string& filename) const;
      std::vector<std::string> getListOfSupportedExtensions() const;

     private:
      core::Mesh::Ptr readFile(const std::string& filename) const;
    };

  }  // namespace io
}  // namespace kinverse
