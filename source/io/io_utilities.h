#pragma once

namespace kinverse {
  namespace io {

    class IOUtilities {
     public:
      static std::string getExtension(const std::string& filename);
      static bool fileExists(const std::string& filename);
      static std::string toLowerCase(const std::string& str);
    };

  }  // namespace io
}  // namespace kinverse
