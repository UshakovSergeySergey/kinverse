#include "stdafx.h"
#include "io_utilities.h"

std::string kinverse::io::IOUtilities::getExtension(const std::string& filename) {
  const auto extension = std::filesystem::path(filename).extension().string();
  return extension;
}

bool kinverse::io::IOUtilities::fileExists(const std::string& filename) {
  return std::filesystem::exists(filename);
}

std::string kinverse::io::IOUtilities::toLowerCase(const std::string& str) {
  auto lowerCaseStr = str;
  std::transform(str.begin(), str.end(), lowerCaseStr.begin(), [](unsigned char c) { return std::tolower(c); });
  return lowerCaseStr;
}
