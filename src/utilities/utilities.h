#pragma once

#include <string>

#include <maliput/common/filesystem.h>
#include <maliput/common/maliput_throw.h>

namespace utilities {

constexpr char MALIPUT_OSM_RESOURCE_ROOT[] = "MALIPUT_OSM_RESOURCE_ROOT";

/// TODO: Use ament_index to find the package's resources.
/// @returns @p file_name 's path located at MALIPUT_OSM_RESOURCE_ROOT env path.
/// If not located, an empty string is returned.
/// @throws maliput::common::assertion_error When @p file_name is an absolute path.
std::string FindOSMResource(const std::string& file_name) {
  MALIPUT_THROW_UNLESS(!maliput::common::Path{file_name}.is_absolute());
  maliput::common::Path file_path =
      maliput::common::Filesystem::get_env_path(MALIPUT_OSM_RESOURCE_ROOT) + "/resources/osm";
  file_path.append(file_name);
  // Returns empty string if file_path doesn't match any existing file.
  return file_path.exists() ? file_path.get_path() : "";
}

}  // namespace utilities
