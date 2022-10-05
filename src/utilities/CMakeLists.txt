##############################################################################
# osm
##############################################################################
add_library(utilities
  utilities.cc
)
add_library(maliput_osm::utilities ALIAS utilities)
set_target_properties(utilities
  PROPERTIES
    OUTPUT_NAME maliput_osm_utilities
)

target_include_directories(
  utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(utilities
   PRIVATE
    maliput::common
)

install(TARGETS utilities
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
