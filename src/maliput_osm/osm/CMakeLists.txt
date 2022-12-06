##############################################################################
# osm
##############################################################################
add_library(osm
  conversions.cc
  osm_manager.cc
)
add_library(maliput_osm::osm ALIAS osm)
set_target_properties(osm
  PROPERTIES
    OUTPUT_NAME maliput_osm_osm
)

target_include_directories(
  osm
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(osm
   PRIVATE
    lanelet2_core::lanelet2_core
    lanelet2_io::lanelet2_io
    maliput::api
    maliput::common
    maliput_sparse::geometry
    maliput_sparse::parser
)

install(TARGETS osm
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
