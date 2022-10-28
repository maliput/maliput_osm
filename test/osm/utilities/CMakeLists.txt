##############################################################################
# osm
##############################################################################
add_library(osm_utilities
  lanelet2.cc
)
add_library(maliput_osm::osm_utilities ALIAS osm_utilities)
set_target_properties(osm_utilities
  PROPERTIES
    OUTPUT_NAME maliput_osm_osm_utilities
)

target_include_directories(
  osm_utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test>
)

target_link_libraries(osm_utilities
   PRIVATE
   lanelet2_core::lanelet2_core

)
