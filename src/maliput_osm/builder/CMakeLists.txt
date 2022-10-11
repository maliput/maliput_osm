##############################################################################
# osm
##############################################################################
add_library(builder
  builder_configuration.cc
  road_geometry_builder.cc
  road_network_builder.cc
)

add_library(maliput_osm::builder ALIAS builder)
set_target_properties(builder
  PROPERTIES
    OUTPUT_NAME maliput_osm_builder
)

target_include_directories(
  builder
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(builder
  PUBLIC
    maliput::api
    maliput::base
    maliput::geometry_base
  PRIVATE
    maliput_sparse::builder
    maliput_sparse::geometry
    maliput_osm::osm
)

install(TARGETS builder
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
