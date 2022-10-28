find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

macro(add_dependencies_to_test target)
if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${PROJECT_SOURCE_DIR}/src
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
        lanelet2_io::lanelet2_io
        maliput::api
        maliput::common
        maliput::plugin
        maliput::test_utilities
        maliput_osm::builder
        maliput_osm::osm
        maliput_osm::osm_utilities
        maliput_osm::utilities
        maliput_osm::test_utilities
        maliput_sparse::builder
        maliput_sparse::geometry
      )

    endif()
endmacro()

add_subdirectory(builder)
add_subdirectory(osm)
add_subdirectory(plugin)
