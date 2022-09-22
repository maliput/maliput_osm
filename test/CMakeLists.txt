find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

macro(add_dependencies_to_test target)
if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
          maliput_osm::osm
      )

    endif()
endmacro()

add_subdirectory(osm)
