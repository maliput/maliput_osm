find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

macro(add_dependencies_to_test target)
if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/include
          ${PROJECT_SOURCE_DIR}/src
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
        maliput::api
        maliput::common
        maliput_sparse::geometry
      )

    endif()
endmacro()

add_subdirectory(osm)
