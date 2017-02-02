include (${project_cmake_dir}/Utils.cmake)

add_subdirectory(opendrive)
add_subdirectory(rndf)

set (sources
  ${rndf_sources}
  Helpers.cc
  RoadNetwork.cc
)

set (gtest_sources
  Helpers_TEST.cc
  RoadNetwork_TEST.cc
)

MESSAGE(STATUS "Files: ${sources}")

include_directories(${CMAKE_BINARY_DIR}/ ${CMAKE_BINARY_DIR}/test/)

ign_build_tests(${gtest_sources})

if (UNIX)
  ign_add_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION} ${sources})
elseif(WIN32)
  add_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION} STATIC ${sources})
endif()

target_link_libraries(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}
  ${IGNITION-MATH_LIBRARIES})

if(WIN32)
  target_link_libraries(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}
    ws2_32 Iphlpapi
  )
else()
  target_link_libraries(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}
  )
endif()

ign_install_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION})
