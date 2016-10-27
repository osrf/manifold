include (${project_cmake_dir}/Utils.cmake)

set (sources
  Helpers.cc
)

set (gtest_sources
  Helpers_TEST.cc
)

include_directories(${CMAKE_BINARY_DIR}/ ${CMAKE_BINARY_DIR}/test/)

ign_build_tests(${gtest_sources})

if (UNIX)
  ign_add_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION} ${sources})
elseif(WIN32)
  add_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION} STATIC ${sources})
endif()

# Protobuf module does not have LIBRARIES but LIBRARY
target_link_libraries(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}
                     )

if(WIN32)
  target_link_libraries(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}
    ws2_32 Iphlpapi
  )
else()
  target_link_libraries(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}
  )
endif()

ign_install_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION})