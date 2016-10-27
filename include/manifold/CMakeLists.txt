include (${project_cmake_dir}/Utils.cmake)

set (headers
  Helpers.hh
)

#################################################
### Do not change the following

set (ign_headers "" CACHE INTERNAL "Manifold headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(ign_headers
    "Manifold headers" "#include <${MANIFOLD_PROJECT_NAME}/${hdr}>\n")
endforeach()

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/manifold_auto_headers.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME_LOWER}.hh)

ign_install_includes("${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME_LOWER}"
                     ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME_LOWER}.hh)
ign_install_includes("${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME_LOWER}"
                     ${headers})
