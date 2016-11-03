include (${project_cmake_dir}/Utils.cmake)

set (common_headers
  Graph.hh
  Helpers.hh
  RoadNetwork.hh
)

set (rndf_headers
  rndf/Checkpoint.hh
  rndf/Lane.hh
  rndf/ParkingSpot.hh
  rndf/Perimeter.hh
  rndf/RNDF.hh
  rndf/Segment.hh
  rndf/Waypoint.hh
  rndf/Zone.hh
)

#################################################
### Do not change the following
set (ign_headers "" CACHE INTERNAL "Manifold headers" FORCE)
foreach (hdr ${common_headers} ${rndf_headers})
  APPEND_TO_CACHED_STRING(ign_headers
    "Manifold headers" "#include <${PROJECT_NAME_LOWER}/${hdr}>\n")
endforeach()

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/manifold_auto_headers.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME_LOWER}.hh)

ign_install_includes("${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME_LOWER}"
                     ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME_LOWER}.hh)
ign_install_includes("${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME_LOWER}"
                     ${common_headers})
ign_install_includes("${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME_LOWER}/rndf"
                     ${rndf_headers})
