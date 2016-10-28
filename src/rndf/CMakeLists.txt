include (${project_cmake_dir}/Utils.cmake)

set (rndf_sources
  rndf/Lane.cc
  rndf/ParkingSpot.cc
  rndf/Perimeter.cc
  rndf/RNDF.cc
  rndf/Segment.cc
  rndf/Waypoint.cc
  rndf/Zone.cc
  PARENT_SCOPE
)

set (gtest_sources
  Lane_TEST.cc
  ParkingSpot_TEST.cc
  Perimeter_TEST.cc
  RNDF_TEST.cc
  Segment_TEST.cc
  Waypoint_TEST.cc
  Zone_TEST.cc
)


ign_build_tests(${gtest_sources})