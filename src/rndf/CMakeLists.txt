include (${project_cmake_dir}/Utils.cmake)

set (sources
  Lane.cc
  ParkingSpot.cc
  Perimeter.cc
  RNDF.cc
  Segment.cc
  Waypoint.cc
  Zone.cc
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