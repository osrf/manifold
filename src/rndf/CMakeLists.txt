include (${project_cmake_dir}/Utils.cmake)

set (rndf_sources
  rndf/Checkpoint.cc
  rndf/Exit.cc
  rndf/Lane.cc
  rndf/ParkingSpot.cc
  rndf/ParserUtils.cc
  rndf/Perimeter.cc
  rndf/RNDF.cc
  rndf/Segment.cc
  rndf/UniqueId.cc
  rndf/Waypoint.cc
  rndf/Zone.cc
  PARENT_SCOPE
)

set (gtest_sources
  Checkpoint_TEST.cc
  Exit_TEST.cc
  Lane_TEST.cc
  ParkingSpot_TEST.cc
  ParserUtils_TEST.cc
  Perimeter_TEST.cc
  RNDF_TEST.cc
  Segment_TEST.cc
  UniqueId_TEST.cc
  Waypoint_TEST.cc
  Zone_TEST.cc
)

ign_build_tests(${gtest_sources})
