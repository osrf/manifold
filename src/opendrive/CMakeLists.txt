include (${project_cmake_dir}/Utils.cmake)

set (opendrive_sources
  opendrive/Importer.cc
  PARENT_SCOPE
)

set (gtest_sources
  Importer_TEST.cc
)

ign_build_tests(${gtest_sources})