cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find the Manifold library
find_package(manifold0 QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MANIFOLD_CXX_FLAGS}")
include_directories(${MANIFOLD_INCLUDE_DIRS})
link_directories(${MANIFOLD_LIBRARY_DIRS})

# Generate examples

if (MSVC)

  # Don't pull in the Windows min/max macros
  add_definitions(-DNOMINMAX)

endif()
