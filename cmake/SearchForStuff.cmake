include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)
include (FindPkgConfig)
include (${project_cmake_dir}/FindOS.cmake)

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

########################################
# Find ignition math in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  find_package(ignition-math2 2.4 QUIET)
  if (NOT ignition-math2_FOUND)
    message(STATUS "Looking for ignition-math2-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition math2 library (libignition-math2-dev).")
  else()
    message(STATUS "Looking for ignition-math2-config.cmake - found")
    include_directories(${IGNITION-MATH_INCLUDE_DIRS})
    link_directories(${IGNITION-MATH_LIBRARY_DIRS})
  endif()
endif()

########################################
# Find Xerces-C++ in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  pkg_check_modules(XERCESC xerces-c)
  if (NOT XERCESC_FOUND)
    message (STATUS "Looking for Xerces-C++ - not found")
    BUILD_ERROR("Missing: libxerces-c-dev")
  else()
    MESSAGE (STATUS "Xerces include: ${XERCESC_INCLUDEDIR}")
    MESSAGE (STATUS "Xerces lib dir: ${XERCESC_LIBDIR}")
    MESSAGE (STATUS "Xerces lib: ${XERCESC_LIBRARIES}")
    include_directories(${XERCESC_INCLUDEDIR})
    link_directories(${XERCESC_LIBDIR})
  endif()
endif()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()
