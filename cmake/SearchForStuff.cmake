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
# Find tinyxml2 in unix platforms
# In Windows we expect a call from configure.bat script with the paths
if (NOT WIN32)
  pkg_check_modules(tinyxml2 tinyxml2)
  if (NOT tinyxml2_FOUND)
      find_path (tinyxml2_INCLUDE_DIRS tinyxml2.h ${tinyxml2_INCLUDE_DIRS} ENV CPATH)
      find_library(tinyxml2_LIBRARIES NAMES tinyxml2)
      set (tinyxml2_FAIL False)
      if (NOT tinyxml2_INCLUDE_DIRS)
        message (STATUS "Looking for tinyxml2 headers - not found")
        set (tinyxml2_FAIL True)
      endif()
      if (NOT tinyxml2_LIBRARIES)
        message (STATUS "Looking for tinyxml2 library - not found")
        set (tinyxml2_FAIL True)
      endif()
      if (NOT tinyxml2_LIBRARY_DIRS)
        message (STATUS "Looking for tinyxml2 library dirs - not found")
        set (tinyxml2_FAIL True)
      endif()
  endif()

  if (tinyxml2_FAIL)
    message (STATUS "Looking for tinyxml2.h - not found")
    BUILD_ERROR("Missing: libtinyxml2-dev")
  else()
    include_directories(${tinyxml2_INCLUDE_DIRS})
    link_directories(${tinyxml2_LIBRARY_DIRS})
  endif()
endif()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()
