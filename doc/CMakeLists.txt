find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/doc/manifold.in
                 ${CMAKE_BINARY_DIR}/manifold.dox @ONLY)

  add_custom_target(doc

    # Generate the API documentation
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/manifold.dox
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/ignition_logo.svg
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/search.js
            ${CMAKE_BINARY_DIR}/doxygen/html/search
    # COMMAND make -C ${CMAKE_BINARY_DIR}/doxygen/latex
    # COMMAND mv ${CMAKE_BINARY_DIR}/doxygen/latex/refman.pdf
    # ${CMAKE_BINARY_DIR}/doxygen/latex/${PROJECT_NAME}-${PROJECT_VERSION_FULL}.pdf

    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()
