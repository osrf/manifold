include (${project_cmake_dir}/Utils.cmake)

set (schemas
  OpenDRIVE_1.4H.xsd
)

install(FILES ${schemas} DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/manifold/)
