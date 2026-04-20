include(GNUInstallDirs)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR}/KDTree
    CACHE PATH "Location of header files" )
set(SYSCONFIG_INSTALL_DIR ${CMAKE_INSTALL_SYSCONFDIR}/KDTree
    CACHE PATH "Location of configuration files" )

include(CMakePackageConfigHelpers)
configure_package_config_file(${KD_TREE_SOURCE_DIR}/KDTreeConfig.cmake.in ${KD_TREE_BINARY_DIR}/KDTreeConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/KDTree
  PATH_VARS INCLUDE_INSTALL_DIR SYSCONFIG_INSTALL_DIR)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/KDTreeConfigVersion.cmake
  VERSION ${KD_TREE_VERSION}
  COMPATIBILITY SameMajorVersion )
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/KDTreeConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/KDTreeConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/KDTree)