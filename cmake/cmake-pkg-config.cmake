include(GNUInstallDirs)

set(KD_TREE_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/KDTree")

include(${KD_TREE_SOURCE_DIR}/cmake/package-dependencies.cmake)

set(KD_TREE_FIND_DEPENDENCY_BLOCK "")
foreach(_kd_tree_dep IN LISTS KD_TREE_PACKAGE_DEPENDENCIES)
  string(APPEND KD_TREE_FIND_DEPENDENCY_BLOCK "find_dependency(${_kd_tree_dep} REQUIRED)\n")
endforeach()
unset(_kd_tree_dep)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR}/KDTree
    CACHE PATH "Location of header files" )
set(SYSCONFIG_INSTALL_DIR ${CMAKE_INSTALL_SYSCONFDIR}/KDTree
    CACHE PATH "Location of configuration files" )

include(CMakePackageConfigHelpers)
configure_package_config_file(${KD_TREE_SOURCE_DIR}/KDTreeConfig.cmake.in ${KD_TREE_BINARY_DIR}/KDTreeConfig.cmake
  INSTALL_DESTINATION ${KD_TREE_INSTALL_CMAKEDIR}
  PATH_VARS INCLUDE_INSTALL_DIR SYSCONFIG_INSTALL_DIR)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/KDTreeConfigVersion.cmake
  VERSION ${KD_TREE_VERSION}
  COMPATIBILITY SameMajorVersion )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/KDTreeConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/KDTreeConfigVersion.cmake
        DESTINATION ${KD_TREE_INSTALL_CMAKEDIR})