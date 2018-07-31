# - Generate and install CMake config files so this package can be used in
# other projects

# This is where .cmake files will be installed (default: under share/)
SET(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_DATADIR}/wave_geometry/cmake" CACHE PATH
  "Installation directory for CMake files")

# Generate the Config file for the install tree
SET(WAVE_GEOMETRY_EXTRA_CMAKE_DIR "cmake")
CONFIGURE_PACKAGE_CONFIG_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/wave_geometryConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/wave_geometryConfig.cmake"
  INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}")

# Generate the Version file
WRITE_BASIC_PACKAGE_VERSION_FILE(wave_geometryConfigVersion.cmake
  VERSION ${WAVE_GEOMETRY_PACKAGE_VERSION}
  COMPATIBILITY SameMajorVersion)

# Install the Config and ConfigVersion files
INSTALL(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/wave_geometryConfig.cmake"
  "${PROJECT_BINARY_DIR}/wave_geometryConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}")
# Install auto-generated targets file used by the Config file
INSTALL(TARGETS wave_geometry EXPORT wave_geometryTargets)
INSTALL(EXPORT wave_geometryTargets DESTINATION "${INSTALL_CMAKE_DIR}")
# Install our import scripts so we can use them in the Config script. That way,
# other projects using wave_geometry will get transient dependencies (such as
# Eigen) automatically
INSTALL(DIRECTORY "${WAVE_GEOMETRY_EXTRA_CMAKE_DIR}/"
  DESTINATION "${INSTALL_CMAKE_DIR}/${WAVE_GEOMETRY_EXTRA_CMAKE_DIR}"
  FILES_MATCHING PATTERN "Add*.cmake" PATTERN "Find*.cmake")

IF(EXPORT_BUILD)
  # Export this build so the package can be found through CMake's registry
  # without being installed.
  EXPORT(PACKAGE wave_geometry)

  # Create a Targets file in the build directory
  EXPORT(EXPORT wave_geometryTargets FILE wave_geometryTargets.cmake)

  # Generate the Config file for the build tree
  # It differs from the installed Config file in where our bundled
  # Import*.cmake scripts are included from. Here, use the relative path from
  # the build directory to this source directory
  FILE(RELATIVE_PATH path_to_wave_geometry_source
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR})
  SET(WAVE_GEOMETRY_EXTRA_CMAKE_DIR "${path_to_wave_geometry_source}cmake")
  CONFIGURE_PACKAGE_CONFIG_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/wave_geometryConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/wave_geometryConfig.cmake"
    INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}")

  message(STATUS "Exporting to the CMake user package registry. The command "
          "FIND_PACKAGE(wave_geometry) can now find this directory, without "
          "installing.")
ENDIF(EXPORT_BUILD)