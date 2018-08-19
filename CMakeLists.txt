CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
PROJECT(wave_geometry)

# Compiler settings for all targets
SET(CMAKE_CXX_STANDARD 14)

# Package version, used when other projects FIND_PACKAGE(wave <version>)
SET(WAVE_GEOMETRY_PACKAGE_VERSION 0.2.0)

INCLUDE(cmake/WaveGeometryHelpers.cmake)

IF(TARGET wave)
  # We are being included from inside the `wave` project
  # Assume some things, such as test and benchmark targets, are already made
ELSE(TARGET wave)
  # We are being built standalone
  # CMake modules
  LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
  INCLUDE(CMakePackageConfigHelpers)
  INCLUDE(GNUInstallDirs)
  INCLUDE(cmake/AddEigen3.cmake)
  ENABLE_TESTING()

  # User options
  OPTION(BUILD_TESTING "Build tests" ON)
  OPTION(EXPORT_BUILD
      "Add this build directory to CMake's user package registry.\
 Allows the package to be found without install." OFF)
  OPTION(BUILD_BENCHMARKS
      "Build benchmarks for some components. Requires google benchmark package."
      OFF)

  # Optionally build tests. `gtest` is included with this project
  IF(BUILD_TESTING)
    # Build gtest from source
    INCLUDE(cmake/AddGTest.cmake)

    # This target is used to build all tests, without running them
    ADD_CUSTOM_TARGET(tests)
    MESSAGE(STATUS "Building tests")
  ENDIF(BUILD_TESTING)


  IF(BUILD_BENCHMARKS)
    FIND_PACKAGE(benchmark REQUIRED)

    STRING(TOLOWER "${CMAKE_BUILD_TYPE}" build_type)
    IF(NOT build_type MATCHES rel)
        MESSAGE(WARNING "Building benchmarks but not in Release mode."
          " (CMAKE_BUILD_TYPE=\"${CMAKE_BUILD_TYPE}\")\n"
          "Benchmark results might not be useful.")
    ENDIF()

    # This target is used to build (but not run) benchmarks
    ADD_CUSTOM_TARGET(benchmarks)

    # This target is used to run benchmarks via "make benchmark".
    # Like "make test", it does not build anything. (@todo change?)
    # It runs all tests labelled "benchmark" by WAVE_GEOMETRY_ADD_BENCHMARK helper.
    ADD_CUSTOM_TARGET(benchmark
        COMMAND ${CMAKE_CTEST_COMMAND} -C benchmark -L benchmark)
    MESSAGE(STATUS "Building benchmarks")
  ENDIF(BUILD_BENCHMARKS)

  # We use header-only parts of boost: boost::optional
  FIND_PACKAGE(Boost REQUIRED)
ENDIF(TARGET wave)

IF(BUILD_TESTING)
  ADD_SUBDIRECTORY(test)
ENDIF(BUILD_TESTING)

IF(BUILD_BENCHMARKS)
  ADD_SUBDIRECTORY(benchmarks)
ENDIF(BUILD_BENCHMARKS)

IF(TARGET wave)
  WAVE_ADD_MODULE(wave_geometry DEPENDS Eigen3::Eigen Boost::boost)
ELSE(TARGET wave)
  # Make a target for wave_geometry
  ADD_LIBRARY(wave_geometry INTERFACE)
  TARGET_COMPILE_OPTIONS(wave_geometry INTERFACE -Wall -Wextra)
  TARGET_LINK_LIBRARIES(wave_geometry INTERFACE Eigen3::Eigen ${BOOST_LIBRARIES})

  # Set the public include paths so they are usable from both the build and
  # install tree. See:
  # https://cmake.org/cmake/help/v3.4/manual/cmake-buildsystem.7.html#include-directories-and-usage-requirements
  # https://stackoverflow.com/a/25681179
  TARGET_INCLUDE_DIRECTORIES(wave_geometry INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  # Install to include path set by GNUInstallDirs
  INSTALL(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  INCLUDE(cmake/InstallConfigFiles.cmake)
ENDIF(TARGET wave)

# We ship the headers of Tick and always want to use this exact version
# Make it an imported target.
ADD_LIBRARY(Tick::Tick INTERFACE IMPORTED)
SET_PROPERTY(TARGET Tick::Tick PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/3rd-party/Tick")
# Add Tick header to the include path as well.
TARGET_INCLUDE_DIRECTORIES(wave_geometry INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/Tick>
    $<INSTALL_INTERFACE:3rd-party/Tick>)
INSTALL(DIRECTORY 3rd-party DESTINATION .)
