CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
PROJECT(wave_geometry)

# Compiler settings for all targets
SET(CMAKE_CXX_STANDARD 11)

# CMake modules
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
INCLUDE(CMakePackageConfigHelpers)
INCLUDE(cmake/WaveGeometryHelpers.cmake)
INCLUDE(cmake/AddEigen3.cmake)
ENABLE_TESTING()

# User options
OPTION(BUILD_TESTING "Build tests" ON)
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

  ADD_SUBDIRECTORY(test)
ENDIF(BUILD_TESTING)

# Optionally build benchmarks (requires `benchmark` package to be installed)
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
  # It runs all tests labelled "benchmark" by WAVE_ADD_BENCHMARK helper.
  ADD_CUSTOM_TARGET(benchmark
    COMMAND ${CMAKE_CTEST_COMMAND} -C benchmark -L benchmark)
  MESSAGE(STATUS "Building benchmarks")

  ADD_SUBDIRECTORY(benchmarks)
ENDIF(BUILD_BENCHMARKS)

# We use header-only parts of boost: boost::optional
FIND_PACKAGE(Boost REQUIRED)

# We ship the headers of Tick. Make it an imported target.
ADD_LIBRARY(Tick::Tick INTERFACE IMPORTED)
SET_PROPERTY(TARGET Tick::Tick PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/3rd-party/Tick")

# Make a target for wave_geometry
ADD_LIBRARY(wave_geometry INTERFACE)
TARGET_COMPILE_OPTIONS(wave_geometry INTERFACE -Wall -Wextra)
TARGET_LINK_LIBRARIES(wave_geometry INTERFACE Eigen3::Eigen Tick::Tick ${BOOST_LIBRARIES})
TARGET_INCLUDE_DIRECTORIES(wave_geometry INTERFACE include/)
