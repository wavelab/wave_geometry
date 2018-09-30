# Make the GTest::GTest and GTest::Main targets available, either by finding them or
# by building GTest from a source package

find_package(GTest QUIET)
# FindGTest will define a proper-exported GTest library Target in CMake 3.5+
# Otherwise, don't risk incompatible flags, and don't use it
if(TARGET GTest::GTest)
  message(STATUS "Found GTest: ${GTEST_LIBRARIES}")
else(TARGET GTest::GTest)
  if(GTEST_FOUND)
    message(STATUS "Found GTest at ${GTEST_LIBRARIES}, but not using it due to possible compatibility issues")
  endif()

  # Try to build it from /usr/src (where debian package goes)
  find_path(GTEST_SRC_ROOT CMakeLists.txt
    HINTS ${GTEST_ROOT} $ENV{GTEST_ROOT}
    PATH /usr/src/googletest/googletest
    PATH /usr/src/gtest)

  if(NOT GTEST_SRC_ROOT)
    MESSAGE(FATAL_ERROR
      "Neither a GTest::GTest library nor a googletest source package was found.\n"
      "To continue without building tests, add -DBUILD_TESTING=OFF to CMake arguments.")
  else(NOT GTEST_SRC_ROOT)
    # Build it as part of this project
    set(GTEST_SRC_DIR ${GTEST_SRC_ROOT})
    set(GTEST_BINARY_DIR ${CMAKE_BINARY_DIR}/gtest)
    add_subdirectory(${GTEST_SRC_DIR} ${GTEST_BINARY_DIR})

    message(STATUS "Building GTest from ${GTEST_SRC_DIR}")

    # The project in GTEST_SRC_ROOT will probably build plain libraries, gtest and gtest_main
    # Check for modern target just in case
    if(NOT TARGET GTest::GTest)
      # Manually define modern targets
      # This needs the headers as well
      find_path(GTEST_INCLUDE_DIR gtest/gtest.h
        HINTS $ENV{GTEST_ROOT}/include ${GTEST_ROOT}/include)
      set_property(TARGET gtest PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${GTEST_INCLUDE_DIR})
      set_property(TARGET gtest_main PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${GTEST_INCLUDE_DIR})

      add_library(GTest::GTest ALIAS gtest)
      add_library(GTest::Main ALIAS gtest_main)
    endif(NOT TARGET GTest::GTest)
  endif(NOT GTEST_SRC_ROOT)
endif(TARGET GTest::GTest)
