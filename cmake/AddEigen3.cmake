# Make the Eigen3::Eigen target available, either by finding a new version of Eigen or by
# wrapping an old version of Eigen.

# Try config-file search only
find_package(Eigen3 3.3.7 QUIET NO_MODULE)
if(Eigen3_FOUND)
  message(STATUS "Found Eigen: ${EIGEN3_INCLUDE_DIR}")
else()
  # Try using FindEigen3.cmake (needed for older distributions)
  find_package(Eigen3 3.3.7 REQUIRED)
endif()

# For older Eigen there may not be a Eigen3::Eigen target. Make one.
if (NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_property(TARGET Eigen3::Eigen PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR})
  set_property(TARGET Eigen3::Eigen PROPERTY
    INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
endif (NOT TARGET Eigen3::Eigen)
