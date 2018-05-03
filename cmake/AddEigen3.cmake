# Make the Eigen3::Eigen target available, either by finding a new version of Eigen or by
# wrapping an old version of Eigen.

find_package (Eigen3 REQUIRED)

if (NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_property(TARGET Eigen3::Eigen PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR})
  set_property(TARGET Eigen3::Eigen PROPERTY
    INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
endif (NOT TARGET Eigen3::Eigen)

message(STATUS "Found Eigen: ${EIGEN3_INCLUDE_DIR}")
