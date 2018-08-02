# Benchmarks of the example
# v2 = C1*C2*...*CN*v1
#
# Each file tests a different implementation (ceres autodiff, hand, wave) for different N.

# Use C++17 for structured bindings
# Rotate_chain uses C++17
list (FIND CMAKE_CXX_KNOWN_FEATURES "cxx_std_17" index)
if (${index} GREATER -1)
  set(CMAKE_CXX_STANDARD 17)
else()
  message(STATUS "Not configuring rotate_chain benchmarks because C++17 is not known to this version of CMake")
  return()
endif()

wave_geometry_add_benchmark(imu_preint imu_preint.cpp)
