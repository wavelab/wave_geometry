# Benchmarks of the example
# v2 = C1*C2*...*CN*v1
#
# Each file tests a different implementation (ceres autodiff, hand, wave) for different N.

# Use C++17 for structured bindings
# Rotate_chain uses C++17
set(CMAKE_CXX_STANDARD 17)
list (FIND CMAKE_CXX_KNOWN_FEATURES "cxx_std_17" index)
if (${index} GREATER -1)

else()
  message(STATUS "Not configuring rotate_chain benchmarks because C++17 is not known to this version of CMake")
  return()
endif()

wave_add_benchmark(rotate_chain_hand_bench rotate_chain_hand_bench.cpp)
wave_add_benchmark(rotate_chain_wave_bench rotate_chain_wave_bench.cpp)
wave_add_benchmark(rotate_chain_wave_untyped_bench rotate_chain_wave_untyped_bench.cpp)
wave_add_benchmark(rotate_chain_wave_reverse_bench rotate_chain_wave_reverse_bench.cpp)

find_package(Ceres QUIET)
if(Ceres_FOUND)
  wave_add_benchmark(rotate_chain_ceres_bench rotate_chain_ceres_bench.cpp)
  target_link_libraries(rotate_chain_ceres_bench ceres)
  wave_add_benchmark(rotate_chain_ceres_quat_bench rotate_chain_ceres_quat_bench.cpp)
  target_link_libraries(rotate_chain_ceres_quat_bench ceres)
endif(Ceres_FOUND)

find_package(GTSAM QUIET)
if(GTSAM_FOUND)
  wave_add_benchmark(rotate_chain_gtsam_bench rotate_chain_gtsam_bench.cpp)
  target_link_libraries(rotate_chain_gtsam_bench gtsam)
endif(GTSAM_FOUND)