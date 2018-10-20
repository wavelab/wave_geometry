# Benchmarks of the example
# v2 = C1*C2*...*CN*v1
#
# Each file tests a different implementation (ceres autodiff, hand, wave) for different N.

find_package(GTSAM QUIET)
if(GTSAM_FOUND)
  add_executable(rotate_chain_gtsam_bench rotate_chain_gtsam_bench.cpp)
  target_link_libraries(rotate_chain_gtsam_bench gtsam benchmark::benchmark GTest::GTest)
  set_target_properties(rotate_chain_gtsam_bench PROPERTIES
          RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/benchmarks)
  target_include_directories(rotate_chain_gtsam_bench PRIVATE ${PROJECT_SOURCE_DIR}/include ${GTSAM_INCLUDE_DIR})
endif()

# Use C++17 for structured bindings
# Rotate_chain uses C++17
list (FIND CMAKE_CXX_COMPILE_FEATURES "cxx_std_17" index)
if (${index} GREATER -1)
  set(CMAKE_CXX_STANDARD 17)
else()
  message(STATUS "Not configuring rotate_chain benchmarks because (according to CMake) compiler does not support C++17")
  return()
endif()

wave_geometry_add_benchmark(rotate_chain_hand_bench rotate_chain_hand_bench.cpp)
wave_geometry_add_benchmark(rotate_chain_wave_bench rotate_chain_wave_bench.cpp)
wave_geometry_add_benchmark(rotate_chain_wave_untyped_bench rotate_chain_wave_untyped_bench.cpp)
wave_geometry_add_benchmark(rotate_chain_wave_reverse_bench rotate_chain_wave_reverse_bench.cpp)
wave_geometry_add_benchmark(rotate_chain_wave_dynamic_bench rotate_chain_wave_dynamic_bench.cpp)


wave_geometry_add_benchmark(imu_preint imu_preint.cpp)
