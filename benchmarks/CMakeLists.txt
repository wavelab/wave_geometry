find_package(benchmark REQUIRED)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# This target is used to build all benchmarks, without running them
ADD_CUSTOM_TARGET(benchmarks)

# wave_add_benchmark: Add a benchmark target
function(WAVE_ADD_BENCHMARK NAME SOURCE_FILE)
  add_executable(${NAME} ${SOURCE_FILE})
  target_link_libraries(${NAME} wave_geometry benchmark::benchmark GTest::GTest)
  add_test(NAME ${NAME} COMMAND ${NAME} CONFIGURATIONS benchmark)
  set_tests_properties(${NAME} PROPERTIES LABELS benchmark)

  # Build this target on "make benchmarks"
  ADD_DEPENDENCIES(benchmarks ${NAME})
endfunction(WAVE_ADD_BENCHMARK)

add_custom_target(benchmark
  COMMAND ${CMAKE_CTEST_COMMAND} -C benchmark -L benchmark)

wave_add_benchmark(util_cross_matrix_bench util_cross_matrix_bench.cpp)
wave_add_benchmark(util_identity_bench util_identity_bench.cpp)
wave_add_benchmark(imu imu.cpp)

add_subdirectory(rotate_chain)
