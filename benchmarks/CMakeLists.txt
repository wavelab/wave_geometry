find_package(benchmark REQUIRED)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

wave_add_benchmark(util_cross_matrix_bench util_cross_matrix_bench.cpp)
wave_add_benchmark(util_identity_bench util_identity_bench.cpp)

add_subdirectory(rotate_chain)
