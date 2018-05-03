cmake_minimum_required(VERSION 3.2)
project(wave_geometry)
set(CMAKE_CXX_STANDARD 11)

include(cmake/AddEigen3.cmake)

# We use header-only parts of boost: boost::optional
find_package(Boost REQUIRED)

# We ship the headers of Tick. Make it an imported target.
add_library(Tick::Tick INTERFACE IMPORTED)
set_property(TARGET Tick::Tick PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/3rd-party/Tick")

add_library(wave_geometry INTERFACE)
target_compile_options(wave_geometry INTERFACE -Wall -Wextra)
target_link_libraries(wave_geometry INTERFACE Eigen3::Eigen Tick::Tick ${BOOST_LIBRARIES})
target_include_directories(wave_geometry INTERFACE include/)

enable_testing()
add_subdirectory(test)
add_subdirectory(benchmarks)
