# Install

## Get the source code

The latest version of wave_geometry is available [on GitHub](https://github.com/wavelab/wave_geometry).

```bash
git clone https://github.com/wavelab/wave_geometry
```

## Dependencies

wave_geometry requires:
  * an existing installation of [Eigen](http://eigen.tuxfamily.org) 3.3.2 or above
  * an existing installation of Boost 1.58 or above (header-only libraries)
  * a C++17 compiler (tested on GCC 6.5, clang 5.0)

It has only been tested on Linux.

## Install on Linux
wave_geometry is a header-only library, meaning no compilation is required to use it in your project.
It can be installed with CMake.

```bash
cd wave_geometry
mkdir build
cd build
cmake .. -DBUILD_TESTING=OFF
sudo make install
```

Setting the `BUILD_TESTING` option to `ON` (the default) will build unit tests,
and `make test` will run them.

As an alternative to `make install`, the CMake option `-DEXPORT_BUILD=ON` will
make the build directory findable by other CMake projects without installation.

## Use in a CMake project

Once wave_geometry has been either installed or exported by CMake, it can be used in
your project's `CMakeLists.txt` file as follows:

```cmake
cmake_minimum_required(VERSION 3.8)
project(example)

set(CMAKE_CXX_STANDARD 17)

find_package(wave_geometry REQUIRED)

add_executable(example example.cpp)
target_link_libraries(example wave_geometry)
```

In your C++ file, write:
```cpp
#include <wave/geometry/geometry.hpp>
```
