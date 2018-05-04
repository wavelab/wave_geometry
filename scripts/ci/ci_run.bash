#!/bin/bash
set -e  # exit on first error
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)"
CMAKE_CONFIG_DIR="../../cmake"  # cmake folder relative to this script

compile_libwave() {
    mkdir -p build
    cd build
    cmake .. -DCMAKE_MODULE_PATH=$DIR/$CMAKE_CONFIG_DIR \
        -DCMAKE_CXX_FLAGS="-Werror -Wno-error=deprecated-declarations"
    make -j2  # Travis gives two cores

    # (don't rely on nproc: https://github.com/travis-ci/travis-ci/issues/6859)
}

test_libwave() {
    export GTEST_COLOR=1
    ctest --output-on-failure
}

# MAIN
compile_libwave
test_libwave
