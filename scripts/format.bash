#!/bin/bash

# Usage: format.bash [path to search]
# clang-format will be called for each .hpp and .cpp file in the given path(s)
# The default is set below:
SEARCH_PATHS=${@:-"include test benchmarks"}

# Set the variable FORMAT_EXE to use a different version of clang-format
# The default is set below:
: ${FORMAT_EXE:=clang-format}

# Format all source files. Run from the source root directory.
find ${SEARCH_PATHS} -name "*.hpp" -o -name "*.cpp" | xargs ${FORMAT_EXE} -i
