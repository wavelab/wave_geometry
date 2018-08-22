#!/bin/bash
set -e  # exit on first error
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)" # This script's directory

check_format() {
    # Use specifically clang-format-3.8 on CI
    export FORMAT_EXE=clang-format-3.8

    # Run clang-format on all files
    bash "${DIR}/../format.bash"
    # Check if it replaced anything, and print the files if so
    # This method easily prints nice output, but does rely on nothing else on CI
    # changing the files.
    if git diff-index --quiet HEAD --; then
        echo "clang-format check passed"
    else
        >&2 echo "The following files were changed after running clang-format:"
        >&2 git diff --stat
        >&2 echo "clang-format check failed"
        exit 1
    fi
}

# MAIN
check_format